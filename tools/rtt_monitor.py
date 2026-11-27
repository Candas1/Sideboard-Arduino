#!/usr/bin/env python3
"""Live plot of the sideboard telemetry over SEGGER RTT (ST-Link + pyOCD).

The firmware (built with DEBUG_RTT) prints, after the debug command "p1":
    $H,ms,s1,s2,ax,...   header: field names, depends on the enabled features
    $T,1234,0,1,0.12,... one sample every TIME_TELEMETRY ms
Other lines are printed to the console. Lines typed in the console are sent
to the board (e.g. "?", "i", "o").

The RTT control block address is read from the firmware ELF (_SEGGER_RTT),
so it works with pyOCD's generic "cortex_m" target on GD32 and STM32 alike.

Examples:
    python tools/rtt_monitor.py --env GD32F130C8
    python tools/rtt_monitor.py --elf .pio/build/STM32F103C8/firmware.elf --csv log.csv
    python tools/rtt_monitor.py --rtt-address 0x20000000 --rtt-size 0x1000

Only one program can use the ST-Link at a time: stop PlatformIO's RTT/openocd
session (and the serial monitor) first.
"""

import argparse
import collections
import csv
import os
import queue
import sys
import threading
import time

GROUPS = [  # (title, fields): only groups with fields present in the header are shown
    ("Angles [deg]", ["roll", "pitch", "yaw"]),
    ("Gyro [rad/s]", ["gx", "gy", "gz"]),
    ("Accel [m/s²]", ["ax", "ay", "az"]),
    ("Sent: commands", ["tx_cmd1", "tx_cmd2"]),
    ("Sent: pitch [deg*100] / dPitch [raw]", ["tx_pitch", "tx_dpitch"]),
    ("Received: speed", ["rx_speedR", "rx_speedL"]),
    ("Received: battery / temp [raw]", ["rx_bat", "rx_temp"]),
    ("Sensors", ["s1", "s2"]),
]

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def find_rtt_symbol(elf_path):
    from elftools.elf.elffile import ELFFile  # pyelftools, installed with pyocd
    with open(elf_path, "rb") as f:
        symtab = ELFFile(f).get_section_by_name(".symtab")
        syms = symtab.get_symbol_by_name("_SEGGER_RTT") if symtab else None
        if not syms:
            sys.exit(f"_SEGGER_RTT not found in {elf_path} (built with DEBUG_RTT?)")
        return syms[0]["st_value"]


class Telemetry:
    """Parsed samples shared between the RTT thread and the plot."""

    def __init__(self, maxlen, csv_path):
        self.lock = threading.Lock()
        self.fields = None
        self.rows = collections.deque(maxlen=maxlen)
        self.header_event = threading.Event()
        self._csv_file = open(csv_path, "w", newline="") if csv_path else None
        self._csv = csv.writer(self._csv_file) if self._csv_file else None

    def handle_line(self, line):
        if line.startswith("$H,"):
            fields = line[3:].split(",")
            with self.lock:
                if self.fields and fields != self.fields:
                    print("Header changed, restart the monitor to plot new fields")
                    return
                self.fields = fields
            if self._csv and not self.header_event.is_set():
                self._csv.writerow(fields)
            self.header_event.set()
        elif line.startswith("$T,"):
            if not self.fields:
                return
            try:
                values = [float(v) for v in line[3:].split(",")]
            except ValueError:
                return  # truncated line (RTT buffer overflow)
            if len(values) != len(self.fields):
                return
            with self.lock:
                self.rows.append(values)
            if self._csv:
                self._csv.writerow(values)
        else:
            print(line)

    def close(self):
        if self._csv_file:
            self._csv_file.close()


def rtt_worker(args, address, size, telem, commands, stop):
    from pyocd.core.helpers import ConnectHelper
    from pyocd.debug.rtt import RTTControlBlock

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=args.uid, target_override=args.target,
        connect_mode="attach", blocking=False, frequency=args.frequency)
    if session is None:
        print("No debug probe found")
        stop.set()
        return
    try:
        with session:
            target = session.board.target
            cb = RTTControlBlock.from_target(target, address=address, size=size)
            cb.start()
            up = cb.up_channels[0]
            down = cb.down_channels[0] if cb.down_channels else None
            if down is None:
                print("No RTT down channel: commands can't be sent")
            else:
                down.write(b"p1\n")  # enable telemetry, firmware answers with $H
            print(f"RTT connected (control block 0x{address:08x}). Type commands, close the plot to quit.")

            pending, out = b"", b""
            while not stop.is_set():
                data = up.read()
                if data:
                    pending += data
                    *lines, pending = pending.split(b"\n")
                    for raw in lines:
                        telem.handle_line(raw.decode(errors="replace").rstrip("\r"))
                while not commands.empty():
                    out += commands.get()
                if out and down:
                    out = out[down.write(out):]
                if not data:
                    time.sleep(0.005)
            if down:
                down.write(b"p0\n")  # leave the board quiet
    except Exception as e:  # probe unplugged, control block not found, ...
        print(f"RTT error: {e}")
    finally:
        stop.set()


def stdin_reader(commands, stop):
    for line in sys.stdin:
        if stop.is_set():
            break
        commands.put(line.rstrip("\r\n").encode() + b"\n")


def run_plot(args, telem, stop):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    if not telem.header_event.wait(timeout=10):
        print("No telemetry header received (firmware built with DEBUG_RTT? 'p' command present?)")
        stop.set()
        return

    fields = telem.fields
    groups = [(t, [f for f in fs if f in fields]) for t, fs in GROUPS]
    groups = [(t, fs) for t, fs in groups if fs]
    idx = {f: i for i, f in enumerate(fields)}

    fig, axes = plt.subplots(len(groups), 1, sharex=True, squeeze=False,
                             figsize=(10, 1.8 * len(groups) + 1))
    axes = axes[:, 0]
    lines = {}
    for ax, (title, fs) in zip(axes, groups):
        ax.set_title(title, fontsize=9, loc="left")
        ax.grid(True, alpha=0.3)
        for f in fs:
            (lines[f],) = ax.plot([], [], label=f, linewidth=1)
        ax.legend(loc="upper left", fontsize=8)
    axes[-1].set_xlabel("time [s]")
    fig.canvas.manager.set_window_title("Sideboard RTT monitor")
    fig.tight_layout()

    def update(_):
        if stop.is_set():
            plt.close(fig)
            return []
        with telem.lock:
            rows = list(telem.rows)
        if not rows:
            return []
        t = [r[idx["ms"]] / 1000.0 for r in rows]
        t_end = t[-1]
        for f, line in lines.items():
            line.set_data(t, [r[idx[f]] for r in rows])
        for ax in axes:
            ax.set_xlim(t_end - args.window, t_end)
            ax.relim()
            ax.autoscale_view(scalex=False)
        return list(lines.values())

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841 (keep a reference)
    plt.show()
    stop.set()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group()
    src.add_argument("--env", help="PlatformIO env: uses .pio/build/<env>/firmware.elf")
    src.add_argument("--elf", help="firmware ELF to read the _SEGGER_RTT address from")
    p.add_argument("--rtt-address", type=lambda s: int(s, 0), help="RTT control block (search) address")
    p.add_argument("--rtt-size", type=lambda s: int(s, 0), help="search range from --rtt-address")
    p.add_argument("--target", default="cortex_m", help="pyOCD target (default: generic cortex_m)")
    p.add_argument("--uid", help="probe unique ID (if several are connected)")
    p.add_argument("--frequency", type=int, help="SWD frequency [Hz]")
    p.add_argument("--window", type=float, default=10.0, help="plotted time window [s]")
    p.add_argument("--csv", help="also save samples to this CSV file")
    args = p.parse_args()

    address, size = args.rtt_address, args.rtt_size
    elf = args.elf or (os.path.join(REPO, ".pio", "build", args.env, "firmware.elf") if args.env else None)
    if address is None:
        if elf:
            address, size = find_rtt_symbol(elf), None  # exact address, no search
        else:
            address, size = 0x20000000, 0x1000  # smallest RAM of the supported chips (GD32F130C6: 4 KB)
            print("No --env/--elf: searching RTT in the first 4 KB of RAM")

    telem = Telemetry(maxlen=int(args.window * 1000 / 5), csv_path=args.csv)
    commands, stop = queue.Queue(), threading.Event()
    worker = threading.Thread(target=rtt_worker, args=(args, address, size, telem, commands, stop), daemon=True)
    worker.start()
    threading.Thread(target=stdin_reader, args=(commands, stop), daemon=True).start()
    try:
        run_plot(args, telem, stop)
    except KeyboardInterrupt:
        stop.set()
    worker.join(timeout=3)  # let the worker send "p0" and close the session
    telem.close()


if __name__ == "__main__":
    main()
