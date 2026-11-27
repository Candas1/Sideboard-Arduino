#!/usr/bin/env python3
"""Flash use per component from a GNU ld map file.

Usage: python tools/size_report.py .pio/build/<env>/firmware.map [title]

Build with the map enabled, e.g.:
    PLATFORMIO_BUILD_FLAGS="-Wl,-Map,.pio/build/<env>/firmware.map" pio run -e <env>

Counts input sections that end up in flash (code, constants, .data init
values, vectors); RAM-only (.bss, .noinit, COMMON) and debug sections are
skipped. Prints a markdown table, largest first.
"""

import collections
import re
import sys

LINE = re.compile(r"^\s*(\S+)?\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(\S.*)$")
RAM_ONLY = (".bss", ".noinit", "COMMON", ".heap", "._user_heap_stack", ".stack")


def component(path):
    p = path.replace("\\", "/")
    m = re.search(r"/(lib(gcc|m|c|c_nano|nosys|stdc\+\+|stdc\+\+_nano|supc\+\+|supc\+\+_nano))\.a\(", p)
    if m:
        name = m.group(1)
        return {"libgcc": "libgcc (soft-float, int div)", "libm": "libm (math)"}.get(name, name)
    m = re.search(r"/\.pio/build/[^/]+/lib[^/]*/([^/]+)/", p) or re.search(r"/\.pio/libdeps/[^/]+/([^/]+)/", p)
    if m:
        return m.group(1)
    m = re.search(r"/libraries/([^/]+)/", p)  # framework libraries (Wire, SPI, SrcWrapper...)
    if m:
        return m.group(1)
    if "/.pio/build/" in p and "/src/" in p:
        return "sideboard src"
    if "FrameworkArduinoVariant" in p:
        return "core: variant"
    if "FrameworkArduino" in p or "framework-arduino" in p or "libFrameworkArduino" in p:
        return "core"
    if "crt" in p.rsplit("/", 1)[-1]:
        return "startup (crt)"
    return p.rsplit("/", 1)[-1]


def parse(map_path):
    sizes = collections.Counter()
    in_map = False
    pending = None  # section name on its own line, address/size on the next
    with open(map_path, errors="replace") as f:
        for line in f:
            if not in_map:
                in_map = line.startswith("Linker script and memory map")
                continue
            if line.startswith("OUTPUT(") or line.startswith("LOAD linker stubs"):
                break
            m = LINE.match(line)
            if not m:
                s = line.strip()
                pending = s if s.startswith(".") and " " not in s else None
                continue
            name, size, path = (m.group(1) or pending), int(m.group(3), 16), m.group(4).strip()
            pending = None
            if not name or size == 0 or name.startswith("*fill*"):
                continue
            if name.startswith(".debug") or name.startswith(".comment") or name.startswith(".ARM.attributes"):
                continue
            if name.startswith(RAM_ONLY):
                continue
            if not (path.endswith(")") or path.endswith(".o")):
                continue  # symbol lines ("0x... funcname") and linker-script expressions
            sizes[component(path)] += size
    return sizes


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sizes = parse(sys.argv[1])
    total = sum(sizes.values())
    title = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1]
    print(f"### Flash by component: {title}\n")
    print("| Component | Bytes | % |\n|---|---:|---:|")
    for name, size in sizes.most_common():
        print(f"| {name} | {size} | {100.0 * size / total:.1f} |")
    print(f"| **total** | **{total}** | |\n")


if __name__ == "__main__":
    main()
