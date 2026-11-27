# Hardware test plan (dev branch)

Everything on `dev` builds in CI, but **nothing has been tested on hardware yet**.
This file is the checklist for the first test session, and the place to write the
results down (fill the tables, commit them). It is written so that someone, or a
Claude session, without the history of the earlier work can follow it.

## Setup

- PlatformIO (VS Code), an ST-Link V2 with a working driver, the sideboard on SWD
  (3.3 V, SWCLK, SWDIO, GND).
- **Power:** SWD 3.3 V only for the first tests. On a hoverboard the sideboard
  connector also carries **15 V (max. 200 mA, shared with the board itself)**: connect
  only GND/TX/RX to anything else. Motors only with someone present.
- RTT monitor: `pip install -r tools/requirements.txt` (pyOCD, matplotlib).
- Only one program can use the ST-Link at a time: close the PlatformIO RTT task /
  serial monitor before `rtt_monitor.py`, and the other way round.

Features are enabled in `platformio.ini`, section `[features]` (comment a line to
disable). Default: `DEBUG_RTT`, `HOVER_SERIAL`. `TEST` is off on purpose: it makes the
mainboard's motors move (±50).

## Debug console (RTT)

PlatformIO: run the "RTT Start" task, then the serial monitor (`socket://localhost:9090`),
or use `python tools/rtt_monitor.py --env <env>` (it forwards typed lines).

| Command | Effect |
|---|---|
| `?` | list the commands |
| `p` / `p1` / `p0` | telemetry toggle / on / off (`$H,...` header, then `$T,...` every 20 ms) |
| `i` / `o` | print hoverserial frames received / sent (with `HOVER_SERIAL`) |
| `r` | print remote codes (with `REMOTE`) |
| `w` | read the IMU WHO_AM_I registers (with an IMU) |

## Tests

Write the env (`GD32F130C6`, `GD32F130C8`, `STM32F103C6`, `STM32F103C8`), the result
(✅ / ❌ / ⚠️) and notes. Board markings (MCU, IMU chip) help.

### 1. Boot, LEDs, console (default features)

1. Flash with the default `[features]`.
2. At power-up all 5 LEDs are on for 0.5 s, then off.
3. No mainboard connected: **yellow LED blinks** (no feedback, ~5 Hz).
4. Console: `?` lists the commands; the boot message `Hoverboard Serial v1.0` is in the RTT buffer.

### 2. RTT monitor

1. `python tools/rtt_monitor.py --env <env>` (reads the RTT address from `.pio/build/<env>/firmware.elf`).
2. Expected: "RTT connected", a plot window with "Sent: commands", "Sensors"...
3. Press the optical sensors: `s1` / `s2` change in the plot.
4. Type `?` in the terminal: the command list is printed.
5. Close the window: telemetry stops (`p0` sent).
6. If it fails on GD32 with the generic target, note the error (try `--target stm32f103rc` on STM32).
7. If the plot stutters / lines are dropped: note it (fix: raise `TIME_TELEMETRY` or `BUFFER_SIZE_UP`).

### 3. Mainboard link (HOVER_SERIAL)

Mainboard with hoverboard-firmware-hack-FOC, sideboard on its sideboard connector.

1. Yellow stops blinking; LEDs follow the mainboard (`cmdLed`).
2. Telemetry: `rx_frames` increases, `rx_bat` / `rx_temp` plausible.
3. Unplug the cable (or power the mainboard off): within ~0.6 s yellow blinks again.
4. `o`: frames sent every 100 ms; `tx_sensors` bit 0/1 follow the optical sensors.

### 4. IMU identification (most important for the next steps)

Enable `-D MPU6050` (keep `DEBUG_RTT`). Flash and read the boot output, or type `w`.
Repeat for **every board / IMU variant you have**.

- `IMU @0x68: reg 0x75=0x?? reg 0x00=0x??` → note both values.
- `no ACK` at both addresses → bus/pin problem (e.g. Wire not on PB6/PB7 on GD32), not the chip.
- Then: `IMU found!` or `Failed to find IMU chip` (Adafruit only accepts WHO_AM_I 0x68).

| Env | IMU marking | Addr | reg 0x75 | reg 0x00 | `begin()` | Notes |
|---|---|---|---|---|---|---|
| | | | | | | |

### 5. IMU data and pitch (MPU6050 + AHRS)

Enable `-D MPU6050` and `-D AHRS` (C8 boards; on STM32F103C6 this does not fit in flash yet).

1. `rtt_monitor.py`: tilt the board, `pitch` / `roll` follow, `gx..gz` react to rotation, `az` ≈ 9.8 at rest.
2. `tx_pitch` = pitch × 100, `tx_dpitch` follows the rotation speed, `tx_sensors` bit 2 = 1 (IMU OK).
3. IMU not found: green off, **red blinks** (with no mainboard link).
4. Optional: measure the filter cost (time `filter.updateIMU()` with `micros()`).

### 6. Remote (REMOTE, boards with a 433 MHz receiver)

1. Enable `-D REMOTE`, type `r`, press the remote buttons: codes are printed.
2. Update `REMOTE_BUTTON1..4` in `include/config.h` if needed.

### 7. Risk from PR #16 (libraries linked as archives)

Unused library code is no longer linked. If something silently stopped working
compared with older builds (LEDs, UART, remote interrupt, IMU, RTT), note it here.

## Hardware checks (multimeter)

| Check | How | Result |
|---|---|---|
| LED transistors, marked "G1" | diode test (NPN?), look up the marking (maybe MMBT5551, not verified) | |
| LED supply voltage | measure at the LED side (expected ~15 V from the sideboard connector) | |
| 78M05 input | measure | |
| CC2541 board only: UART to the MCU | continuity from the CC2541 TX/RX to MCU pins | |

## Results log

| Date | Env | Test | Result | Notes |
|---|---|---|---|---|
| | | | | |

## Next steps after the tests (context)

- **IMU plan:** raw-register driver for the MPU6050 family (WHO_AM_I at 0x75:
  MPU6050, 6050A/C, 6052C, ICM-20689; the ICM-20618/20948 family uses 0x00 and
  banked registers), fixed-point complementary filter (pitch/roll in deg×100), integer
  telemetry. Goal: drop Madgwick + soft-float + libm (~10 KB) so IMU + AHRS fits on
  STM32F103C6. No DMP (owner's choice).
- **Flash:** each CI run has a size table per component and a `size-summary` job
  comparing with `dev`. stm32duino#3117 (merged upstream, unreleased) saves ~2.1 KB
  more on STM32 once released.
- **Kept on purpose:** `MPU_SCL`/`MPU_SDA` in `defines.h` (not wired to `Wire` yet).
- **Ideas:** LED pins all have timer PWM (both MCUs) for dimming or driving
  H-bridge/BLDC driver inputs (tap the MCU side, the LED outputs are 15 V low-side);
  CC2541 as a BLE serial link (Web Bluetooth).
