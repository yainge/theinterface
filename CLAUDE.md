# CLAUDE.md

Guidance for Claude Code when working in this repository.
Read this file fully before writing any code. Where this file contradicts DESIGN.md, **this file wins**.

---

## What this project is

**The Interface** — a two-participant biofeedback art installation on a single Arduino UNO R4 Minima.
Each participant has a heart rate sensor, a vibration motor (rumble pack), and a 44-LED strip.
The strips and motors respond to each participant's live heartrate.
Sync/nudge logic (aligning two heartrates toward a shared flow-state BPM) is **deferred — not in scope yet.**

See `DESIGN.md` for the human-readable version.

---

## Locked decisions (authoritative — override anything else in the repo)

| Decision | Value |
|---|---|
| Board | Arduino UNO R4 Minima (Renesas RA4M1, 48 MHz, 32 KB RAM) |
| FQBN | `arduino:renesas_uno:minima` |
| Board package | "Arduino UNO R4 Boards" |
| LEDs | WS2812-family, Adafruit NeoPixel library, GRB colour order |
| Motor IN2 | Hardwired to GND — one Arduino pin (IN1 only) per motor |
| PCA9515A | Not installed. Physically absent. Ignore it. |
| I2C API | SoftWire only — Wire hardware API abandoned (causes lockups) |
| Sensor 1 bus | SoftWire(8, 9) — D8=SDA, D9=SCL |
| Sensor 2 bus | SoftWire(A4, A5) — A4=SDA, A5=SCL |
| Deadline | August 9th — single-person heartbeat → LED + motor pulse first |

---

## Board

**Arduino UNO R4 Minima** (Renesas RA4M1). Not the ATmega328P classic Uno.

```bash
# Compile
arduino-cli compile --fqbn arduino:renesas_uno:minima theinterface.ino

# Upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:renesas_uno:minima theinterface.ino

# Serial monitor
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200

# Compile test harness (--library . lets HardwareTest find InterfaceSensor in project root)
arduino-cli compile --fqbn arduino:renesas_uno:minima --library . HardwareTest
```

---

## Hardware

### Pin assignments

| Pin | Function | Wire colour |
|---|---|---|
| A0 | Button — cycle LED patterns | — |
| A2 | Potentiometer (motor intensity) | — |
| A4 | Sensor 2 SDA (SoftWire) — **not an analog input** | Blue |
| A5 | Sensor 2 SCL (SoftWire) — **not an analog input** | Yellow |
| D6 | Motor 1 IN1 (DRV8833) | Green |
| D7 | LED strip 1 data | Orange |
| D8 | Sensor 1 SDA (SoftWire) | Blue |
| D9 | Sensor 1 SCL (SoftWire) | Yellow |
| D12 | LED strip 2 data | Orange |
| D13 | Motor 2 IN1 (DRV8833) | Green |

Sensor wiring: SCL=Yellow, SDA=Blue, GND=Green, PWR=Red.

### Motor wiring (DRV8833)

IN2 is hardwired to GND on both boards. Speed control = PWM on IN1 only.
Heartbeat feel = ramp PWM up at beat onset, coast to zero. No second control pin.

---

## Critical I2C rule — SoftWire only, never Wire

The hardware I2C API (`Wire.begin()`, `requestFrom()`, `beginTransmission()`) causes
intermittent bus lockups. Do not use it under any circumstances.

Both MAX30102 sensors share address `0x57`. They live on separate SoftWire buses:
- Sensor 1: `SoftWire(8, 9)`
- Sensor 2: `SoftWire(A4, A5)`

Never share them on the same bus.

---

## Known contradiction — FIFO read strategy (resolve empirically, do not fix blind)

There is a disagreement between notes and code about the correct FIFO read strategy:

- **Current code**: gates every read on `getFIFOCount() > 0`; FIFO config `0x5F` (rollover enabled).
- **Alternative note**: skip `getFIFOCount()`, call `readFIFO()` directly; FIFO config `0x4F` (rollover disabled).
- **Symptom of the bug**: IR value freezes on a repeating number for >2–3 seconds.

**Resolution protocol** (human must run this):
Run the single-sensor IR loop (~5 min) with a finger on the sensor. Watch the OVF register.
- IR stable, OVF stays 0 → current code is correct, leave it.
- IR freezes on a repeating value → apply the fix: set FIFO config to `0x4F`, remove `getFIFOCount()` gate.

Do not change the FIFO strategy until this test has been run and the result reported.

---

## Dependencies (Arduino Library Manager)

| Library | Purpose |
|---|---|
| `SoftWire` | Bit-bang I2C for both sensors |
| `Adafruit NeoPixel` | LED strips (WS2812, GRB) |

---

## File map

| File | Purpose |
|---|---|
| `InterfaceSensor.h` / `.cpp` | MAX30102 library — complete and working |
| `theinterface.ino` | Main sketch — single-sensor IR loop; needs full build |
| `HardwareTest/HardwareTest.ino` | Serial menu-driven hardware test for all components |
| `DESIGN.md` | Human-readable design doc |

---

## Planned architecture (designed in Claude.ai, not yet implemented)

This seam lets motor/LED code be built and tested without a live working sensor:

| Component | Role |
|---|---|
| `BeatClock` | Phase clock — tracks position in the current beat cycle (0.0–1.0) |
| `SimSource` | Fake heartbeat — feeds BeatClock at a configurable BPM for offline testing |
| `RumbleRenderer` | Consumes BeatClock, outputs PWM envelope on motor IN1 |
| `LedRenderer` | Consumes BeatClock, outputs per-strip brightness pulse to NeoPixel |

Implementation order: SimSource → BeatClock first, confirm motors and LEDs respond, then swap in the real sensor.

---

## InterfaceSensor API

```cpp
InterfaceSensor sensor(sdaPin, sclPin);

sensor.begin()                 // init SoftWire, verify part ID (reg 0xFF == 0x15) → bool
sensor.setupSensor()           // reset, configure SpO2 100 Hz 411µs 4× avg → bool
sensor.getFIFOCount()          // (WR_PTR - RD_PTR) & 0x1F
sensor.getIR()                 // one IR sample or -1 on error
sensor.readFIFO(red, ir)       // both channels, 18-bit each
sensor.dumpConfig()            // print key registers to Serial
sensor.readRegister(reg)       // single-byte read
sensor.writeRegister(reg, val) // single-byte write
```

MAX30102 FIFO registers: WR_PTR `0x04`, OVF `0x05`, RD_PTR `0x06`, DATA `0x07`.
Part ID at `0xFF` must equal `0x15` for `begin()` to succeed.

---

## LED behaviour specification

| State | LED behaviour |
|---|---|
| Idle / no reading | Slow twinkling ambient |
| Heartbeat detected | Brightness pulses up on each beat |
| Near-sync (5–10 BPM delta) | **Deferred** |
| Full sync | **Deferred** |

Each strip is independent: strip 1 = participant 1's heartrate, strip 2 = participant 2's.

NeoPixel `show()` briefly disables interrupts. SoftWire is bit-banged in the loop.
Do not call them simultaneously. Verify combined loop timing once both sensors and strips run together.

---

## Task order

1. **Human runs HardwareTest** — all 8 tests pass, IR > 50 000 with finger on sensor, LED colours correct under NeoPixel GRB. This is the gate before any further code work.
2. **Sensor soak test** — 5 min IR loop, watch OVF, resolve the FIFO contradiction above.
3. **IR → BPM** — peak detector on 100 Hz IR stream → inter-beat intervals → smoothed BPM. Treat IR ≤ 0 as invalid (coast, do not act). Validate against a real reference (phone HR app).
4. **Motor pulse** — per-beat PWM ramp on IN1, intensity from A2 potentiometer.
5. **LED pulse** — per-strip brightness pulse on each detected beat; dual strips independent.
6. **Dual sensor** — add second InterfaceSensor on A4/A5; run both channels.
7. **Sync logic** — deferred, out of scope for August 9th deadline.

---

## Known issues / gotchas

- `getIR()` does not guard against an empty FIFO — always call `getFIFOCount()` first (pending FIFO contradiction resolution above).
- A4/A5 are Sensor 2 SoftWire pins — never call `analogRead()` on them.
- Main loop must be cooperative and non-blocking. No `delay()` once sensors and actuation run together.
- PCA9515A repeaters: not installed, not in circuit. Do not mention or reference them in code.

---

## HardwareTest usage (quick reference)

Flash `HardwareTest/HardwareTest.ino`. Open Serial Monitor at 115200 baud. Send one character:

```
1  LED strip 1 (D7)       — cycles R/G/B/W, all 44 LEDs
2  LED strip 2 (D12)      — same
3  Motor 1 (D6)           — three vibration steps (25/50/100%)
4  Motor 2 (D13)          — same
5  Sensor 1 (D8/D9)       — 10 IR readings; finger on sensor → expect > 50 000
6  Sensor 2 (A4/A5)       — same
7  Button (A0)            — blocks until press + release
8  Potentiometer (A2)     — 10 readings; turn pot to verify 0–1023
a  All tests in sequence
h  Menu
```

**First question to ask after flashing**: "Which numbered tests passed, and did the LED colours look correct?"
That result decides whether we debug hardware or go straight to the BPM peak detector.
