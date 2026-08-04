# CLAUDE.md

Guidance for Claude Code when working in this repository.
Read this file fully before writing any code.

---

## What this project is

**The Interface** — a two-participant biofeedback art installation running on a single Arduino Uno R4 Minima.
Each participant wears a heart rate sensor, a vibration motor (rumble pack), and has a 44-LED strip.
The LED strips respond to each participant's live heartrate and, when the two heartrates come within
~5–10 BPM of each other, the system nudges them into sync. The whole thing is self-contained
(battery-powered, no external computer).

For the full design rationale see `DESIGN.md`.

---

## Board

**Arduino Uno R4 Minima** (Renesas RA4M1). Only one program runs at a time — motor, LED, and
sensor logic all share a single `setup()`/`loop()`.

---

## Hardware

### Components

| Component | Part | Notes |
|---|---|---|
| Heart sensor ×2 | MAX30102 | I2C address `0x57` on both — put on separate buses |
| Motor driver ×2 | DRV8833 | One H-bridge per rumble pack; IN2 hardwired to GND, only IN1 is driven |
| I2C repeater ×2 | PCA9515A | Not currently in circuit — see Known Issues |
| LED strips ×2 | TBD (44 LEDs each) | Type not yet confirmed; WS2812B assumed |

### Pin assignments

| Pin | Function | Wire colour |
|---|---|---|
| A0 | Button — cycle LED patterns | — |
| A2 | Potentiometer (green) | — |
| A4 | **DO NOT USE as analog** (= SDA on Uno R4 hardware I2C) | — |
| A5 | **DO NOT USE as analog** (= SCL on Uno R4 hardware I2C) | — |
| D6 | Motor Driver 1 IN1 | Green |
| D7 | LED strip 1 data | Orange |
| D8 | Sensor 1 SDA (SoftWire) | Blue |
| D9 | Sensor 1 SCL (SoftWire) | Yellow |
| D12 | LED strip 2 data | Orange |
| D13 | Motor Driver 2 IN1 | Green |
| SDA (A4) | Sensor 2 SDA | Blue |
| SCL (A5) | Sensor 2 SCL | Yellow |

Sensor wiring colours: SCL=Yellow, SDA=Blue, GND=Green, PWR=Red.

### Open hardware unknowns

- **LED strip type** — WS2812B assumed; confirm before writing LED library calls.

---

## Critical I2C rule — use SoftWire, never Wire

The Arduino hardware I2C API (`Wire.begin()`, `requestFrom()`, `beginTransmission()`) causes
intermittent bus lockups on this hardware. **All I2C must go through `SoftWire`.**

Sensor 1 and Sensor 2 both have address `0x57`. They live on separate SoftWire buses:
- Sensor 1: SoftWire on D8 (SDA), D9 (SCL)
- Sensor 2: SoftWire on A4 (SDA), A5 (SCL)

Never share the two sensors on the same bus.

---

## Dependencies (Arduino Library Manager)

| Library | Purpose |
|---|---|
| `SoftWire` | Bit-bang I2C for both sensors |
| `Adafruit NeoPixel` or FastLED | LED strips (TBD on which) |

---

## Building and flashing

```bash
# Compile (replace board/port as needed)
arduino-cli compile --fqbn arduino:renesas_uno:minima theinterface.ino

# Upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:renesas_uno:minima theinterface.ino

# Serial monitor
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

For the hardware test harness, compile the `HardwareTest/` sketch instead:
```bash
arduino-cli compile --fqbn arduino:renesas_uno:minima --library . HardwareTest
```
(The `--library .` flag lets HardwareTest find InterfaceSensor in the project root.)

---

## File map

| File | Purpose |
|---|---|
| `InterfaceSensor.h` / `.cpp` | MAX30102 sensor library — complete and working |
| `theinterface.ino` | Main sketch — two-participant serial visualizer with runtime calibration and BPM/HRV/SpO2 tracking |
| `HardwareTest/HardwareTest.ino` | Serial menu-driven hardware test for all components |
| `DESIGN.md` | Human-readable design doc (goals, LED behaviour, task order) |

---

## InterfaceSensor API

```cpp
InterfaceSensor sensor(sdaPin, sclPin);

sensor.begin()         // init SoftWire, verify part ID (0x15) — returns bool
sensor.setupSensor()   // reset chip, configure SpO2 mode 100 Hz 411µs 4× avg — returns bool
sensor.setIRLedAmplitude(value)  // set the IR LED current (register 0x0D)
sensor.setRedLedAmplitude(value) // set the Red LED current (register 0x0C)
sensor.readFIFO(red, ir)  // burst-read 6 bytes → two 18-bit values
sensor.getIR()         // returns one IR sample, or -1 on error
sensor.getFIFOCount()  // (WR_PTR - RD_PTR) & 0x1F
sensor.getOverflowCount() // OVF_COUNTER (register 0x05) — samples lost since last read
sensor.dumpConfig()    // print key registers to Serial (debug)
sensor.readRegister(reg) / writeRegister(reg, val)
```

MAX30102 FIFO registers: WR_PTR `0x04`, OVF `0x05`, RD_PTR `0x06`, DATA `0x07`.
Part ID at `0xFF` must equal `0x15`.

---

## LED behaviour specification

| Mode | Behaviour |
|---|---|
| Baseline | Twinkling / ambient |
| Heartbeat pulse | Brightness dips up on each detected beat |
| Near-sync (5–10 BPM delta) | Strips visually reflect closeness; system begins nudging |
| Full sync | Both strips pulse together at the shared "flow state" BPM |

Each strip is independent; strip 1 = participant 1's heartrate, strip 2 = participant 2's.

---

## Task order (dependencies)

1. **Validate raw IR/Red → BPM/HRV/SpO2** — the main sketch now has a per-person `no contact → calibrating → tracking` pipeline.
   - Calibration independently adjusts that sensor's IR and Red LED current and locks both before tracking starts.
   - Tracking uses a DC-removed IR waveform, adaptive per-person threshold, plausible beat intervals, and a rolling eight-interval BPM average.
   - HRV is RMSSD over the same eight-interval buffer, in ms.
   - SpO2 is a ratio-of-ratios estimate from Red/IR AC-DC levels using an uncalibrated empirical formula — a biofeedback cue, not a medical reading.
   - A nonzero `OVF_COUNTER` forces a channel back to `NO_CONTACT` so lost FIFO samples never silently desync the timing these metrics depend on.
   - Validate the displayed BPM against a manually counted pulse before treating it as ready for the installation.

2. **Motor modulation** — implemented. Each detected beat triggers a non-blocking linear
   ramp up/down (`HeartChannel::motorIntensity()`), driven via `analogWrite` on D6/D13
   with IN2 hardwired to GND. Peak intensity is a fixed constant for now; potentiometer
   control is not wired in yet.

3. **LED coherence logic** — async dual-strip, brightness pulse on beat, sync logic when
   BPM delta ≤ 5–10.

---

## Known issues / decisions

- PCA9515A repeaters tried and abandoned — they cut out intermittently (possibly power
  differentials). Physically removed from the circuit; hardcoded SoftWire on D8/D9 and
  A4/A5 is stable without them.
- Hardware I2C Wire API caused failures; SoftWire fixed it.
- `getIR()` does not guard against an empty FIFO — always call `getFIFOCount()` first.
- The visualizer's automatic calibration targets roughly 90,000–225,000 IR counts. It recalibrates when contact is lost; no reflash is needed for a new participant.
- FIFO_CFG enables 4-sample averaging, so the MAX30102 pushes a new FIFO entry every
  40 ms even though it samples internally at 100 Hz. `SAMPLE_PERIOD_MS` in
  `theinterface.ino` must track this (40 ms), not the raw 100 Hz sample rate — using
  10 ms understated real beat intervals ~4x and made valid beats fail the
  `MIN_BEAT_INTERVAL_MS` plausibility check.

---

## HardwareTest usage (quick reference)

Flash `HardwareTest/HardwareTest.ino`, open Serial Monitor at 115200 baud, send:

```
1  LED strip 1 (D7)       — cycles R/G/B/W, all 44 LEDs
2  LED strip 2 (D12)      — same
3  Motor 1 (D6)           — three vibration steps
4  Motor 2 (D13)          — same
5  Sensor 1 (D8/D9)       — 10 IR readings; finger on sensor → expect >50 000
6  Sensor 2 (A4/A5)       — same
7  Button (A0)            — blocks until press + release
8  Potentiometer (A2)     — 10 readings; turn pot to verify 0–1023 range
a  All tests in sequence
h  Menu
```
