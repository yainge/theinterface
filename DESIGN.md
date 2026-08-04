# The Interface — Design Document

A two-participant biofeedback installation. Each person wears a heart rate sensor.
The system reads both heartrates and uses light and vibration to create a shared sensory
experience, nudging both participants toward a shared "flow state" rhythm.

---

## Hardware overview

One Arduino Uno R4 Minima runs everything. Each participant has:

- **Heart sensor** (MAX30102) — reads pulse via infrared light through skin
- **Vibration motor** (DRV8833 driver) — rumble pack, tuned to feel like a heartbeat
- **LED strip** — 44 lights that respond to that participant's heartrate

Shared inputs:
- **Button** (A0) — cycles through LED patterns
- **Potentiometer** (A2) — tunes motor intensity

---

## What the system does

### Current heart-rate visualizer

The main sketch provides a two-participant development visualizer over USB serial. Each participant is handled independently:

1. **No contact** — the channel reports no BPM/HRV/SpO2.
2. **Calibrating** — the software adjusts that sensor's IR and Red LED current independently until each signal is in a useful range, then waits for both to settle.
3. **Tracking** — gain is locked, the DC component is removed from both channels, and adaptive peak detection produces a rolling BPM estimate, an HRV (RMSSD) estimate, and a ratio-of-ratios SpO2 estimate.

This lets people with different skin, finger placement, and signal strengths use either sensor without reflashing. Open Arduino IDE's Serial Plotter at 115200 baud to view `Wave1`, `Wave2`, `Beat1`, `Beat2`, `BPM1`, `BPM2`, `HRV1`, `HRV2`, `SpO2_1`, `SpO2_2`, `State1`, and `State2`. State values are 0 = no contact, 1 = calibrating, and 2 = tracking.

SpO2 is a simplified, uncalibrated ratio-of-ratios estimate (Red vs. IR AC/DC) — a biofeedback cue for this installation, not a medically accurate reading.

### LED behaviour

The two LED strips run independently and reflect each participant's heartrate in real time.

| State | What the LEDs do |
|---|---|
| Idle / no reading | Slow twinkling ambient light |
| Heartbeat detected | Strip brightness pulses up on each beat |
| Both heartrates within 5–10 BPM | Strips begin to visually echo each other; software nudges them toward sync |
| Fully synced | Both strips pulse together at the shared rhythm |

### Motor behaviour

The vibration motor fires on each detected heartbeat with a linear ramp up (40 ms) and
ramp down (200 ms) — a felt pulse rather than an abrupt on/off click. Peak intensity is
currently a fixed constant (`MOTOR_PEAK_INTENSITY` in `theinterface.ino`); potentiometer
control is not wired in yet.

---

## Build order

Everything depends on a reliable BPM number. Do these in order:

1. **Validate clean BPM from raw sensor data**
   The main sketch now performs per-person automatic gain calibration followed by adaptive
   peak detection. Compare its BPM output against a manually counted pulse and tune only if
   needed; do not use it for medical measurement.

2. **Build motor modulation**
   Smooth onset, natural decay, tuned to feel like a heartbeat rather than a buzz.

3. **Build LED coherence logic**
   Each strip pulses on its participant's beat. When the two BPMs come within 5–10 of each
   other, the strips visually reflect closeness. The system gently nudges both toward a
   shared target BPM.

---

## Three LED modes (goal)

| Mode | Description |
|---|---|
| Fallback | LEDs pulse at an artificial "flow state" BPM. Both strips sync to this fixed rhythm even without live sensor data. |
| Default | Each strip reflects its participant's real heartrate. |
| Ideal | Each strip reflects its participant's heartrate, and the software gradually nudges both BPMs toward a shared flow state. |

---

## Open questions

- **LED strip type** — WS2812B assumed but not confirmed. Affects which library to use.

---

## Technical constraints

- Arduino Uno R4 Minima: one program, one loop. All logic (sensors, LEDs, motors) shares the same `loop()`.
- Both heart sensors have the same I2C address (`0x57`). They are isolated on separate software I2C buses (SoftWire library) to avoid conflicts.
- Hardware I2C (Wire library) caused intermittent bus lockups and was abandoned. SoftWire on bit-bang pins is the stable solution.
- DRV8833 motor drivers: IN2 is hardwired to GND on both boards; only IN1 (D6, D13) is driven from the Arduino.
- PCA9515A I2C repeaters are physically removed from the circuit, not just bypassed in code.

---

## Files

| File | What it is |
|---|---|
| `InterfaceSensor.h/.cpp` | Heart sensor library — done |
| `theinterface.ino` | Main sketch — two-person HR visualizer with runtime calibration and BPM tracking |
| `HardwareTest/HardwareTest.ino` | Test harness — flash this to verify each component works before writing production code |
| `CLAUDE.md` | Technical reference for AI-assisted development |
