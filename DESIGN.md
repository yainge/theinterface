# The Interface — Design Document

A two-participant biofeedback installation. Each person wears a heart rate sensor.
The system reads both heartrates and uses light and vibration to create a shared sensory
experience, nudging both participants toward a shared "flow state" rhythm.

---

## Hardware overview

One Arduino Uno runs everything. Each participant has:

- **Heart sensor** (MAX30102) — reads pulse via infrared light through skin
- **Vibration motor** (DRV8833 driver) — rumble pack, tuned to feel like a heartbeat
- **LED strip** — 44 lights that respond to that participant's heartrate

Shared inputs:
- **Button** (A0) — cycles through LED patterns
- **Potentiometer** (A2) — tunes motor intensity

---

## What the system does

### LED behaviour

The two LED strips run independently and reflect each participant's heartrate in real time.

| State | What the LEDs do |
|---|---|
| Idle / no reading | Slow twinkling ambient light |
| Heartbeat detected | Strip brightness pulses up on each beat |
| Both heartrates within 5–10 BPM | Strips begin to visually echo each other; software nudges them toward sync |
| Fully synced | Both strips pulse together at the shared rhythm |

### Motor behaviour

The vibration motor fires on each detected heartbeat with a smooth onset — not a click,
but a felt pulse. Intensity is tunable via the potentiometer.

---

## Build order

Everything depends on a reliable BPM number. Do these in order:

1. **Get clean BPM from raw sensor data**
   The MAX30102 outputs raw 18-bit infrared samples at 100 Hz. A peak-detection algorithm
   finds the peaks and computes inter-beat intervals → BPM. There is an existing example
   program that does this translation; use it as the starting point.

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
- **Motor wiring** — DRV8833 needs two control pins per channel (IN1 + IN2). Only one Arduino pin is mapped per motor. Need to confirm whether IN2 is hardwired to GND or uses a second pin.
- **PCA9515A I2C repeaters** — physically in circuit or removed? Currently bypassed in code.

---

## Technical constraints

- Arduino Uno: one program, one loop. All logic (sensors, LEDs, motors) shares the same `loop()`.
- Both heart sensors have the same I2C address (`0x57`). They are isolated on separate software I2C buses (SoftWire library) to avoid conflicts.
- Hardware I2C (Wire library) caused intermittent bus lockups and was abandoned. SoftWire on bit-bang pins is the stable solution.

---

## Files

| File | What it is |
|---|---|
| `InterfaceSensor.h/.cpp` | Heart sensor library — done |
| `theinterface.ino` | Main sketch — currently a single-sensor IR loop; needs full implementation |
| `HardwareTest/HardwareTest.ino` | Test harness — flash this to verify each component works before writing production code |
| `CLAUDE.md` | Technical reference for AI-assisted development |
