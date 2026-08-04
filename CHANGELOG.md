# Changelog

All notable changes to The Interface are documented here.

## Unreleased

### Added

- Two-participant serial heart-rate visualizer in `theinterface.ino`.
- Per-participant runtime states: no contact, calibration, and tracking.
- Per-participant automatic IR LED-current calibration with bounded gain and settling time.
- DC-removed waveforms, adaptive beat thresholds, beat markers, and rolling BPM estimates.
- Clean Arduino Serial Plotter fields: `Wave1`, `Wave2`, `Beat1`, `Beat2`, `BPM1`, `BPM2`, `State1`, and `State2`.
- `InterfaceSensor::setIRLedAmplitude()` for runtime IR LED-current control.

### Changed

- Replaced the original single-sensor IR print loop with a two-person visualizer.
- Replaced continuous gain changes during BPM detection with a calibration phase that locks gain before tracking.
- Updated the design and development documentation to describe the current sensor workflow.

### Fixed

- Switched MAX30102 reads to SoftWire's buffered transaction API, fixing register and FIFO reads on the Arduino UNO R4 Minima.
