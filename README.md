# NERDSTAR eQ

NERDSTAR eQ is an ESP32-based **equatorial (RA/Dec) mount controller** for DIY telescope builds.

This revision rebuilds the motion model around **equatorial movement math**:
- **RA axis** is treated as continuous 0–360° (or 0–24h equivalent).
- **Dec axis** is no longer clamped by firmware soft limits.
- Tracking rates are interpreted as **RA/Dec degrees per second**.
- Legacy `AZ/ALT` command aliases remain available for compatibility, but map internally to `RA/DEC`.

---

## Features

- Dual-stepper control on ESP32 (RA + Dec)
- Manual motion and goto velocity channels
- Independent tracking-rate contribution (RA/Dec)
- Backlash compensation per axis with configurable take-up rate
- EEPROM-backed configuration (calibration, inversion, backlash)
- UART command protocol between controller roles
- LX200-compatible control over USB serial (Stellarmate/INDI compatible)
- Optional Stellarium LX200-compatible TCP bridge
- RTC support and OTA/Wi-Fi utility hooks

---

## Coordinate & Motion Model

### Axes

- `Axis::Ra` (legacy alias: `Axis::Az`)
- `Axis::Dec` (legacy alias: `Axis::Alt`)

### Conversion helpers

Primary API:
- `stepsToRaDegrees()` / `raDegreesToSteps()`
- `stepsToDecDegrees()` / `decDegreesToSteps()`

Compatibility wrappers:
- `stepsToAzDegrees()` / `azDegreesToSteps()`
- `stepsToAltDegrees()` / `altDegreesToSteps()`

### Limits

- RA wraps modulo 360°
- Declination is not soft-clamped by motion control

---

## Command Interface (main role)

Axis argument parser accepts both modern and legacy names:
- `RA`, `DEC`
- `AZ`, `ALT` (mapped to RA/Dec)

Motion conversion commands:
- `STEPS_TO_RA` (or `STEPS_TO_AZ`)
- `STEPS_TO_DEC` (or `STEPS_TO_ALT`)
- `RA_TO_STEPS` (or `AZ_TO_STEPS`)
- `DEC_TO_STEPS` (or `ALT_TO_STEPS`)

Tracking:
- `SET_TRACKING_RATES <ra_deg_per_sec> <dec_deg_per_sec>`

---

### LX200 control (Stellarmate via USB)

NERDSTAR now accepts LX200 commands over the USB serial connection (`Serial`) in addition to the TCP bridge.

Implemented LX200 command groups include:
- Position queries: `:GR#`, `:GD#`
- Target setup + GoTo: `:SrHH:MM:SS#`, `:Sd+DD*MM:SS#`, `:MS#`
- Manual slewing: `:Mn#`, `:Ms#`, `:Me#`, `:Mw#`, stop with `:Qn#`, `:Qs#`, `:Qe#`, `:Qw#`, `:Q#`
- Compatibility acknowledgements: `:SC...#`, `:SL...#`

This allows Stellarmate's LX200-compatible drivers to control Nerdstar directly over USB.

## Build

Use Arduino CLI (example for main controller role):

```bash
arduino-cli compile --build-property build.extra_flags=-DDEVICE_ROLE_MAIN
```

Then upload as usual for your board profile.

---

## Wiring

Default pin mapping is defined in `config.h`:
- RA stepper: `EN_RA`, `DIR_RA`, `STEP_RA`
- Dec stepper: `EN_DEC`, `DIR_DEC`, `STEP_DEC`
- Link UART: `COMM_TX_PIN`, `COMM_RX_PIN`
- RTC I2C: `RTC_SDA_PIN`, `RTC_SCL_PIN`

---

## Notes

- Existing configs that still reference old Az/Alt field names continue to work at firmware level.
- If you are migrating a physical mount from Alt/Az mechanics to equatorial mechanics, re-run calibration and verify motor inversion + backlash values.

---

## License

GPLv3 (see `LICENSE`).
