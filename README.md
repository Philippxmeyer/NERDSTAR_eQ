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
- Software clock seeded by the LX200 host (no battery-backed RTC required)

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

### LX200 control (Stellarmate / INDI via USB)

NERDSTAR exposes an LX200-compatible command channel on the USB serial port
(`Serial`). The port runs at **9600 baud** so it matches the default speed of
the Meade-compatible INDI drivers shipped with Stellarmate.

Implemented LX200 command groups:
- Position queries: `:GR#`, `:GD#`
- Target setup + GoTo: `:SrHH:MM:SS#`, `:Sd+DD*MM:SS#`, `:MS#`
- Manual slewing: `:Mn#`, `:Ms#`, `:Me#`, `:Mw#`; stop via `:Qn#`, `:Qs#`, `:Qe#`, `:Qw#`, `:Q#`
- Product / firmware identification: `:GVP#`, `:GVN#`, `:GVF#`, `:GVD#`, `:GVT#`
- Observer site (persisted in EEPROM):
  - `:StsDD*MM[:SS]#` - set latitude (north-positive, stored in `SiteLocation.latitudeDeg`)
  - `:SgDDD*MM[:SS]#` - set longitude (Meade's west-positive form is converted to ISO east-positive on the fly)
  - `:SGsHH.H#` / `:SGsHH:MM#` - UTC offset
  - `:Gt#` / `:Gg#` - read back stored latitude / longitude
- Host-supplied date / time seed the firmware's software clock (the mount
  has no hardware RTC; the host is expected to push a fresh sync every
  ~30 minutes to keep drift small):
  - `:SCMM/DD/YY#` (or `:SCMM/DD/YYYY#`) buffers the date
  - `:SLHH:MM:SS#` completes the pair; the firmware combines both with the
    stored UTC offset and calls `time_utils::setUtcEpoch(...)`
  - `:GC#` / `:GL#` / `:GG#` read the current local date / local time / UTC
    offset back from the software clock
  - Custom: `:GTS#` returns the seconds elapsed since the last `:SC`/`:SL`
    sync so the host can decide when a re-sync is due
- Slew-rate / tracking-rate / precision toggles silently accepted: `:RG#`, `:RC#`, `:RM#`, `:RS#`, `:TQ#`, `:TS#`, `:TL#`, `:T+#`, `:T-#`, `:U#`
- Distance bars: `:D#` (empty when idle, `|#` while slewing)
- No LX200 homing/park-to-endstop commands are implemented (`:hP#`, `:hC#`, `:hF#` return `0`).

Declination replies are clamped to `[-90°, +90°]` so INDI never receives
out-of-range coordinates from an uncalibrated axis.

USB serial is reserved exclusively for LX200 traffic - the firmware does not
print any debug or boot banners on `Serial`.

## Build

Use Arduino CLI:

```bash
arduino-cli compile
```

Then upload as usual for your board profile.

---

## Wiring

Default pin mapping is defined in `config.h`:
- RA stepper: `EN_RA`, `DIR_RA`, `STEP_RA`
- Dec stepper: `EN_DEC`, `DIR_DEC`, `STEP_DEC`
- Link UART: `COMM_TX_PIN`, `COMM_RX_PIN`

---

## Notes

- Existing configs that still reference old Az/Alt field names continue to work at firmware level.
- If you are migrating a physical mount from Alt/Az mechanics to equatorial mechanics, re-run calibration and verify motor inversion + backlash values.

---

## License

GPLv3 (see `LICENSE`).
