# SmartScope

A Raspberry Pi 4 telescope controller for the NERDSTAR eQ mount.  
Controls the mount via LX200 protocol over USB serial, captures frames with the Raspberry Pi HQ Camera, stacks them, plate-solves with ASTAP, and serves a mobile-optimised web UI over a WiFi hotspot.

---

## Hardware

| Component | Notes |
|---|---|
| Raspberry Pi 4 (any RAM) | Raspberry Pi OS Bookworm 64-bit |
| Raspberry Pi HQ Camera Module | Connected via CSI ribbon |
| NERDSTAR eQ mount | ESP32 connected via USB → `/dev/ttyUSB0` |
| USB stick | Mounted at `/mnt/storage` for FITS files |
| Power bank / 12 V supply | For field use |

The Pi is configured as a WiFi hotspot reachable at **192.168.4.1**.  
The web UI is served at **http://192.168.4.1:8000**.

---

## Software requirements

| Package | Source |
|---|---|
| Python 3.11+ | Included in RPi OS Bookworm |
| `python3-picamera2`, `libcamera-apps` | `apt` |
| `fastapi`, `uvicorn[standard]` | `pip` |
| `sse-starlette` | `pip` |
| `pyserial` | `pip` |
| `astropy` | `pip` |
| `Pillow` | `pip` |
| ASTAP | Manual – see below |

---

## Installation

### 1. Clone / copy files to the Pi

```bash
# From your development machine
scp -r smartscope/ pi@192.168.4.1:~/smartscope-src/

# Or directly on the Pi
git clone https://github.com/Philippxmeyer/NERDSTAR_eQ.git
cd NERDSTAR_eQ/smartscope
```

### 2. Run the installer

```bash
cd ~/NERDSTAR_eQ/smartscope   # or wherever you placed the files
sudo bash install.sh
```

The installer:
- Installs `python3-picamera2`, `libcamera-apps`, and other system packages via `apt`
- Installs Python packages (`fastapi`, `uvicorn`, `sse-starlette`, `pyserial`, `astropy`, `Pillow`) via `pip`
- Creates `/mnt/storage` (FITS storage) and `/home/pi/smartscope/` (application root)
- Copies application files to `/home/pi/smartscope/`
- Adds `pi` to the `dialout` group for USB serial access
- Writes a udev rule that creates `/dev/nerdstar` as a stable symlink for the ESP32
- Installs and enables the `smartscope.service` systemd unit

After installation, **log out and back in** (or reboot) so the `dialout` group membership takes effect.

### 3. Install ASTAP (plate solver)

ASTAP is not in the standard Raspberry Pi OS repositories and must be installed manually.

```bash
# Download the ARM64 .deb from https://www.hnsky.org/astap.htm
wget https://www.hnsky.org/astap_armv8.deb        # check the site for the current filename
sudo apt install ./astap_armv8.deb

# Download a star catalog – G18 works well for this field of view (~1.3°)
# Follow the link on https://www.hnsky.org/astap.htm#astap_g17
# Extract catalog files (*.1476 etc.) to:
sudo mkdir -p /usr/share/astap
sudo cp g18_* /usr/share/astap/
```

Plate solving is optional – all other features work without ASTAP.

### 4. Mount the USB storage stick

```bash
# Find the UUID of your USB stick
lsblk -o NAME,UUID,FSTYPE,MOUNTPOINT

# Add to /etc/fstab (replace UUID and fstype as needed)
echo 'UUID=xxxx-xxxx  /mnt/storage  vfat  defaults,nofail,uid=1000,gid=1000  0  2' | sudo tee -a /etc/fstab
sudo mount -a
```

If no USB stick is present, FITS files will not be written but all other features still work.

### 5. Set up the WiFi hotspot

Use NetworkManager (pre-installed on Bookworm) to create a persistent access point:

```bash
sudo nmcli con add \
  type wifi \
  ifname wlan0 \
  con-name "SmartScope-AP" \
  autoconnect yes \
  ssid "SmartScope" \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  ipv4.method shared \
  ipv4.addresses 192.168.4.1/24 \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "smartscope"

sudo nmcli con up "SmartScope-AP"
```

The Pi will now broadcast **SSID: SmartScope** and hand out DHCP leases in the `192.168.4.x` subnet.  
Connect your iPhone and open **http://192.168.4.1:8000**.

To disable the hotspot when connected to your home WiFi:
```bash
sudo nmcli con down "SmartScope-AP"
```

### 6. Start the service

```bash
sudo systemctl start smartscope
sudo systemctl status smartscope
```

The service is enabled at boot, so it starts automatically after the next reboot.

---

## Configuration

All hardware constants live in `config.py`:

```python
FOCAL_LENGTH_MM  = 326.0   # Telescope focal length
APERTURE_MM      = 50.0    # Aperture
PIXEL_SIZE_UM    = 2.0     # HQ Camera pixel pitch
SENSOR_WIDTH_PX  = 3840    # Capture width
SENSOR_HEIGHT_PX = 2160    # Capture height
SERIAL_PORT      = "/dev/ttyUSB0"   # or "/dev/nerdstar" (udev symlink)
SERIAL_BAUD      = 9600
STORAGE_PATH     = "/mnt/storage"
ASTAP_PATH       = "/usr/bin/astap"
CATALOG_PATH     = "/home/pi/smartscope/data/catalog.xml"
```

Edit `/home/pi/smartscope/config.py` and restart the service after any change:

```bash
sudo systemctl restart smartscope
```

---

## Object catalog

The bundled catalog lives at `smartscope/data/catalog.xml` in the repository and is copied to `/home/pi/smartscope/data/catalog.xml` by `install.sh` as part of the normal file rsync.  It contains 112 objects: 7 planets, the Moon, 49 bright stars, 34 clusters, 4 double stars, 16 galaxies, 7 nebulae, and 4 planetary nebulae.

You can replace it at any time by uploading a new file via the web UI (`/catalog/upload`) or by replacing `/home/pi/smartscope/data/catalog.xml` and restarting the service.

### XML format

```xml
<?xml version="1.0" encoding="UTF-8"?>
<catalog>
  <object name="Orion Nebula"   code="Messier 042" type="Nebula"
          ra_hours="5.5881" dec_degrees="-5.3911" magnitude="4.0"/>
  <object name="Andromeda Galaxy" code="Messier 031" type="Galaxy"
          ra_hours="0.7123" dec_degrees="41.2692" magnitude="3.4"/>
  <object name="Jupiter"        code="Jupiter"     type="Planet"
          ra_hours="0.0"    dec_degrees="0.0"      magnitude="-2.9"/>
  <object name="Earth Moon"     code="Earth Moon"  type="Moon"
          ra_hours="0.0"    dec_degrees="0.0"      magnitude="-12.7"/>
</catalog>
```

**Fields:**

| Attribute | Description |
|---|---|
| `name` | Display name (searched) |
| `code` | Catalog code used by `/goto/object` (searched) |
| `type` | `Planet` or `Moon` → live position via astropy; anything else → fixed coords |
| `ra_hours` | Right ascension in decimal hours (ignored for Planet/Moon) |
| `dec_degrees` | Declination in decimal degrees (ignored for Planet/Moon) |
| `magnitude` | Apparent magnitude (display only) |

**Supported types in the bundled catalog:** `Planet`, `Moon`, `Star`, `Cluster`, `Double Star`, `Galaxy`, `Nebula`, `Planetary Nebula`.

**Supported planet/moon codes** (case-insensitive): `mercury`, `venus`, `mars`, `jupiter`, `saturn`, `uranus`, `neptune`, `earth moon`, `sun`.

### Search shortcuts

The search field recognises observer shorthand:

| You type | Matches |
|---|---|
| `M42` | `Messier 042` |
| `m 16` | `Messier 016` |
| `NGC253` | `NGC 0253` |
| `orion` | Any object with "orion" in name or code |

---

## Using the web UI

Open **http://192.168.4.1:8000** on your iPhone after connecting to the `SmartScope` hotspot.

### First launch – Init dialog

On first load, a dialog asks for UTC time and your location.  
- Time is pre-filled from your iPhone's clock.  
- Location is auto-filled if you allow browser location access.  
- Tap **Initialise** to push time and location to the ESP32 RTC via LX200.

The warning banner at the top disappears once the mount is initialised.

### Search & GoTo

Type an object name or Messier code in the search field.  Planets show their current calculated position.  Tap a result to slew the mount to that object.

### Manual slew (D-pad)

Hold a direction button to slew.  Release to stop.  The selected slew speed (Guide / Center / Find / Max) applies.

### Capture & Stack

Set exposure time, analogue gain, and frame count, then tap **▶ Capture**.  
Frames are written as FITS files to `/mnt/storage` and added to the running stack.  
The live preview updates every 2 seconds with a percentile-stretched JPEG of the stack.

### Plate solve

Tap **⊕ Solve** to capture a fresh frame and run ASTAP.  On success:
- The pointing offset (`state.ra_offset`, `state.dec_offset`) is updated.
- All subsequent GoTo commands apply this correction automatically.
- The ESP32 RTC is refreshed via `sync_time_location`.

### Tracking toggle

Enables/disables the running capture loop.  The mount's internal sidereal tracking is always active; this toggle only controls whether the Pi captures and stacks frames.

---

## REST API

All endpoints are at `http://192.168.4.1:8000`.  Request/response bodies are JSON unless noted.

### Status & preview

| Method | Path | Description |
|---|---|---|
| `GET` | `/` | Serve web UI |
| `GET` | `/status` | JSON status snapshot |
| `GET` | `/preview` | SSE stream – base64 JPEG every 2 s |

#### `GET /status` response

```json
{
  "ra": 5.588,
  "dec": -5.391,
  "tracking": false,
  "slewing": false,
  "stack_count": 12,
  "utc": "2025-04-23T21:34:00+00:00",
  "location_set": true,
  "esp32_connected": true,
  "catalog_loaded": true,
  "capture_active": true,
  "frames_captured": 12
}
```

### Initialisation

| Method | Path | Body |
|---|---|---|
| `POST` | `/init` | `{"utc": "2025-04-23T21:34:00Z", "lat": 48.1, "lon": 11.6}` |

Sets the time reference and observer location; forwards them to the ESP32 via LX200 `:SC#` / `:SL#` / `:Sts#` / `:Sg#` / `:SG#`.

### GoTo

| Method | Path | Body |
|---|---|---|
| `POST` | `/goto/radec` | `{"ra_hours": 5.588, "dec_degrees": -5.391}` |
| `POST` | `/goto/object` | `{"code": "M42"}` |

Pointing offsets from the last plate solve are applied before the mount command is issued.

### Search

| Method | Path | Query |
|---|---|---|
| `GET` | `/search` | `?q=orion` |

Returns a JSON array of matching catalog objects.  Planet positions are calculated for the current UTC.

### Capture & solve

| Method | Path | Body |
|---|---|---|
| `POST` | `/capture` | `{"exposure": 2.0, "gain": 1.0, "frames": 100}` |
| `POST` | `/solve` | _(no body)_ – captures a frame and runs ASTAP |

### Mount control

| Method | Path | Body |
|---|---|---|
| `POST` | `/track` | `{"enabled": true}` |
| `POST` | `/slew` | `{"direction": "n"\|"s"\|"e"\|"w", "active": true}` |
| `POST` | `/slew/rate` | `{"rate": "guide"\|"center"\|"find"\|"max"}` |
| `POST` | `/stop` | _(emergency stop – no body)_ |
| `POST` | `/park` | _(no body)_ |
| `POST` | `/shutdown` | _(queues park, then host shutdown)_ |

### Stack & catalog

| Method | Path | Description |
|---|---|---|
| `POST` | `/stack/reset` | Clear the image stack |
| `POST` | `/catalog/upload` | Multipart file upload of XML catalog |

---

## Service management

```bash
# Start / stop / restart
sudo systemctl start   smartscope
sudo systemctl stop    smartscope
sudo systemctl restart smartscope

# Enable / disable autostart
sudo systemctl enable  smartscope
sudo systemctl disable smartscope

# Live logs
sudo journalctl -u smartscope -f

# Last 100 lines
sudo journalctl -u smartscope -n 100
```

---

## Architecture notes

### LX200 serial driver (`lx200.py`)

All serial I/O flows through a single `asyncio.Queue`.  A worker coroutine dequeues one `(command, future)` pair at a time and runs the blocking `serial.write` / `serial.read_until('#')` in a `ThreadPoolExecutor` thread.  This guarantees that the NERDSTAR eQ's request–response protocol is never interleaved by concurrent callers and that the asyncio event loop is never blocked.

Commands with no reply (`:RG#`, `:RC#`, `:RM#`, `:RS#`, `:T*#`, `:U#`) pass `future=None`; the worker sends them and moves on immediately.

### Shutdown flow

`POST /shutdown` and the optional Pimoroni `onoffshim` button both use the same flow: queue LX200 park (`:hP#`) first, then request Linux shutdown (`sudo shutdown -h now`). Park is intentionally non-blocking so shutdown is not delayed indefinitely by missing mount feedback.

### Camera (`camera.py`)

`Picamera2` is initialised once and kept running.  All camera calls go through a `ThreadPoolExecutor(max_workers=1)`, so the event loop is never blocked and concurrent captures are impossible.  Frames are returned as `HxWx3 uint8` numpy arrays and converted to grayscale `float32` for stacking and FITS output.

### Time reference (`state.py`)

`ScopeState.utc_time_ref` stores the UTC time received from the iPhone browser via `POST /init`.  `ScopeState.monotonic_ref` stores the corresponding `time.monotonic()` value.  `get_current_utc()` advances the stored UTC by the elapsed monotonic time, giving a stable UTC clock that survives network gaps without requiring NTP.

### Pointing model (`pointing.py`)

Phase 1 is a simple additive offset:

```
ra_corrected  = ra_hours  + ra_offset_arcsec  / (15 × 3600)
dec_corrected = dec_deg   + dec_offset_arcsec / 3600
```

The offsets are populated by `POST /solve`.  The module is structured so that a Taki / Konradi or Geostar multi-star model can replace `apply_offsets()` in a future phase without changing any callers.

### SSE preview

`GET /preview` is a Server-Sent Events stream.  The browser's `EventSource` API reconnects automatically after network interruptions (e.g. iPhone screen lock), making it more resilient than WebSocket for this use case.

---

## Troubleshooting

### Mount not connecting (`esp32_connected: false`)

```bash
# Verify the device appears
ls -la /dev/ttyUSB* /dev/nerdstar 2>/dev/null

# Check kernel sees the CP210x chip
dmesg | grep -i "cp210x\|ttyUSB"

# Verify group membership (must log out/in after install)
groups pi   # should include "dialout"

# Test raw serial manually
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)
s.write(b':GVP#')
time.sleep(0.5)
print(repr(s.read_all()))
"
```

Expected reply: `NERDSTAR-eQ#`

### Camera not working

```bash
# Check libcamera sees the sensor
libcamera-still --list-cameras

# Quick test capture
libcamera-still -o /tmp/test.jpg --width 640 --height 480
```

### ASTAP plate solve always fails

1. Verify ASTAP is installed: `astap -h`
2. Verify the star catalog exists: `ls /usr/share/astap/*.1476` (or similar)
3. Check the solve log: `sudo journalctl -u smartscope -n 50 | grep -i "astap\|solve"`
4. Try a manual solve:
   ```bash
   astap -f /mnt/storage/latest.fits -ra 5.588 -spd 84.609 -r 10 -fov 1.35 -update
   ```

### Service fails to start

```bash
sudo systemctl status smartscope
sudo journalctl -u smartscope -n 50
```

Common causes:
- Python package missing → re-run `sudo bash install.sh`
- `/home/pi/smartscope/` not found → check that install copied files correctly
- Port 8000 already in use → `sudo lsof -i :8000`

### Web UI shows warning banner after init

The banner disappears only after a successful `POST /init` response.  Open the browser console (Safari DevTools) and check for network errors on the `/init` request.

---

## License

GPLv3 – see `LICENSE` in the repository root.
