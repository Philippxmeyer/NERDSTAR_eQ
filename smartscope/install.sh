#!/usr/bin/env bash
# SmartScope installer
# Run as root from inside the smartscope/ directory:  sudo bash install.sh
#
# What this script does:
#   1. Installs system packages (picamera2, libcamera, udev, …)
#   2. Installs Python packages via pip
#   3. Creates /mnt/storage and the application directory
#   4. Copies application files to /home/pi/smartscope/
#   5. Writes a udev rule for the NERDSTAR eQ USB serial adapter
#   6. Adds the pi user to the dialout group
#   7. Installs and enables the smartscope systemd service

set -euo pipefail

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERR]${NC}   $*" >&2; }
section() { echo -e "\n${BLUE}▶ $*${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="/home/pi/smartscope"
SERVICE_NAME="smartscope"
PI_USER="pi"

# ---------------------------------------------------------------------------
# 0. Pre-flight checks
# ---------------------------------------------------------------------------
section "Pre-flight checks"

if [[ $EUID -ne 0 ]]; then
    error "This script must be run as root.  Try:  sudo bash install.sh"
    exit 1
fi

if ! id "$PI_USER" &>/dev/null; then
    error "User '$PI_USER' not found. Adjust PI_USER at the top of this script."
    exit 1
fi

if command -v python3 &>/dev/null; then
    PY_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
    info "Python ${PY_VER} found"
    if python3 -c "import sys; sys.exit(0 if sys.version_info >= (3,11) else 1)"; then
        info "Python version OK (≥ 3.11)"
    else
        warn "Python ${PY_VER} detected – 3.11+ recommended. Continuing anyway."
    fi
else
    error "python3 not found. Install Raspberry Pi OS Bookworm 64-bit."
    exit 1
fi

if grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    MODEL=$(tr -d '\0' < /proc/device-tree/model)
    info "Running on: ${MODEL}"
else
    warn "Not a Raspberry Pi (or /proc/device-tree not readable) – continuing."
fi

# ---------------------------------------------------------------------------
# 1. System packages
# ---------------------------------------------------------------------------
section "Installing system packages"

apt-get update -qq

PKGS=(
    python3-pip
    python3-picamera2
    python3-numpy
    libcamera-apps
    v4l-utils
    rsync
    git
)

apt-get install -y "${PKGS[@]}"
info "System packages installed."

# ---------------------------------------------------------------------------
# 2. Python packages
# ---------------------------------------------------------------------------
section "Installing Python packages"

# On Raspberry Pi OS Bookworm, pip installs require --break-system-packages
# because the OS uses PEP 668 externally-managed environments.
PIP_FLAGS="--break-system-packages --quiet"

pip3 install $PIP_FLAGS \
    "fastapi>=0.110.0" \
    "uvicorn[standard]>=0.29.0" \
    "sse-starlette>=2.0.0" \
    "pyserial>=3.5" \
    "astropy>=6.0" \
    "Pillow>=10.0"

info "Python packages installed."

# ---------------------------------------------------------------------------
# 3. ASTAP (plate solver)
# ---------------------------------------------------------------------------
section "Checking ASTAP"

if command -v astap &>/dev/null; then
    info "ASTAP already installed at: $(command -v astap)"
else
    warn "ASTAP not found.  Plate solving will be unavailable until it is installed."
    warn ""
    warn "  Download the ARM64 .deb from https://www.hnsky.org/astap.htm"
    warn "  Then install with:"
    warn "    sudo apt install ./astap_armv8.deb"
    warn ""
    warn "  Also download a star catalog (G18 recommended for this FOV):"
    warn "    https://www.hnsky.org/astap.htm#astap_g17"
    warn "  Extract catalog files to: /usr/share/astap/"
fi

# ---------------------------------------------------------------------------
# 4. Directory structure
# ---------------------------------------------------------------------------
section "Creating directories"

install -d -m 755 -o "$PI_USER" -g "$PI_USER" "$APP_DIR"
install -d -m 755 -o "$PI_USER" -g "$PI_USER" "$APP_DIR/static"
install -d -m 755 -o "$PI_USER" -g "$PI_USER" "$APP_DIR/systemd"
info "Application directory: $APP_DIR"

install -d -m 777 /mnt/storage
info "Storage mount point: /mnt/storage"

if ! grep -q '/mnt/storage' /etc/fstab 2>/dev/null; then
    warn "/mnt/storage is not in /etc/fstab."
    warn "Add a line like (replace UUID and fstype as needed):"
    warn "  UUID=xxxx-xxxx  /mnt/storage  vfat  defaults,nofail,uid=1000,gid=1000  0  2"
    warn "Then run:  sudo mount -a"
fi

# ---------------------------------------------------------------------------
# 5. Copy application files
# ---------------------------------------------------------------------------
section "Copying application files to $APP_DIR"

rsync -a --exclude='.git' --exclude='__pycache__' \
    "$SCRIPT_DIR/" "$APP_DIR/"
chown -R "$PI_USER:$PI_USER" "$APP_DIR"
info "Files copied."

# ---------------------------------------------------------------------------
# 6. USB serial permissions
# ---------------------------------------------------------------------------
section "Configuring serial port access"

# Add pi to dialout group so it can access /dev/ttyUSB0 without sudo
if id -nG "$PI_USER" | grep -qw dialout; then
    info "User '$PI_USER' is already in the dialout group."
else
    usermod -aG dialout "$PI_USER"
    info "Added '$PI_USER' to the dialout group (re-login required for this to take effect)."
fi

# udev symlink for NERDSTAR eQ (Silicon Labs CP210x USB-UART bridge)
UDEV_RULE=/etc/udev/rules.d/99-nerdstar-eq.rules
cat > "$UDEV_RULE" <<'EOF'
# NERDSTAR eQ – Silicon Labs CP210x USB-UART (ESP32 DevKit)
# Creates /dev/nerdstar as a stable symlink alongside /dev/ttyUSB0
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="nerdstar", MODE="0660", GROUP="dialout", \
  TAG+="systemd", ENV{SYSTEMD_ALIAS}="/dev/nerdstar"
EOF
udevadm control --reload-rules
info "udev rule written to $UDEV_RULE"
info "Stable device symlink: /dev/nerdstar → /dev/ttyUSBx"

# ---------------------------------------------------------------------------
# 7. systemd service
# ---------------------------------------------------------------------------
section "Installing systemd service"

SYSTEMD_SRC="$SCRIPT_DIR/systemd/${SERVICE_NAME}.service"
SYSTEMD_DEST="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ ! -f "$SYSTEMD_SRC" ]]; then
    error "Service file not found: $SYSTEMD_SRC"
    exit 1
fi

cp "$SYSTEMD_SRC" "$SYSTEMD_DEST"
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"
info "Service installed and enabled: $SYSTEMD_DEST"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo
echo -e "${GREEN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║      SmartScope installation complete!           ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════╝${NC}"
echo
info "Start the service now:   sudo systemctl start ${SERVICE_NAME}"
info "View live logs:          sudo journalctl -u ${SERVICE_NAME} -f"
info "Web UI (hotspot):        http://192.168.4.1:8000"
info "Web UI (local):          http://localhost:8000"
echo
if ! command -v astap &>/dev/null; then
    warn "Remember to install ASTAP before using the plate-solve feature."
fi
if ! id -nG "$PI_USER" | grep -qw dialout; then
    warn "Log out and back in (or reboot) for the dialout group change to take effect."
fi
