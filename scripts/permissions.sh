#!/usr/bin/env bash
# install_device_rules.sh
#
# Installs / updates udev rules for:
#   • NXP “USBCAN II”     (USB‑CAN adapter)
#   • LP‑Research IMUs    (CP210x / ttyUSB*)
#   • u‑blox GNSS family  (CDC‑ACM / ttyACM*)
#
# It also ensures the invoking user is in the correct group and
# optionally builds in‑tree packages as that user.
#
# Usage:   sudo ./install_device_rules.sh
# Re‑run anytime; it is idempotent.

set -euo pipefail

# ───── helpers ────────────────────────────────────────────────────────
log()  { printf '\e[1;34m[i] %s\e[0m\n'  "$*"; }
warn() { printf '\e[1;33m[!] %s\e[0m\n' "$*"; }
die()  { printf '\e[1;31m[✗] %s\e[0m\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "Please run this script with sudo (or as root)."

# workspace root = directory that contains src/
WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. && pwd )"
cd "$WS_ROOT"

ensure_in_group() {
  local grp="$1"
  if ! id -nG "$SUDO_USER" | grep -qw "$grp"; then
    usermod -aG "$grp" "$SUDO_USER"
    warn "Added $SUDO_USER to $grp – log out / in (or run: newgrp $grp)."
  fi
}

# ───── SECTION 1 — USBCAN II adapter (VID 0471:1200) ────────────────
USB_RULE="/etc/udev/rules.d/99-usbcan.rules"
USB_VID="0471"  USB_PID="1200"  USB_GRP="plugdev"

log "--> Ensuring group '$USB_GRP' for USBCAN"
ensure_in_group "$USB_GRP"

log "--> Installing rule → $USB_RULE"
install -m 644 /dev/stdin "$USB_RULE" <<EOF
# NXP USBCAN II
SUBSYSTEM=="usb", ATTR{idVendor}=="$USB_VID", ATTR{idProduct}=="$USB_PID", MODE="0660", GROUP="$USB_GRP"
KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="$USB_VID", ATTRS{idProduct}=="$USB_PID", \
  SYMLINK+="usbcan%n", MODE="0660", GROUP="$USB_GRP"
EOF

# ───── SECTION 2 — LP‑Research IMU (VID 10c4:ea60) ───────────────────
IMU_RULE="/etc/udev/rules.d/99-lpms-imu.rules"
IMU_VID="10c4"  IMU_PID="ea60"  IMU_GRP="dialout"

log "--> Ensuring group '$IMU_GRP' for IMU"
ensure_in_group "$IMU_GRP"

log "--> Installing rule → $IMU_RULE"
install -m 644 /dev/stdin "$IMU_RULE" <<EOF
# LP‑Research IMU (CP210x)
SUBSYSTEM=="tty", ATTRS{idVendor}=="$IMU_VID", ATTRS{idProduct}=="$IMU_PID", \
  SYMLINK+="lpms_imu%n", MODE="0660", GROUP="$IMU_GRP"
EOF

# ───── SECTION 3 — u‑blox GNSS (vendor‑only match) ───────────────────
GPS_RULE="/etc/udev/rules.d/99-ublox-gps.rules"
GPS_VID="1546"  GPS_GRP="dialout"

log "--> Ensuring group '$GPS_GRP' for GNSS"
ensure_in_group "$GPS_GRP"

log "--> Installing rule → $GPS_RULE"
install -m 644 /dev/stdin "$GPS_RULE" <<EOF
# u‑blox GNSS receivers – match ANY PID for Vendor 1546
KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="$GPS_VID", \
  SYMLINK+="ublox_gps%n", MODE="0660", GROUP="$GPS_GRP"
SUBSYSTEM=="usb", ATTR{idVendor}=="$GPS_VID", MODE="0660", GROUP="$GPS_GRP"
EOF

# ───── SECTION 4 — Reload udev rules ────────────────────────────────
log "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

log "All done. Unplug and re‑plug your devices (or wait a few seconds)."
