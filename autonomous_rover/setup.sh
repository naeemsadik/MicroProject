#!/usr/bin/env bash
#
# setup.sh - one-time setup helper for the RPi4 warehouse rover.
#
# What it does:
#   1. Creates a Python virtual environment in .venv (so pip does not
#      fight the system "externally managed" environment on Debian /
#      Raspberry Pi OS).
#   2. Installs the project dependencies into the venv.
#   3. Tries to add the current user to the dialout group (needed for
#      /dev/ttyACM0 access).
#   4. Tries to fix the serial port permissions.
#   5. Regenerates the warehouse occupancy grid from the floorplan PNG.
#   6. Generates the QR code images for the configured slots.
#
# Re-run this script any time you change config files or the map image.
#
# After setup, always invoke the project via the venv, e.g.:
#     source .venv/bin/activate
#     python src/admin_panel.py
# Or directly without activating:
#     .venv/bin/python src/admin_panel.py
#
# The admin panel is the SINGLE entry point. It does everything:
# camera stream, manual driving, gripper, and auto-drive to a
# detected QR slot. There is no separate "main.py" anymore.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

VENV_DIR="$PROJECT_DIR/.venv"

echo "==> Creating Python virtual environment at $VENV_DIR"
if [ ! -d "$VENV_DIR" ]; then
    if ! command -v python3 >/dev/null 2>&1; then
        echo "    python3 not found; install it with: sudo apt install python3 python3-venv python3-pip"
        exit 1
    fi
    if ! python3 -m venv "$VENV_DIR" 2>/dev/null; then
        echo "    python3-venv is missing; install it with: sudo apt install python3-venv"
        exit 1
    fi
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

echo "==> Upgrading pip inside the venv"
python -m pip install --upgrade pip >/dev/null

echo "==> Installing Python dependencies from requirements.txt"
python -m pip install -r requirements.txt

echo "==> Adding user to the dialout group (serial port access)"
if getent group dialout >/dev/null; then
    if id -nG "$USER" 2>/dev/null | tr ' ' '\n' | grep -qx "dialout"; then
        echo "    user $USER is already in dialout"
    else
        if command -v sudo >/dev/null 2>&1; then
            sudo usermod -a -G dialout "$USER" || echo "    (could not add user to dialout, run manually)"
            echo "    NOTE: log out and back in (or run 'newgrp dialout') for it to take effect"
        else
            echo "    sudo not available; ask an admin to run: usermod -a -G dialout $USER"
        fi
    fi
fi

echo "==> Fixing serial port permissions"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
if [ -e "$SERIAL_PORT" ]; then
    if command -v sudo >/dev/null 2>&1; then
        sudo chmod a+rw "$SERIAL_PORT" 2>/dev/null || echo "    (could not chmod $SERIAL_PORT)"
    else
        echo "    $SERIAL_PORT exists; no sudo available to fix perms"
    fi
else
    echo "    no $SERIAL_PORT device found; plug in the ESP32 first"
fi

echo "==> Regenerating warehouse occupancy grid"
python -c "
from src.map_processor import MapProcessor
import os
mp = MapProcessor(resolution_cm_per_px=1.0, robot_radius_cm=4.0)
mp.generate_occupancy_grid('maps/floorplan.png', 'maps/occupancy_grid.npy')
print('    grid saved to maps/occupancy_grid.npy')
"

echo "==> Generating QR code images"
python generate_qr_codes.py --out qrcodes

cat <<EOF

==> Setup complete.

The admin panel is the single entry point. To start the robot:

    $VENV_DIR/bin/python src/admin_panel.py

Then open http://<RPi4-IP>:8080 from a browser on the same network.

From the admin panel you can:
  * watch the live USB camera feed
  * drive manually with the on-screen buttons
  * open / close the gripper
  * scan a QR and click "Drive to slot" for autonomous delivery

Run the unit tests (optional):
    $VENV_DIR/bin/python run_tests.py
EOF
