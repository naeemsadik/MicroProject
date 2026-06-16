# Quick Start Guide

## One-time setup (RPi4)

```bash
cd ~/MicroProject/autonomous_rover
bash setup.sh
```

The `setup.sh` script will:

1. Install all Python dependencies.
2. Try to add your user to the `dialout` group (needed for ESP32 USB).
3. Try to fix permissions on `/dev/ttyACM0`.
4. Regenerate the warehouse occupancy grid.
5. Generate printable QR codes for each slot.

If step 2 needs sudo and asks for a password, enter it. Then
**log out and back in** so the group change takes effect.

## ESP32 firmware (upload once)

1. Open `autonomous_rover/esp32/rover_firmware/rover_firmware.ino`
   in the Arduino IDE.
2. Make sure the ESP32 board support and `ESP32Servo` library are
   installed.
3. Select the right port and click **Upload**.
4. Open the Serial Monitor at 115200 baud. You should see `<READY>`.

## Run the admin panel

```bash
cd ~/MicroProject/autonomous_rover
python3 src/admin_panel.py
```

Open the URL it prints (default `http://0.0.0.0:8080`) from a phone or
laptop on the same network.

What you will see:

* **Live RPi Camera** - MJPEG stream of the USB camera with QR
  detection overlay.
* **Warehouse Map + Route** - the occupancy grid with the planned
  route to the currently detected QR code's slot.
* **Status cards** - ESP32 connection, camera status, last QR,
  ultrasonic distance, MPU6050 yaw, uptime.
* **Manual Controls** - forward / backward / left / right / stop,
  open/close gripper.
* **Destination** - JSON view of the slot lookup and planned route.
* **Known Slots** - list of configured slot IDs.
* **Logs** - the last 120 admin messages.

## Run a single mission

```bash
# Dry run (plan only, no driving)
python3 src/main.py --qr R1C1 --dry-run

# Real run with a fixed QR
python3 src/main.py --qr R1C1

# Real run that scans the QR from the USB camera
python3 src/main.py
```

## Generate QR codes

```bash
python3 generate_qr_codes.py
# Output -> qrcodes/R1C1.png, R1C2.png, ...
```

Print the PNGs, glue them to your packages, and the RPi4 USB camera
will pick them up automatically.

## Run tests

```bash
python3 run_tests.py
```

## Troubleshooting

* **"Permission denied: /dev/ttyACM0"** - your user is not in the
  `dialout` group. Re-run `bash setup.sh` and log out / back in.
* **"Could not open camera index 0"** - the USB camera is on a
  different index. Try `cv2.VideoCapture(1)` or check `ls /dev/video*`.
* **"No path from start to target"** - the destination coordinate
  is inside an obstacle. Update the `approach` / `drop` in
  `config/warehouse_slots.yaml`.
* **ESP32 not responding to `<PING>`** - re-upload the firmware and
  check that the right COM port is selected. The port name in
  `config/settings.yaml` must match.
