# RPi4 Admin Panel

The RPi4 admin panel is a local web dashboard for the warehouse robot.

Run it on the Raspberry Pi:

```bash
cd autonomous_rover
pip install -r requirements.txt
python src/admin_panel.py
```

Then open this from a phone/laptop on the same network:

```text
http://RASPBERRY_PI_IP:8080
```

## What It Shows

- Live RPi USB camera feed.
- QR code currently detected by the camera.
- Destination slot lookup from `config/warehouse_slots.yaml`.
- Planned route preview for the detected QR destination.
- ESP32 connection status.
- Ultrasonic distance telemetry.
- MPU6050 yaw telemetry.
- Recent admin logs.
- Known warehouse slot IDs.

## Manual Controls

The panel includes basic manual controls:

- forward nudge,
- backward nudge,
- left nudge,
- right nudge,
- stop,
- gripper open,
- gripper close.

The nudge duration and speed are configured in `config/settings.yaml`:

```yaml
admin:
  manual_speed: 120
  nudge_duration_s: 0.35
```

## ESP32 Connection

Connect the ESP32 to the RPi4 using USB:

```text
RPi4 USB port -> ESP32 USB cable
```

The RPi4 sends serial commands:

```text
<V,left,right>
<G,OPEN>
<G,CLOSE>
```

The ESP32 sends telemetry:

```text
<T,distance_cm,left_ticks,right_ticks,yaw_deg>
```

Because the current robot has no encoders, left and right ticks stay at `0`.

## Important Note

The manual controls require the ESP32 firmware that understands serial commands. The current `WarehouseRobot.ino` is an ESP32-only timed demo with its own WiFi page. For RPi-controlled driving, upload:

```text
autonomous_rover/esp32/rover_firmware/rover_firmware.ino
```

The camera feed and QR detection work from the RPi4 even if the ESP32 is not connected.
