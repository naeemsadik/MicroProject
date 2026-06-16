# Warehouse Rover - Microproject

End-to-end automated warehouse robot. The Raspberry Pi 4 is the mission
brain (camera, QR scanning, path planning, decision making). The ESP32
is the low-level controller (motors, gripper, ultrasonic, MPU6050).

```
RPi4 USB  ---->  ESP32 USB
RPi4 USB  ---->  USB camera
ESP32     ---->  L298N, servos, HC-SR04, MPU6050
```

## Repository layout

```
.
+-- autonomous_rover/                  RPi4 software package
|   +-- config/
|   |   +-- settings.yaml              Serial, camera, map, navigation params
|   |   +-- warehouse_slots.yaml       Slot ID -> (x, y) lookup
|   +-- maps/
|   |   +-- floorplan.png              Mock warehouse top view
|   |   +-- occupancy_grid.npy         Generated occupancy grid (0=free, 1=obs)
|   +-- src/
|   |   +-- admin_panel.py             Web admin panel (HTTP server)
|   |   +-- comms.py                   ESP32 serial link (V/G/T protocol)
|   |   +-- map_processor.py           Map image -> occupancy grid
|   |   +-- mission_controller.py      QR -> path -> drive loop
|   |   +-- navigator.py               Waypoint steering controller
|   |   +-- odometry.py                Differential-drive pose estimator
|   |   +-- planner.py                 A* + line-of-sight path pruning
|   |   +-- qr_scanner.py              OpenCV QR detection wrapper
|   |   +-- warehouse_slots.py         Slot config loader + normaliser
|   |   +-- localization.py            Optional ArUco helper
|   |   +-- virtual_hardware.py        Optional SIL twin
|   +-- esp32/
|   |   +-- rover_firmware/
|   |       +-- rover_firmware.ino     Upload this to the ESP32
|   +-- generate_qr_codes.py           Build printable QR labels
|   +-- run_tests.py                   Unit tests
|   +-- setup.sh                       One-time setup helper
+-- WarehouseRobot.ino                 Legacy standalone demo (WiFi panel)
+-- PinDiagram.md                      Wiring reference
+-- changedPIn.md                      Pin change log
+-- uppdatedPinDiagram.md              Latest pin diagram
+-- RPi4_ESP32_Positioning_And_Connection_Plan.md
+-- totalplan.md                       High-level architecture plan
+-- Plan.md                            Standalone demo plan
+-- RPi_Admin_Panel.md                 Admin panel plan
```

## Quick start (RPi4)

```bash
# 1. One-time setup
cd autonomous_rover
bash setup.sh

# 2. Run the admin panel
python3 src/admin_panel.py
# Then open http://<RPi4-IP>:8080 from a phone or laptop

# 3. Run a single mission (dry run first)
python3 src/main.py --qr R1C1 --dry-run
python3 src/main.py --qr R1C1
```

## Quick start (ESP32)

1. Open `autonomous_rover/esp32/rover_firmware/rover_firmware.ino` in
   the Arduino IDE.
2. Make sure the ESP32 board support and `ESP32Servo` library are
   installed.
3. Select the right port and click **Upload**.
4. Open the Arduino Serial Monitor at 115200 baud. You should see
   `<READY>` within a couple of seconds.
5. From a serial console (or via the RPi4 admin panel), send
   `<PING>` and look for `<PONG>` to confirm the link.

## Serial protocol

```
RPi4 -> ESP32  <V,left,right>      signed motor PWM (-255..255)
             <G,OPEN>              open gripper
             <G,CLOSE>             close gripper
             <PING>                handshake
             <RESET_TICKS>         zero encoders + yaw

ESP32 -> RPi4  <T,distance_cm,left_ticks,right_ticks,yaw_deg>
             <PONG>                reply to <PING>
             <READY>               sent once at boot
```

## Configuration

`config/settings.yaml` keys:

* `serial.port` / `serial.baudrate` - ESP32 serial port (default
  `/dev/ttyACM0` because the ESP32 enumerates as a CDC ACM device on
  the RPi4).
* `camera.index` / `camera.width` / `camera.height` - USB camera
  parameters.
* `map.image` / `map.grid` - warehouse map image and the generated
  occupancy grid.
* `map.robot_radius_cm` - safety inflation radius around obstacles.
* `navigation.home` - start position in map pixels.
* `navigation.obstacle_stop_cm` / `obstacle_resume_cm` - ultrasonic
  safety thresholds.
* `navigation.max_drive_speed` / `max_turn_speed` - PWM clip for
  autonomous driving.
* `odometry.enabled` - leave **false** until wheel encoders are wired
  and being read. With no encoders the mission controller falls back to
  dead reckoning using timed motion.
* `dead_reckoning.forward_speed_cm_per_s` /
  `dead_reckoning.turn_speed_deg_per_s` - calibration for dead
  reckoning.
* `admin.host` / `admin.port` - web panel bind address.
* `admin.manual_speed` / `nudge_duration_s` - manual drive tuning.

`config/warehouse_slots.yaml` keys:

```yaml
home: [20, 20]
slots:
  R1C1:
    drop: [190, 50]      # final drop point
    approach: [180, 50]  # optional pre-drop point
  ...
```

QR codes should encode a slot ID like `R1C1`, `R02C03` (the
normalisation accepts leading zeros and ignores case / dashes).

## Tests

```bash
cd autonomous_rover
python3 run_tests.py
```

The suite covers the warehouse slot loader, A* planner, navigator,
odometry, comms, and an end-to-end dry-run mission.
