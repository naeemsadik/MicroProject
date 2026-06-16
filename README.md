# Warehouse Rover — Full Project Explanation

> **Single command to run everything:** `cd autonomous_rover && .venv/bin/python src/admin_panel.py`
>
> Then open `http://<RPi4-IP>:8080` from a browser. The admin panel
> is the **only** thing you need to run. From the page you can
> watch the camera, drive manually, open/close the gripper, and
> start an autonomous delivery to any slot.

This document is a complete, end-to-end explanation of the
warehouse rover: hardware, wiring, firmware, every Python module,
every config file, and the algorithms behind the autonomy.

---

## 1. Project Goal

A small **autonomous warehouse robot** that:

1. Sits at a "home" position on a known floorplan.
2. Reads a **QR code** stuck on a package (camera input).
3. Looks up the QR code in a slot dictionary → gets a (x, y) drop point.
4. **Plans a path** through the warehouse using A\* search.
5. **Drives the path** with a differential-drive rover while
   avoiding walls (ultrasonic sensor) and obstacles on the map.
6. **Picks up / drops off** the package with a two-servo gripper.
7. Serves a **live web admin panel** so a human can watch the
   camera, see the planned route, manually drive it, start an
   autonomous mission, and read telemetry — all from a browser.

### High-level architecture

```
                       +-------------------+
   +------+           |  RPi4  (mission)  |          +--------+
   | USB  |  MJPEG -->|  camera, QR, plan,|--> UART->| ESP32  |
   | cam  |           |  steer, odometry  |          |(low lv)|
   +------+           +-------------------+          +---+----+
       |                                                  |
       v                                                  v
   /dev/video0                                        L298N motors
   640x480                                            2x servos
                                                      HC-SR04
                                                      MPU6050
```

* **RPi4 = the brain.** Perception, planning, and high-level
  decisions in Python. Hosts the web admin panel on port 8080.
* **ESP32 = the low-level controller.** Real-time-ish things:
  motor PWM, servo PWM, ultrasonic pulse, IMU sampling.
* They talk over **USB serial** at 115200 baud using a tiny,
  line-based protocol described in §7.

The **admin panel is the single user-facing entry point.** It
embeds the auto-drive routine: a "Drive to slot" button on the
HTML page triggers a background-thread mission that scans a QR
(or uses the one currently in view), plans a path with A\*, and
follows it autonomously. No separate `main.py` is needed.

---

## 2. Repository Layout (after cleanup)

```
MicroProject/
├── README.md                                    (this file)
└── autonomous_rover/
    ├── setup.sh                                 One-time setup
    ├── requirements.txt
    ├── run_tests.py                             16 unit tests
    ├── generate_qr_codes.py                     Regenerate QR PNGs
    ├── config/
    │   ├── settings.yaml                        All tunables
    │   └── warehouse_slots.yaml                 Slot ID -> drop/approach
    ├── maps/
    │   ├── floorplan.png                        200x200 mock top view
    │   └── occupancy_grid.npy                   Cached inflated grid
    ├── qrcodes/                                 R1C1.png … R3C3.png
    ├── esp32/rover_firmware/
    │   └── rover_firmware.ino                   Upload to ESP32
    └── src/
        ├── __init__.py
        ├── admin_panel.py                       Web admin (HTTP + MJPEG)
        ├── mission_controller.py                Auto-drive orchestrator
        ├── comms.py                             Serial link to ESP32
        ├── planner.py                           A* + line-of-sight pruner
        ├── navigator.py                         P-controller waypoint follower
        ├── odometry.py                          Wheel-encoder / yaw pose
        ├── map_processor.py                     Image -> occupancy grid
        └── warehouse_slots.py                   Slot config + normaliser
```

---

## 3. Hardware & Wiring

### 3.1 The two boards

| Board  | Role                                                                 |
| ------ | -------------------------------------------------------------------- |
| RPi 4  | Runs Python. Camera, QR, A\*, navigation, web UI.                    |
| ESP32  | Runs Arduino sketch. Motors, servos, ultrasonic, MPU6050, serial.    |

Connected by a **single USB cable** between RPi4 and ESP32. The
ESP32 shows up as `/dev/ttyACM0` at **115200 baud**.

### 3.2 ESP32 pin map

| Component           | Signal | ESP32 Pin |
| ------------------- | -----: | --------: |
| Gripper servo 1     | PWM    | **GPIO18** |
| Gripper servo 2     | PWM    | **GPIO19** |
| Ultrasonic sensor   | TRIG   | **GPIO12** |
| Ultrasonic sensor   | ECHO   | **GPIO13** |
| L298N motor driver  | IN1    | **GPIO25** |
| L298N motor driver  | IN2    | **GPIO33** |
| L298N motor driver  | IN3    | **GPIO32** |
| L298N motor driver  | IN4    | **GPIO27** |
| MPU6050             | SDA    | **GPIO21** |
| MPU6050             | SCL    | **GPIO22** |
| MPU6050             | VCC    | 3.3V      |
| MPU6050             | GND    | GND       |

Reserved for future encoders: **GPIO34** (left), **GPIO35** (right).
GPIO34/35 are **input-only** — they cannot drive outputs. That is
why the motor IN4 went from GPIO34 to GPIO27 in the change log.

### 3.3 L298N wiring

```
ESP32 GPIO25 ──> L298N IN1   ┐
ESP32 GPIO33 ──> L298N IN2   ├─ Left side motors  → OUT1 / OUT2
ESP32 GPIO32 ──> L298N IN3   ┐
ESP32 GPIO27 ──> L298N IN4   ├─ Right side motors → OUT3 / OUT4
```

The L298N **ENA / ENB jumpers are assumed installed** — speed
control is effectively all-or-nothing. For true PWM speed control
later, wire ENA/ENB to two ESP32 PWM pins and set
`MOTOR_LEFT_ENABLE_PIN` / `MOTOR_RIGHT_ENABLE_PIN` in the firmware
(currently `-1` = "jumper installed, ignore").

### 3.4 Power & ground

Critical — all grounds must be common:

```
ESP32 GND   ─┐
RPi4 GND    ─┤  (through the USB cable)
Servo GND   ─┤
L298N GND   ─┤
MPU6050 GND ─┤
HC-SR04 GND ─┘
```

* Use a **separate 5 V supply** for the servos (a USB charger or
  buck converter is fine). The ESP32's 3.3 V pin cannot power
  servos well — current spikes will brown-out the ESP32.
* Use the **motor battery** (often 7–12 V) for L298N motor power.
* If the HC-SR04 is powered at 5 V, its ECHO pin is 5 V. **Add a
  voltage divider** to GPIO13 (e.g. 1 kΩ + 2 kΩ) — the ESP32 is
  3.3 V tolerant on inputs only up to 3.3 V.
* GPIO12 is an ESP32 **strapping pin**. If the board refuses to
  boot after wiring TRIG to GPIO12, move TRIG to a different safe
  output pin and update the firmware constant.

---

## 4. Components In Depth

### 4.1 ESP32 (DOIT DevKit V1 or equivalent)

* Dual-core Tensilica LX6, 240 MHz.
* Built-in WiFi/BT (not used here — communication is wired USB).
* 3.3 V logic, 5 V tolerant on input on most pins.
* The Arduino-ESP32 core + the `ESP32Servo` library provide
  `Servo::attach()` and PWM. Up to 16 simultaneous PWM channels.
* We use `Wire.begin(SDA, SCL)` to put I²C on **any two GPIO pins**
  (here 21/22) — the ESP32 supports `Wire` on any pin.

### 4.2 L298N dual H-bridge motor driver

* Two H-bridges, one per motor (or per side of the chassis).
* `IN1`/`IN2` direction pins, `ENA` enable / speed pin.
* Logic supply: 5 V. Motor supply: up to 35 V.
* Direction table for one side:

  | IN1 | IN2 | ENA | Motor |
  | :-: | :-: | :-: | :---- |
  |  0  |  0  |  *  | stop (coast) |
  |  0  |  1  |  1  | forward      |
  |  1  |  0  |  1  | reverse      |
  |  1  |  1  |  *  | brake        |

* The firmware implements this in `driveLeftMotor()` /
  `driveRightMotor()`. With ENA jumpered, the speed is
  effectively "always full" and the PWM magnitude is just a
  "did we go forward / backward?" signal.

### 4.3 HC-SR04 ultrasonic distance sensor

* 4 pins: VCC (5 V), TRIG (input), ECHO (output, 5 V), GND.
* The host pulses TRIG HIGH for ≥ 10 µs. The sensor emits a 40 kHz
  burst and raises ECHO HIGH for a duration proportional to the
  round-trip time.
* Distance = `(echo_high_time_us × 0.0343 cm/µs) / 2`.
* `pulseIn(ECHO, HIGH, 30000)` returns 0 on timeout (30 ms ≈ 5 m
  max). The firmware converts that to `-1.0` so the RPi can
  distinguish "no echo" from "very far".
* The firmware has a **hardware safety**: if the distance drops
  below `STOP_DISTANCE_CM = 12.0` cm, it **automatically stops the
  motors** even if the RPi hasn't noticed yet.

### 4.4 MPU6050 IMU (6-axis)

* I²C address `0x68`.
* We only use the **Z-axis gyroscope** to integrate yaw.
* On boot we:
  1. Wake the device by writing 0 to register `0x6B`.
  2. **Calibrate the gyro bias** by averaging 500 samples while
     stationary.
  3. From then on, in `updateYaw()`, read the raw Z rate
     (register `0x47`), subtract the bias, divide by **131.0 LSB/(°/s)**
     (the ±250 °/s full-scale factor), and integrate.
* **Caveat:** the MPU6050 is **not a compass**. Yaw drifts
  continuously. For a robot that drives more than a few metres you
  need an external absolute heading source (magnetometer, visual
  fiducials, etc.).

### 4.5 Two hobby servos (the gripper)

* Standard 50 Hz PWM, 1 ms pulse = 0°, 2 ms pulse = 180°.
* The gripper is a two-jaw parallel mechanism. Each servo drives
  one jaw. We **step the angle one degree at a time with 20 ms
  between steps** to avoid jerking the servo gears and to make
  grasping gentler.
* Open / close angles are tunable:
  * Servo 1: 25° (open) → 95° (closed)
  * Servo 2: 155° (open) → 85° (closed)

### 4.6 USB camera

* Any UVC camera. The RPi4 sees it as `/dev/video0` and OpenCV
  opens it by integer index.
* Used at **640×480** by default — enough for QR detection and not
  heavy on the RPi4's CPU.

### 4.7 Raspberry Pi 4

* 4 GB RAM is plenty. The stack uses ~150 MB of RSS.
* The USB-A port supplies power to the ESP32 and the camera. Use a
  powered hub if the camera is high-current.

---

## 5. The Map and Slots

### 5.1 `maps/floorplan.png` (200×200 px)

A top-down drawing:

```
   0           x grows →         199
 0 ┌──────────────────────────────┐
   │  free  │shelf│ free │shelf│ … │
   │        │  1  │      │  2  │   │
   │        │     │      │     │   │
   │        │     │      │     │   │
   │        │     │      │     │   │
160└──────────────────────────────┘
```

Three vertical "shelves" at x=40..60, x=100..120, x=150..170.
The aisles are: `x=4..39` (left), `x=61..99` (mid-1),
`x=121..149` (mid-2), `x=171..195` (right). All other pixels are
white (free).

The generated `occupancy_grid.npy` is a 200×200 uint8 array:
* `0` = free
* `1` = obstacle (or safety-inflated obstacle)

About **39.5 %** of the grid is marked as obstacle after the safety
inflation.

### 5.2 `config/warehouse_slots.yaml`

A dictionary from a **slot ID** to a (x, y) drop point and an
optional (x, y) approach point. The approach point is the cell
the robot aims for first; the drop point is the final cell.

```yaml
home: [20, 20]
slots:
  R1C1:
    drop: [190, 50]
    approach: [180, 50]
  ...
  R3C3:
    drop: [145, 130]
    approach: [140, 130]
```

Image-style convention: **x grows right, y grows down**. There
are 9 slots arranged 3 rows × 3 columns of aisles.

---

## 6. The Python Stack

All Python source lives in `autonomous_rover/src/`.

### 6.1 `src/admin_panel.py` — entry point + web UI

This is the **only** file the user runs. It:

1. Starts a background camera thread that grabs frames, runs QR
   detection, and stores the latest JPEG + slot ID.
2. Opens the ESP32 serial link.
3. Loads the slot table and occupancy grid.
4. Hosts an HTTP server with:
   * `GET /` — single-page HTML app
   * `GET /video` — MJPEG stream
   * `GET /api/status` — JSON status (incl. auto-drive state)
   * `GET /api/snapshot.jpg` — single camera frame
   * `GET /api/map.jpg` — grid with planned route overlay
   * `POST /api/command?action=…` — manual nudge / gripper
   * `POST /api/drive?action=start&slot=R1C1` — **autonomous drive**
   * `POST /api/drive?action=stop` — abort autonomous drive

The `CameraWorker` daemon thread runs at ~30 Hz:
* reads a frame from the USB camera,
* runs `cv2.QRCodeDetector().detectAndDecode`,
* draws a green polygon around the QR,
* encodes JPEG and stores it under a lock.

`AdminState` owns the camera, the ESP32 link, the slot table, the
occupancy grid, the current `pose`, the manual-nudge lock, and the
`AutoDriver` background thread. The `/api/status` route exposes
all of it as JSON for the browser's JS poll loop.

The **Auto-drive card** on the HTML page lets the user:
* pick a slot from a dropdown (populated from
  `warehouse_slots.yaml`),
* click "Start auto-drive" — sends `POST /api/drive?action=start&slot=…`,
* click "Stop" — sends `POST /api/drive?action=stop`,
* watch a live status line (`R1C1 - driving (1/3)`).

#### How the Auto-drive thread works

`AutoDriver` is a `threading.Thread` subclass that wraps a
`MissionController` (see §6.2) in a daemon thread. Its lifecycle:

1. Spawned by `AdminState.start_auto_drive(slot_id)` if no mission
   is running. Returns 409 if one already is.
2. Builds a `MissionController` with the same `settings.yaml` and
   the requested `slot_id`.
3. Plans the path with A\* and prunes it to corner waypoints.
   Stores the waypoint count in `waypoints_total`.
4. Drives waypoint-by-waypoint, updating `waypoints_done` as it
   progresses.
5. Sets `status = "done"` on success, `"error"` on exception (with
   the message in `error`), or `"aborted"` if the user pressed Stop.

The mission's `_follow_waypoints` method is wrapped so the thread
can update the status JSON after each waypoint.

### 6.2 `src/mission_controller.py` — auto-drive orchestrator

`MissionController.run_once()` is the autonomous mission loop:

```
1.  if not dry_run: open the gripper (so we can pick up a fresh pkg)
2.  slot_id = self._get_slot_id()           # from override or QR scan
3.  destination = slots.get_destination(slot_id)
4.  target = destination.navigation_target  # approach or drop
5.  plan + prune waypoints with A*
6.  if dry_run: return
7.  gripper CLOSE (pick up the package)
8.  send <RESET_TICKS> (zero encoders / yaw)
9.  _follow_waypoints(waypoints)             # 10 Hz loop
10. esp32.stop()
11. gripper OPEN (deliver)
```

`_get_slot_id()`:
* If `--qr` was passed, use it.
* Otherwise instantiate a `QRScanner` and wait for a real QR.

`_plan_waypoints(target)`:
* Round current pose to integer pixels.
* `dense = planner.plan_path(start, target)`. Raise if `None`.
* `pruned = planner.prune_path(dense)`. Drop the first point if
  it equals the start.

`_follow_waypoints(waypoints)` — the main 10 Hz control loop:
1. Read latest telemetry.
2. Update odometry (only if `odometry.enabled`).
3. **Ultrasonic safety**: if `distance_cm > 0 and <
   obstacle_stop_cm`, stop and wait until it climbs back above
   `obstacle_resume_cm`. This handles a real, physical obstacle
   that the map does not know about.
4. `(v, w, arrived) = navigator.get_steering_commands(pose, target)`.
5. If arrived, pop the waypoint, `stop()`, continue.
6. Convert to `(L, R)` PWM, clip, send.
7. `time.sleep(loop_delay)`.

If `odometry.enabled = false` (the default; current hardware has
no encoders), we run in **dead-reckoning** mode: the RPi sends a
single timed pulse per cycle and integrates the assumed
forward/turn rate in software.

If the loop exceeds `max_mission_seconds` (default 180 s) with
waypoints still remaining, raise `TimeoutError`.

`MissionController` also has a CLI `main()` (kept for tests; not
exposed in normal operation).

### 6.3 `src/comms.py` — serial link to ESP32

`ESP32Interface` is a thin wrapper over `pyserial`.

* **Open**: `serial.Serial(port, baudrate, timeout=1)`.
* **Simulation mode**: if `port` is `None` / `"NONE"` or `pyserial`
  is missing or open fails, fall into `simulation_mode = True` and
  silently drop every send. `get_telemetry()` still returns a
  default snapshot. This is the cornerstone of testability.
* **Send commands** (newline-terminated):
  * `<V,left,right>` — `send_velocity_cmd(l, r)`. Clipped to ±255.
  * `<G,OPEN>` / `<G,CLOSE>` — `send_gripper_cmd(action)`.
  * `<PING>` — handshake.
  * `<RESET_TICKS>` — zero encoders and yaw.
* **Receive** in a background thread:
  * `<T,distance_cm,left_ticks,right_ticks,yaw_deg>` →
    `Telemetry` dataclass, guarded by a lock.
  * `<PONG>` → set `_last_pong` (used by `ping()`).
  * `<READY>` and any other line are consumed (so the RX buffer
    does not fill up) and discarded.

A single `threading.Lock` guards the telemetry struct and
`_last_pong`. `get_telemetry()` **copies** the dataclass so the
caller cannot see a half-updated state.

### 6.4 `src/planner.py` — A\* + path pruning

A grid A\* planner with 8-connected neighbours and an Euclidean
heuristic. Two public methods:

* **`plan_path(start, goal) -> list[(x, y)] | None`** — runs A\* and
  returns the dense path (every cell along the way) or `None` if
  no path exists.
* **`prune_path(path) -> list[(x, y)]`** — collapses the dense
  path down to "geometric corners" using line-of-sight.

#### A\* algorithm

* Open set is a binary heap (`heapq`), keyed by `f = g + h`.
* `g_score[node]` is the best known cost from the start.
* `came_from[node]` is the back-pointer for path reconstruction.
* The 8 neighbours have cost 1 (cardinal) or √2 (diagonal).
  Diagonal moves are **only allowed if both side neighbours are
  also free** — that prevents corner-cutting through walls.
* Heuristic: Euclidean distance. **Admissible** because every step
  costs ≥ its Euclidean length.
* On reaching the goal, walk `came_from` backwards to reconstruct
  the path, then reverse it.

#### `has_line_of_sight(p1, p2)`

Bresenham's line algorithm. Steps along the line and checks every
cell. If any cell is out of bounds or an obstacle, returns False.
Used by the pruner.

#### `prune_path(path)` — line-of-sight compressor

Greedy. From each waypoint, look at every later waypoint. The
first later waypoint that has line-of-sight becomes the new
"current" waypoint. Repeat. A path of ~500 dense cells typically
collapses to ~5–10 corner waypoints.

### 6.5 `src/navigator.py` — waypoint follower

`WaypointNavigator` is a **P-controller** that turns
`current_pose = (x, y, θ)` and a `target = (x, y)` into a
`(linear_v, angular_w, arrived)` triple.

```
dx, dy   = target − current
dist     = hypot(dx, dy)
if dist < tolerance:    return 0, 0, True

desired_heading = atan2(dy, dx)
heading_error   = wrap(desired_heading − θ)

if |heading_error| > 0.2 rad (~11°):   # turn in place
    linear_v  = 0
    angular_w = Kp_angular × heading_error
else:                                  # drive forward
    linear_v  = Kp_linear × dist
    if linear_v < min_linear_vel:
        linear_v = min_linear_vel        # creep forward
    angular_w = Kp_angular × heading_error
```

* `Kp_linear = 1.2`, `Kp_angular = 3.5` (from `settings.yaml`).
* `min_linear_vel = 35`, `max_linear_vel = 90`, `max_angular_vel = 90`.

#### `unicycle_to_differential(v, omega)`

Standard conversion:
```
left_pwm  = clip(v − ω, −255, 255)
right_pwm = clip(v + ω, −255, 255)
```
A positive `ω` means turn left → the right wheel goes faster than
the left.

### 6.6 `src/odometry.py` — dead-reckoning pose

`DifferentialOdometry` maintains `pose = [x, y, θ]` in the map frame.

Given:
* `wheel_diameter_cm = 6.5`
* `wheel_base_cm = 15.0`
* `ticks_per_revolution = 20`
* `resolution_cm_per_px = 1.0`
* `yaw_sign = 1.0`

It computes `cm_per_tick = π × wheel_diameter / ticks_per_rev`. On
each `update(left_ticks, right_ticks, yaw_deg=None)`:

* `delta_left_cm = (left_ticks − last_left) × cm_per_tick`
* `delta_right_cm = (right_ticks − last_right) × cm_per_tick`
* `distance_cm = (delta_left + delta_right) / 2`
* If `yaw_deg` is given, set `θ = (yaw_deg − yaw_zero) × yaw_sign`
  (in radians). This overrides wheel-derived heading — good for
  fighting wheel slip.
* Else, derive `Δθ = (Δright − Δleft) / wheel_base` and integrate.
* Advance position: `x += d·cos(θ)`, `y += d·sin(θ)` after
  converting cm → px by dividing by `resolution_cm_per_px`.

### 6.7 `src/map_processor.py` — image to occupancy grid

`MapProcessor.generate_occupancy_grid(image_path, output_path)`
turns a top-down map image into a 0/1 grid.

Steps:

1. **Load grayscale**.
2. **Gaussian blur** (5×5) to smooth scan noise / paper texture.
3. **Otsu's threshold** with `THRESH_BINARY_INV`: Otsu auto-picks
   the cut-point between the two pixel populations. `INV` makes
   dark pixels (walls) become 255 and light pixels (floor) become 0.
4. **Connected components filter**: tiny blobs (text, single-pixel
   specks) below `min_obstacle_area_px = 40` are dropped — the
   "text removal filter".
5. **Inflate (dilate) obstacles** by a kernel of radius
   `ceil(robot_radius_cm / resolution_cm_per_px)`. This is the
   **safety margin** — the planner keeps at least one
   robot-radius away from any wall.
6. **Save** the resulting 0/1 grid as `.npy` and return it.

### 6.8 `src/warehouse_slots.py` — slot config + normaliser

* **`normalize_slot_id(payload)`** — turns any reasonable QR string
  into the canonical `R<row>C<col>` form.
  * Strips, uppercases, removes `-` and `_`.
  * Matches the regex `R0*(\d+)C0*(\d+)`.
  * Returns `f"R{row}C{col}"` with leading zeros stripped.
  * Examples: `"r1c3"` → `"R1C3"`, `"R01C03"` → `"R1C3"`,
    `"r-1_c-3"` → `"R1C3"`. Anything else raises `ValueError`.
* **`SlotDestination`** — frozen dataclass with `slot_id`, `drop`,
  `approach`. The `navigation_target` property returns the
  approach point if it exists, else the drop point.
* **`WarehouseSlots.load(path)`** — reads the YAML, builds a
  `dict[slot_id → SlotDestination]`, returns a `WarehouseSlots`.
* **`get_destination(slot_id)`** — looks up by normalised id and
  raises `KeyError` with a helpful message that lists all known
  slots.

### 6.9 `src/__init__.py`

Empty file. Its only purpose is to make `src/` a Python package so
relative imports work.

---

## 7. The Serial Protocol

```
RPi4 → ESP32  (newline-terminated, ASCII):
    <V,left,right>   signed motor PWM, range −255..+255
    <G,OPEN>         open gripper
    <G,CLOSE>        close gripper
    <PING>           handshake
    <RESET_TICKS>    zero encoder counters and yaw

ESP32 → RPi4:
    <T,distance_cm,left_ticks,right_ticks,yaw_deg>
    <PONG>
    <READY>            sent once on boot
```

* Commands and replies are bracketed by `<` `>` so a partial line
  (e.g. someone `cat`-ing the port in another window) cannot be
  misread as a complete command.
* Telemetry is sent at 10 Hz (`TELEMETRY_INTERVAL_MS = 100`).
* Encoder snapshots are taken inside a `noInterrupts()` /
  `interrupts()` pair so they are atomic with respect to the
  encoder ISRs.

---

## 8. End-to-End User Flow

1. Plug in the RPi4, ESP32, and USB camera. Power on.
2. On the RPi4: `bash setup.sh` (one time).
3. On the RPi4: `.venv/bin/python src/admin_panel.py`.
4. On any phone/laptop on the same network, open
   `http://<RPi4-IP>:8080`.
5. The page shows the live camera feed, the warehouse map, and
   status cards.
6. Stick a QR label (e.g. `R1C1`) on a package and put it in
   front of the camera. The QR is detected and shown on the page.
7. Click **"Start auto-drive"**. The robot:
   * closes the gripper (picks up the package),
   * plans a path from home to R1C1 with A\*,
   * drives each waypoint,
   * opens the gripper (delivers),
   * reports `done` on the page.
8. The map preview updates in real time showing the planned route.
9. If something goes wrong, click **"Stop"** — the motors halt,
   the mission aborts.

Manual controls (forward / backward / left / right / stop /
gripper_open / gripper_close) are still on the page for recovery.

---

## 9. Configuration Reference

`config/settings.yaml` (full key list):

| Key                                 | Default               | What it does                                  |
| ----------------------------------- | --------------------- | --------------------------------------------- |
| `serial.port`                       | `/dev/ttyACM0`        | ESP32 USB serial                              |
| `serial.baudrate`                   | `115200`              | Match the Arduino `Serial.begin`              |
| `camera.index`                      | `0`                   | `cv2.VideoCapture(0)`                         |
| `camera.width / .height`            | `640 / 480`           | Capture resolution                            |
| `camera.qr_timeout_s`               | `30`                  | Max seconds to wait for a QR                  |
| `map.image`                         | `maps/floorplan.png`  | Source blueprint                              |
| `map.grid`                          | `maps/occupancy_grid.npy` | Cached grid                              |
| `map.resolution_cm_per_px`          | `1.0`                 | How big a pixel is in real life               |
| `map.robot_radius_cm`               | `4.0`                 | Safety inflation radius                       |
| `slots.file`                        | `config/warehouse_slots.yaml` | Slot table                          |
| `navigation.home`                   | `[20, 20]`            | Start pose                                    |
| `navigation.obstacle_stop_cm`       | `18.0`                | Stop if ultrasonic < this                     |
| `navigation.obstacle_resume_cm`     | `25.0`                | Resume once > this                            |
| `navigation.loop_hz`                | `10`                  | Control loop frequency                        |
| `navigation.target_tolerance_px`    | `5.0`                 | "Arrived" radius                              |
| `navigation.max_mission_seconds`    | `180`                 | Hard safety timeout                           |
| `navigation.K_linear`               | `1.2`                 | Navigator P-gain for speed                    |
| `navigation.K_angular`              | `3.5`                 | Navigator P-gain for turn                     |
| `navigation.min_linear_vel`         | `35.0`                | Creep speed                                   |
| `navigation.max_linear_vel`         | `90.0`                | Saturation for `v`                            |
| `navigation.max_angular_vel`        | `90.0`                | Saturation for `ω`                            |
| `navigation.max_drive_speed`        | `90`                  | Clip on `(L,R)` PWM (dead-reckon path)        |
| `navigation.max_turn_speed`         | `90`                  | Clip on turn PWM (dead-reckon path)           |
| `odometry.enabled`                  | `false`               | Use encoders? No → dead reckon                |
| `odometry.wheel_diameter_cm`        | `6.5`                 | For cm/tick                                   |
| `odometry.wheel_base_cm`            | `15.0`                | Track width                                   |
| `odometry.ticks_per_revolution`     | `20`                  | Encoder CPR                                   |
| `odometry.yaw_sign`                 | `1.0`                 | Flip if MPU6050 mounted upside-down           |
| `dead_reckoning.forward_speed_cm_per_s` | `20.0`            | Calibrate this!                               |
| `dead_reckoning.backward_speed_cm_per_s`| `18.0`            | Often a bit slower                            |
| `dead_reckoning.turn_speed_deg_per_s` | `60.0`             | Calibrate this!                               |
| `admin.host`                        | `0.0.0.0`             | Bind address                                  |
| `admin.port`                        | `8080`                | HTTP port                                     |
| `admin.manual_speed`                | `90`                  | PWM for manual nudges                         |
| `admin.nudge_duration_s`            | `0.35`                | How long a manual nudge runs                  |

---

## 10. Algorithms Recap (math)

### A\* heuristic
```
h(a, b) = √((a.x − b.x)² + (a.y − b.y)²)
```
Admissible (≤ true cost on the grid), consistent, gives an optimal
8-connected path.

### Bresenham line-of-sight
```
dx, dy  = abs(b − a)
sx, sy  = sign(b − a)
err     = dx − dy  (or dx/2, dy/2 depending on variant)
while not at b:
    if grid[y][x] is obstacle:  return False
    err2 = err
    if err2 > −dx:  err −= dy;  x += sx
    if err2 <  dy:  err += dx;  y += sy
return True
```

### Unicycle → differential drive
```
L = v − ω
R = v + ω
clip both to ±max_pwm
```
(`ω > 0` = turn left.)

### Differential-drive forward kinematics
```
v       = (v_R + v_L) / 2
ω       = (v_R − v_L) / wheel_base
x'      = x + v·cos(θ)·dt
y'      = y + v·sin(θ)·dt
θ'      = θ + ω·dt
```

### Wheel ticks → cm
```
cm_per_tick = π · wheel_diameter / ticks_per_revolution
delta_cm    = (ticks_now − ticks_last) · cm_per_tick
```

### Gyro → yaw
```
gyro_z_dps  = (raw_z − bias) / 131.0   // 131 LSB/(°/s) at ±250°/s
yaw_deg    += gyro_z_dps · dt          // integrate
```

### Ultrasonic → cm
```
distance_cm = (echo_high_us · 0.0343) / 2
```

---

## 11. Testing

`run_tests.py` runs all of the following with stdlib `unittest`:

| Test class              | What it verifies                                  |
| ----------------------- | ------------------------------------------------- |
| `TestWarehouseSlots`    | Slot normaliser accepts variants, loads the YAML  |
| `TestPlanner`           | A\* finds a path, pruner reduces it, walls block  |
| `TestNavigator`         | Arrival, steering, turning, unicycle conversion   |
| `TestOdometry`          | Reset, update with yaw, update without yaw        |
| `TestComms`             | Simulation mode, invalid gripper, speed clipping |
| `TestMissionDryRun`     | Full dry-run mission to R1C1                      |

Current: **16/16 passed**.

---

## 12. Why Everything Works (end-to-end summary)

* **The map is one consistent frame.** All coordinates —
  warehouse slots, planner cells, odometry pose, mission target —
  are in the same `(x, y)` pixel frame as `floorplan.png`.
  `resolution_cm_per_px` is the only place that maps pixels to
  real distances.
* **A\* is the source of truth for "where can I go?"** Every
  other module trusts the grid: the planner's pruner, the mission
  controller, the admin panel's route preview.
* **The navigator + dead-reckoning step form a closed loop**
  that always steers toward the next waypoint. Even when
  odometry is off, the visualised route in the admin panel and
  the printouts in dry-run let you see what the robot intends to
  do.
* **The serial protocol is line-based and ASCII.** You can `cat
  /dev/ttyACM0` in another shell and watch the telemetry stream.
* **The simulation mode in `comms.py` is the cornerstone of
  testability.** Every code path that talks to the ESP32 has a
  silent no-op fallback, so the entire Python stack runs on a
  laptop with zero hardware.
* **The admin panel is the single user-facing entry point.** It
  embeds the auto-drive routine so the user never has to invoke a
  separate script. The mission logic is reused by spawning a
  `MissionController` inside a daemon thread and streaming its
  status back to the page.

---

## 13. Known Limitations

| Limitation                                | Why                                                | Fix                                           |
| ----------------------------------------- | -------------------------------------------------- | --------------------------------------------- |
| No wheel encoders wired                   | Current hardware                                   | Add hall-effect encoders, set `odometry.enabled: true` |
| MPU6050 yaw drifts                        | Gyro-only integration                              | Add magnetometer, or use visual fiducials     |
| HC-SR04 ECHO is 5 V into a 3.3 V pin      | Sensor powered at 5 V                              | Voltage divider on ECHO                       |
| ENA/ENB jumpers installed                 | "Speed" is effectively full / off                  | Wire ENA/ENB to PWM pins and update firmware  |
| `dialout` group not added automatically  | `setup.sh` could not get an interactive password   | Run `sudo usermod -a -G dialout $USER` manually |
| No camera on the auto-drive hot path     | If the camera is missing, "Start auto-drive" still works if a `slot_id` is supplied via the slot selector — it just uses that fixed slot instead of scanning. | Plug a UVC camera into a USB port             |
| No process supervisor                     | RPi reboots kill the admin panel                   | `systemd` unit file (not yet written)         |

---

## 14. TL;DR

* The **RPi4** is the brain. It owns the camera, the QR, the A\*
  path, the navigator, and the web UI. It tells the **ESP32** what
  to do over USB serial.
* The **ESP32** is the low-level controller. It owns the motors,
  the gripper, the ultrasonic, and the IMU. It streams telemetry
  back to the RPi4 ten times a second.
* The **map** is a single image (`floorplan.png`) turned into a
  0/1 grid. **A\*** finds a path on that grid, the pruner shrinks
  the path to corner waypoints, the **navigator** drives to each
  waypoint, and the **ultrasonic** acts as a last-line-of-defence
  stop.
* **No encoders** are currently wired, so the navigator runs in
  **dead-reckoning** mode: it sends a single timed pulse per loop
  and integrates the assumed forward/turn rate in software.
* The **admin panel** is a self-contained `http.server` that
  streams MJPEG, exposes JSON status + manual nudges, **and
  drives the robot autonomously** when you click "Start
  auto-drive". It is the **only** command the user runs.
* **Tests** run end-to-end with no hardware thanks to
  `simulation_mode` in `comms.py`. 16/16 pass.
