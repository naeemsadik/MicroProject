# Warehouse Robot End-to-End Automation Plan

## Summary
Build the system as two cooperating controllers:

- **RPi4:** mission brain. Scans QR, maps row/column to warehouse coordinates, generates path, tracks estimated position, and streams velocity/gripper commands.
- **ESP32:** low-level controller. Drives motors, moves gripper servos, reads ultrasonic sensor, reads wheel encoders, reads MPU6050 gyro data, and sends telemetry back to the RPi4.

Important correction: **MPU6050 is not a compass**. It is an accelerometer/gyro. It can help estimate heading by integrating gyro yaw, but it will drift over time. For a real compass heading, add a magnetometer such as QMC5883L/HMC5883L later.

## Key Architecture
- Generate the warehouse occupancy grid from the top-view image **once**, not after every QR scan.
- QR code should contain a row/column ID, for example:
  - `R1C3`
  - `R02C05`
- Add a warehouse slot config file mapping QR destination IDs to map coordinates:
  - `R1C3 -> pickup/drop point pixel`
  - optional approach point in front of the shelf
- RPi4 mission flow:
  - Start robot.
  - Open gripper.
  - Scan QR from package using USB camera.
  - Decode row/column.
  - Look up destination coordinates.
  - Plan path from current estimated pose to destination.
  - Send signed left/right velocity commands to ESP32.
  - Stop near destination.
  - Release object.
  - Optionally return to home.

## Required Code Updates
- Update `requirements.txt` with at least:
  - `numpy`
  - `opencv-python`
  - `matplotlib`
  - `pyserial`
  - `PyYAML`
- Fix current Python bugs:
  - Move `unicycle_to_differential()` inside `WaypointNavigator`.
  - Fix map display orientation so selected coordinates match `grid[y][x]`.
  - Fix obstacle inflation in `MapProcessor` to use roughly `2 * robot_radius_px + 1`.
  - Prevent diagonal A* corner-cutting through blocked shelf corners.
- Add new RPi modules:
  - `qr_scanner.py`: uses USB camera and OpenCV QR detection.
  - `warehouse_slots.py`: loads row/column-to-coordinate config.
  - `odometry.py`: estimates robot pose from left/right encoder ticks plus MPU6050 yaw rate.
  - `mission_controller.py`: replaces the current static target flow in `main.py`.
- Replace the old `autonomous_rover/esp32/rover_firmware/rover_firmware.ino`.
  - It currently uses Arduino-style PWM pins and does not match your ESP32/L298N wiring.
  - Base the new firmware on `WarehouseRobot.ino`, but remove the fixed timed sequence.
  - Add serial command parsing from the RPi4.

## RPi4 ↔ ESP32 Protocol
Use simple line-based serial messages.

RPi4 to ESP32:
- `<V,left,right>` signed motor speeds, range `-255` to `255`
  - Example: `<V,140,140>` forward
  - Example: `<V,-100,100>` turn left/right depending on motor calibration
  - Example: `<V,0,0>` stop
- `<G,OPEN>` open gripper
- `<G,CLOSE>` close gripper
- `<PING>` optional connection check

ESP32 to RPi4:
- `<T,distance_cm,left_ticks,right_ticks,yaw_deg>`
  - `distance_cm`: ultrasonic distance
  - `left_ticks/right_ticks`: encoder counts
  - `yaw_deg`: MPU6050 gyro-integrated yaw estimate

Default encoder wiring plan:
- Left encoder signal: `GPIO34`
- Right encoder signal: `GPIO35`
- Use external pull-up resistors if the encoder outputs need them, because GPIO34/GPIO35 do not have internal pull-ups.

## Navigation Plan
- Use A* to generate a path on the occupancy grid.
- Prune the path into waypoints.
- RPi4 estimates current pose using:
  - wheel encoder distance
  - differential-drive kinematics
  - MPU6050 gyro yaw correction
- RPi4 sends continuous velocity commands to ESP32 until each waypoint is reached.
- Ultrasonic safety remains on ESP32 and RPi4:
  - ESP32 can stop immediately if an obstacle is too close.
  - RPi4 can pause/replan if telemetry reports blocked path.

## Test Plan
- First test map generation only:
  - Load warehouse image.
  - Generate occupancy grid.
  - Verify shelves/walls are obstacles and aisles are free.
- Test QR only:
  - Show QR to USB camera.
  - Confirm decoded `R#C#`.
  - Confirm correct destination lookup.
- Test ESP32 serial only:
  - Send `<V,120,120>`, `<V,0,0>`, `<G,OPEN>`, `<G,CLOSE>` from serial monitor or Python.
- Test telemetry:
  - Spin wheels by hand and confirm encoder ticks change.
  - Rotate robot and confirm yaw changes.
  - Put object in front and confirm ultrasonic distance changes.
- Test closed-loop navigation with wheels lifted.
- Then test on the floor at slow speed with a short route.
- Only after that, test full QR -> route -> grip -> deliver -> release mission.

## Assumptions
- RPi4 will be the main brain and ESP32 will only handle low-level hardware.
- QR payload will be row/column ID, not raw pixel coordinates.
- Motors either have encoders or encoders will be added; real autonomous navigation should not rely on timed movement.
- MPU6050 will be used as a gyro, not as a compass.
- If long-distance heading drift becomes a problem, add a real magnetometer later.
