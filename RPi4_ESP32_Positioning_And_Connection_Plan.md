# RPi4 + ESP32 Robot Positioning And Connection Plan

## Short Answer

Yes, there is a bypass if you do not have encoders: the RPi4 can **guess** the robot position using:

- the route generated from the warehouse map,
- the known warehouse length and width,
- the commanded motor speed,
- the time spent moving,
- and the MPU6050 gyro yaw estimate.

This method is called **dead reckoning** or **open-loop odometry**.

It can work for a simple demo, but it will not be very accurate for a real warehouse robot. The robot may drift because battery voltage, floor friction, wheel slip, object weight, and motor mismatch all change how far it actually moves.

For your project, the best development path is:

1. Use timed/dead-reckoning movement for the first demo.
2. Add camera-based correction or encoders later.
3. Let the RPi4 make decisions.
4. Let the ESP32 control motors, servos, ultrasonic, and MPU6050.

## Current Hardware Roles

### RPi4 Role

The Raspberry Pi 4 is the main brain.

It should handle:

- USB camera access,
- QR code scanning,
- warehouse map loading,
- row/column destination lookup,
- route planning,
- approximate position estimation,
- sending movement commands to the ESP32,
- deciding when to stop, turn, grip, and release.

### ESP32 Role

The ESP32 is the low-level controller.

It should handle:

- motor direction control through the L298N,
- gripper servo control,
- ultrasonic distance reading,
- MPU6050 gyro reading,
- WiFi admin panel for debug logs,
- receiving commands from the RPi4,
- sending telemetry back to the RPi4.

The ESP32 should not generate the warehouse route. That belongs on the RPi4.

## Can The RPi4 Guess Position Without Encoders?

Yes, but only approximately.

The RPi4 can estimate position like this:

```text
estimated distance = calibrated robot speed x movement time
```

Example:

```text
If the robot moves at 20 cm/s
and it drives forward for 3 seconds,
estimated distance = 20 x 3 = 60 cm
```

If the map scale is known:

```text
1 pixel = 2 cm
60 cm = 30 pixels
```

So the RPi can update the robot position on the map by about 30 pixels.

## Why This Is Not Fully Reliable

Without encoders, the RPi only knows what it **commanded** the robot to do. It does not know what the robot **actually** did.

For example, the RPi may command:

```text
Move forward for 3 seconds
```

But the real robot may move differently because:

- battery voltage dropped,
- one motor is stronger than the other,
- the floor is slippery,
- the robot is carrying a heavy object,
- the wheels slip while turning,
- the L298N wastes voltage and motor power,
- the robot hits a small obstacle,
- the gripper load changes the robot balance.

So the RPi might think:

```text
Robot position: x=300, y=120
```

But the real robot might actually be:

```text
Robot position: x=275, y=145
```

That error grows over time.

## Best Bypass Method For Your Current Robot

Use a hybrid approach:

```text
Map route planning + timed movement + MPU6050 yaw correction + ultrasonic safety
```

This means:

- The RPi4 generates the route.
- The RPi4 breaks the route into short movement segments.
- The ESP32 drives each segment.
- The MPU6050 helps estimate turning angle.
- The ultrasonic sensor stops the robot if something is in front.
- The RPi4 assumes the robot reached each waypoint after the calibrated time.

This is good enough for a controlled demo if:

- the floor is flat,
- the route is short,
- the robot speed is slow,
- the robot starts from a known position,
- the load weight is consistent,
- and the route has wide margins.

## Required Calibration

Before autonomous movement, measure these values.

### 1. Forward Speed

Place the robot on the floor and run it forward for 5 seconds.

Measure how far it moved.

Example:

```text
Distance moved: 100 cm
Time: 5 seconds
Speed: 100 / 5 = 20 cm/s
```

Save this as:

```text
FORWARD_SPEED_CM_PER_SEC = 20
```

### 2. Backward Speed

Do the same backward.

Example:

```text
BACKWARD_SPEED_CM_PER_SEC = 18
```

Backward speed may not be exactly the same as forward speed.

### 3. Turn Speed

Command the robot to turn right for a fixed time.

Example:

```text
Turn right for 2 seconds
Measured rotation: 90 degrees
Turn speed: 90 / 2 = 45 degrees/s
```

Save:

```text
TURN_SPEED_DEG_PER_SEC = 45
```

### 4. Map Scale

If your warehouse image is 1000 pixels wide and the real warehouse is 500 cm wide:

```text
500 cm / 1000 px = 0.5 cm per pixel
```

Save:

```text
RESOLUTION_CM_PER_PX = 0.5
```

## Position Estimation Formula

The RPi stores the estimated robot pose:

```text
x, y, theta
```

Where:

- `x` = horizontal position on the map,
- `y` = vertical position on the map,
- `theta` = robot heading angle.

When the robot moves forward:

```text
distance_cm = speed_cm_per_sec x time_sec
distance_px = distance_cm / resolution_cm_per_px

x = x + distance_px x cos(theta)
y = y + distance_px x sin(theta)
```

When the robot turns:

```text
theta = theta + turn_degrees
```

The MPU6050 can help correct `theta`, but it will drift over time because it is not a compass.

## Recommended Movement Style Without Encoders

Do not try to follow long smooth paths.

Use short simple commands:

```text
turn to face waypoint
move forward a short distance
stop
check ultrasonic
repeat
```

Good:

```text
Move 20 cm
Stop
Turn 15 degrees
Move 20 cm
Stop
```

Risky:

```text
Drive 5 meters continuously through narrow shelves
```

Short steps reduce error.

## Warehouse Map Requirement

The RPi4 must know:

- real warehouse width in cm,
- real warehouse length in cm,
- map image width in pixels,
- map image height in pixels.

Then it can calculate:

```text
cm_per_pixel_x = warehouse_width_cm / image_width_px
cm_per_pixel_y = warehouse_length_cm / image_height_px
```

If these are not equal, the image may be stretched. For easiest navigation, use a top-view image where scale is consistent.

## QR Code Destination Flow

The QR code should contain a row/column ID:

```text
R1C3
R2C5
R02C04
```

The RPi4 reads the QR code and looks up the target coordinate from a config file:

```yaml
slots:
  R1C3:
    drop: [530, 300]
    approach: [500, 300]
```

The robot should first navigate to the `approach` point, then move slowly to the `drop` point.

## RPi4 To ESP32 Physical Connection

Use USB first. This is the simplest and most stable option.

```text
RPi4 USB port  ->  ESP32 USB cable
```

This gives:

- serial communication,
- easy debugging,
- ESP32 programming access,
- no extra level shifter needed.

On the RPi4, the ESP32 will usually appear as:

```text
/dev/ttyUSB0
```

or:

```text
/dev/ttyACM0
```

## RPi4 To ESP32 Software Protocol

Use line-based serial messages.

### RPi4 Sends To ESP32

Move command:

```text
<V,left,right>
```

Examples:

```text
<V,120,120>     forward
<V,-120,-120>   backward
<V,100,-100>    turn right or left depending on wiring
<V,0,0>         stop
```

Gripper commands:

```text
<G,OPEN>
<G,CLOSE>
```

Ping:

```text
<PING>
```

### ESP32 Sends To RPi4

Telemetry:

```text
<T,distance_cm,left_ticks,right_ticks,yaw_deg>
```

Because there are no encoders right now:

```text
left_ticks = 0
right_ticks = 0
```

Example:

```text
<T,24.5,0,0,12.3>
```

Meaning:

```text
ultrasonic distance = 24.5 cm
encoder ticks = 0, 0
estimated yaw = 12.3 degrees
```

## How The Whole System Will Work

### Step 1: Robot Starts

```text
ESP32 boots
ESP32 connects to WiFi admin panel
ESP32 waits for serial commands from RPi4
RPi4 starts mission controller
```

### Step 2: Object Is Picked

```text
RPi4 sends <G,OPEN>
Robot approaches object manually or by simple routine
RPi4 sends <G,CLOSE>
```

### Step 3: QR Code Is Scanned

```text
RPi4 USB camera scans QR
QR value = R1C3
RPi4 checks warehouse_slots.yaml
Destination coordinate found
```

### Step 4: Route Is Generated

```text
RPi4 loads occupancy grid
RPi4 runs A* path planner
RPi4 creates waypoints
```

### Step 5: Route Is Converted To Simple Movements

Without encoders, use movement chunks:

```text
turn toward waypoint
move forward for calculated time
stop
update estimated position
repeat
```

### Step 6: ESP32 Executes Commands

The RPi4 sends:

```text
<V,100,-100>
```

then:

```text
<V,0,0>
```

then:

```text
<V,120,120>
```

The ESP32 only follows commands. It does not decide the route.

### Step 7: Safety Check

The ESP32 constantly reads ultrasonic distance.

If an object is too close:

```text
ESP32 stops motors
ESP32 sends distance telemetry
RPi4 pauses route
```

### Step 8: Drop Object

When the RPi4 thinks the robot reached the destination:

```text
RPi4 sends <V,0,0>
RPi4 sends <G,OPEN>
```

## Recommended Implementation Plan

### Phase 1: Serial Link

Goal: RPi4 can command ESP32.

Test:

```text
RPi4 sends <V,100,100>
Robot moves
RPi4 sends <V,0,0>
Robot stops
RPi4 sends <G,OPEN>
Gripper opens
RPi4 sends <G,CLOSE>
Gripper closes
```

### Phase 2: Telemetry

Goal: RPi4 receives ESP32 sensor data.

Test:

```text
Move hand in front of ultrasonic
RPi4 sees distance change
Rotate robot
RPi4 sees MPU6050 yaw change
```

### Phase 3: Timed Movement Calibration

Goal: RPi4 knows approximate movement speed.

Measure:

```text
forward speed
backward speed
turn speed
```

Save calibration values in a config file.

### Phase 4: QR To Destination

Goal: RPi4 scans QR and finds target coordinate.

Test:

```text
QR says R1C3
RPi4 prints target coordinate
```

### Phase 5: Route Planning

Goal: RPi4 generates waypoints from current position to destination.

Test:

```text
python src/main.py --qr R1C3 --dry-run
```

### Phase 6: Slow Autonomous Demo

Goal: Robot moves using timed dead reckoning.

Rules:

- use slow speed,
- use short waypoint steps,
- use wide aisles,
- stop often,
- avoid narrow shelf gaps.

## Better Future Options

### Option 1: Add Wheel Encoders

This is the most direct improvement.

Benefits:

- RPi knows actual wheel movement,
- distance estimation becomes much better,
- robot can follow routes more accurately.

### Option 2: Add AprilTag/ArUco Markers

Use the RPi camera to detect printed markers on the floor/walls.

Benefits:

- corrects drift,
- no wheel modification needed,
- useful inside a warehouse.

### Option 3: Add A Real Compass

Add QMC5883L or HMC5883L.

Benefits:

- better heading reference than MPU6050 alone,
- helps reduce yaw drift.

Warning: magnetic noise from motors and warehouse metal can affect compass accuracy.

## Final Recommendation

For your current hardware, use:

```text
RPi4 route planning
+ ESP32 command execution
+ timed movement calibration
+ MPU6050 yaw estimate
+ ultrasonic safety
+ WiFi admin panel
```

This will let you build a working demo.

But for reliable real warehouse automation, add at least one correction method:

```text
wheel encoders
or camera markers
or both
```

The best practical upgrade path is:

```text
current timed demo -> add wheel encoders -> add camera marker correction
```
