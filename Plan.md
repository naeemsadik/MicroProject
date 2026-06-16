# Arduino IDE ESP32 Robot Code Plan

## Summary
Write one Arduino `.ino` sketch that can be pasted directly into Arduino IDE and uploaded to the ESP32. The robot will close the gripper, run the timed driving sequence, stop, and release the gripper.

Sequence:

Grip object -> forward 10s -> right 5s -> left 3s -> right 2s -> backward 5s -> stop -> release object.

## Key Code Details
- Use Arduino framework code only, with these libraries:
  - `ESP32Servo.h`
  - `Wire.h`
- Use these pins:
  - Servo gripper 1: `GPIO18`
  - Servo gripper 2: `GPIO19`
  - Ultrasonic trigger: `GPIO23`
  - Ultrasonic echo: `GPIO22`
  - L298N `IN1`: `GPIO25`
  - L298N `IN2`: `GPIO33`
  - L298N `IN3`: `GPIO32`
  - L298N `IN4`: `GPIO27`
  - Compass SDA: `GPIO26`
  - Compass SCL: `GPIO4`
- Use `GPIO27` instead of the originally listed `GPIO34`, because `GPIO34` is input-only and cannot drive the motor driver.
- Add motor helper functions:
  - `moveForward()`
  - `moveBackward()`
  - `turnRight()`
  - `turnLeft()`
  - `stopMotors()`
- Add gripper helper functions:
  - `gripObjectSlowly()`
  - `releaseObjectSlowly()`
- Use configurable constants at the top of the sketch for:
  - Servo open angle
  - Servo closed angle
  - Servo movement delay
  - Obstacle detection threshold
  - Movement durations

## Behavior
- In `setup()`:
  - Start Serial.
  - Set motor pins as outputs.
  - Attach the two servos.
  - Initialize ultrasonic pins.
  - Initialize compass I2C on `GPIO26/GPIO4`.
  - Open gripper once at startup.
  - Measure distance from ultrasonic sensor.
  - If object distance is valid and close enough, close the gripper slowly.
  - If distance is not valid or object is farther than the threshold, print a warning and still continue the sequence.
  - Run:
    - forward for `10 seconds`
    - right for `5 seconds`
    - left for `3 seconds`
    - right for `2 seconds`
    - backward for `5 seconds`
  - Stop motors.
  - Release gripper slowly.
- In `loop()`:
  - Do nothing, so the sequence only runs once after reset.

## Testing
- Test first with the wheels lifted off the ground.
- Confirm each movement direction from Serial output.
- If directions are wrong, adjust the `moveForward()`, `moveBackward()`, `turnRight()`, and `turnLeft()` pin logic.
- Tune servo constants after testing:
  - `SERVO_OPEN_ANGLE`
  - `SERVO_CLOSED_ANGLE`
  - `SERVO_STEP_DELAY_MS`

## Assumptions
- Arduino IDE has ESP32 board support installed.
- Arduino IDE has the `ESP32Servo` library installed.
- L298N ENA/ENB jumpers are installed, so there is no PWM speed control yet.
- Motor-driver pairing is:
  - Left side motors: `IN1/IN2`
  - Right side motors: `IN3/IN4`
- Compass is initialized but not used for navigation in this first timed version.
