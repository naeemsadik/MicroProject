Here is the **current project pin diagram** based on the implemented ESP32 firmware.

**System Connection**
```text
RPi4 USB port  ->  ESP32 USB cable
RPi4 USB port  ->  USB camera
ESP32          ->  L298N motor driver, servos, ultrasonic sensor, MPU6050
```

**ESP32 Pin Diagram**

| Component | Signal | ESP32 Pin |
|---|---:|---:|
| Gripper servo 1 | PWM | GPIO18 |
| Gripper servo 2 | PWM | GPIO19 |
| Ultrasonic sensor | TRIG | GPIO23 |
| Ultrasonic sensor | ECHO | GPIO22 |
| L298N motor driver | IN1 | GPIO25 |
| L298N motor driver | IN2 | GPIO33 |
| L298N motor driver | IN3 | GPIO32 |
| L298N motor driver | IN4 | GPIO27 |
| MPU6050 | SDA | GPIO26 |
| MPU6050 | SCL | GPIO4 |
| MPU6050 | VCC | 3.3V |
| MPU6050 | GND | GND |

**Unused Pins**

| Pin | Current Status |
|---|---|
| GPIO34 | Unused. Reserved for future left encoder if added. |
| GPIO35 | Unused. Reserved for future right encoder if added. |

**L298N Motor Wiring**
```text
ESP32 GPIO25 -> L298N IN1
ESP32 GPIO33 -> L298N IN2
ESP32 GPIO32 -> L298N IN3
ESP32 GPIO27 -> L298N IN4

Left side motors  -> L298N OUT1 / OUT2
Right side motors -> L298N OUT3 / OUT4
```

Current code assumes **ENA/ENB jumpers are installed** on the L298N. That means speed control is limited. For real PWM speed control later, connect ENA and ENB to two ESP32 PWM pins and update:

```cpp
const int MOTOR_LEFT_ENABLE_PIN = -1;
const int MOTOR_RIGHT_ENABLE_PIN = -1;
```

**Important Power Notes**
- Use a separate **5V power supply** for the servos.
- Use motor battery/supply for the L298N motor power.
- Connect **all grounds together**:
```text
ESP32 GND
RPi4 GND through USB
Servo power GND
L298N GND
MPU6050 GND
Ultrasonic GND
```

**Very Important**
- There are **no encoders** in the current hardware setup.
- Without encoders or another position tracking method, the RPi4 can generate a route but cannot accurately know where the robot is while it moves.
- The MPU6050 is **not a compass**. It is an accelerometer/gyro and its yaw estimate will drift over time.
- If your ultrasonic sensor is HC-SR04 powered at 5V, its ECHO pin outputs 5V. ESP32 pins are 3.3V only, so use a voltage divider on ECHO before GPIO22.
