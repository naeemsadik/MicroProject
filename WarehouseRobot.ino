#include <ESP32Servo.h>
#include <Wire.h>

// -------------------- Pin configuration --------------------
const int SERVO_1_PIN = 18;
const int SERVO_2_PIN = 19;

const int ULTRASONIC_TRIG_PIN = 12;
const int ULTRASONIC_ECHO_PIN = 13;

const int MOTOR_IN1_PIN = 25;  // Left side motor direction A
const int MOTOR_IN2_PIN = 33;  // Left side motor direction B
const int MOTOR_IN3_PIN = 32;  // Right side motor direction A
const int MOTOR_IN4_PIN = 27;  // Right side motor direction B

// Set these to the ENA/ENB pins if your motor driver has enable pins connected.
// Leave as -1 if the driver enable jumpers are installed or your driver has no enable pins.
const int MOTOR_LEFT_ENABLE_PIN = -1;
const int MOTOR_RIGHT_ENABLE_PIN = -1;

const int COMPASS_SDA_PIN = 21;
const int COMPASS_SCL_PIN = 22;

// -------------------- Gripper tuning --------------------
// Tune these angles for your physical gripper.
// If a servo moves the wrong way, swap its open and closed values.
const int SERVO_1_OPEN_ANGLE = 25;
const int SERVO_1_CLOSED_ANGLE = 95;

const int SERVO_2_OPEN_ANGLE = 155;
const int SERVO_2_CLOSED_ANGLE = 85;

const int SERVO_STEP_DELAY_MS = 25;
const int SERVO_SETTLE_DELAY_MS = 500;

// -------------------- Ultrasonic tuning --------------------
const float OBJECT_DISTANCE_THRESHOLD_CM = 20.0;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;  // About 5 meters max.

// -------------------- Movement durations --------------------
const unsigned long FORWARD_DURATION_MS = 10000;
const unsigned long RIGHT_1_DURATION_MS = 5000;
const unsigned long LEFT_DURATION_MS = 3000;
const unsigned long RIGHT_2_DURATION_MS = 2000;
const unsigned long BACKWARD_DURATION_MS = 5000;

Servo servo1;
Servo servo2;

int servo1CurrentAngle = SERVO_1_OPEN_ANGLE;
int servo2CurrentAngle = SERVO_2_OPEN_ANGLE;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 warehouse robot starting...");

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);
  if (MOTOR_LEFT_ENABLE_PIN >= 0) {
    pinMode(MOTOR_LEFT_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_LEFT_ENABLE_PIN, HIGH);
  }
  if (MOTOR_RIGHT_ENABLE_PIN >= 0) {
    pinMode(MOTOR_RIGHT_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_RIGHT_ENABLE_PIN, HIGH);
  }
  stopMotors();

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  Wire.begin(COMPASS_SDA_PIN, COMPASS_SCL_PIN);
  Serial.println("MPU6050 I2C initialized on SDA GPIO21 and SCL GPIO22.");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO_1_PIN, 500, 2500);
  servo2.attach(SERVO_2_PIN, 500, 2500);
  writeServosNow();
  delay(SERVO_SETTLE_DELAY_MS);

  Serial.println("Opening gripper...");
  releaseObjectSlowly();
  delay(SERVO_SETTLE_DELAY_MS);

  float distanceCm = readDistanceCm();
  Serial.print("Ultrasonic distance: ");
  if (distanceCm < 0) {
    Serial.println("invalid reading");
    Serial.println("Warning: no valid object distance found. Continuing anyway.");
  } else {
    Serial.print(distanceCm);
    Serial.println(" cm");

    if (distanceCm > OBJECT_DISTANCE_THRESHOLD_CM) {
      Serial.println("Warning: object is farther than threshold. Continuing anyway.");
    }
  }

  Serial.println("Closing gripper...");
  gripObjectSlowly();
  delay(SERVO_SETTLE_DELAY_MS);

  Serial.println("Moving forward for 10 seconds...");
  moveForward();
  delay(FORWARD_DURATION_MS);

  Serial.println("Turning right for 5 seconds...");
  turnRight();
  delay(RIGHT_1_DURATION_MS);

  Serial.println("Turning left for 3 seconds...");
  turnLeft();
  delay(LEFT_DURATION_MS);

  Serial.println("Turning right for 2 seconds...");
  turnRight();
  delay(RIGHT_2_DURATION_MS);

  Serial.println("Moving backward for 5 seconds...");
  moveBackward();
  delay(BACKWARD_DURATION_MS);

  Serial.println("Stopping motors...");
  stopMotors();
  delay(500);

  Serial.println("Releasing gripper...");
  releaseObjectSlowly();

  Serial.println("Sequence complete. Robot is idle.");
}

void loop() {
  // Run the sequence only once after reset.
}

void moveForward() {
  setLeftMotorsForward();
  setRightMotorsForward();
}

void moveBackward() {
  setLeftMotorsBackward();
  setRightMotorsBackward();
}

void turnRight() {
  setLeftMotorsForward();
  setRightMotorsBackward();
}

void turnLeft() {
  setLeftMotorsBackward();
  setRightMotorsForward();
}

void stopMotors() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
}

void writeServosNow() {
  servo1.write(servo1CurrentAngle);
  servo2.write(servo2CurrentAngle);
}

void setLeftMotorsForward() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}

void setLeftMotorsBackward() {
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

void setRightMotorsForward() {
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, HIGH);
}

void setRightMotorsBackward() {
  digitalWrite(MOTOR_IN3_PIN, HIGH);
  digitalWrite(MOTOR_IN4_PIN, LOW);
}

void gripObjectSlowly() {
  moveServosSlowly(SERVO_1_CLOSED_ANGLE, SERVO_2_CLOSED_ANGLE);
}

void releaseObjectSlowly() {
  moveServosSlowly(SERVO_1_OPEN_ANGLE, SERVO_2_OPEN_ANGLE);
}

void moveServosSlowly(int servo1TargetAngle, int servo2TargetAngle) {
  servo1TargetAngle = constrain(servo1TargetAngle, 0, 180);
  servo2TargetAngle = constrain(servo2TargetAngle, 0, 180);

  while (servo1CurrentAngle != servo1TargetAngle || servo2CurrentAngle != servo2TargetAngle) {
    if (servo1CurrentAngle < servo1TargetAngle) {
      servo1CurrentAngle++;
    } else if (servo1CurrentAngle > servo1TargetAngle) {
      servo1CurrentAngle--;
    }

    if (servo2CurrentAngle < servo2TargetAngle) {
      servo2CurrentAngle++;
    } else if (servo2CurrentAngle > servo2TargetAngle) {
      servo2CurrentAngle--;
    }

    servo1.write(servo1CurrentAngle);
    servo2.write(servo2CurrentAngle);
    delay(SERVO_STEP_DELAY_MS);
  }
}

float readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) {
    return -1.0;
  }

  return durationUs * 0.0343 / 2.0;
}
