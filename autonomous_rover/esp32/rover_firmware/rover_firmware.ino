#include <ESP32Servo.h>
#include <Wire.h>

// -------------------- Pin configuration --------------------
const int SERVO_1_PIN = 18;
const int SERVO_2_PIN = 19;

const int ULTRASONIC_TRIG_PIN = 23;
const int ULTRASONIC_ECHO_PIN = 22;

const int MOTOR_IN1_PIN = 25;  // Left side motor direction A
const int MOTOR_IN2_PIN = 33;  // Left side motor direction B
const int MOTOR_IN3_PIN = 32;  // Right side motor direction A
const int MOTOR_IN4_PIN = 27;  // Right side motor direction B

// Set these to your L298N ENA/ENB pins when connected.
// Leave as -1 if the enable jumpers are installed.
const int MOTOR_LEFT_ENABLE_PIN = -1;
const int MOTOR_RIGHT_ENABLE_PIN = -1;

// Set encoder pins when wheel encoders are installed.
// Leave as -1 when you do not have encoders.
const int LEFT_ENCODER_PIN = -1;
const int RIGHT_ENCODER_PIN = -1;

const int MPU_SDA_PIN = 26;
const int MPU_SCL_PIN = 4;
const uint8_t MPU6050_ADDR = 0x68;

// -------------------- Gripper tuning --------------------
const int SERVO_1_OPEN_ANGLE = 25;
const int SERVO_1_CLOSED_ANGLE = 95;
const int SERVO_2_OPEN_ANGLE = 155;
const int SERVO_2_CLOSED_ANGLE = 85;
const int SERVO_STEP_DELAY_MS = 20;

// -------------------- Safety and telemetry --------------------
const float STOP_DISTANCE_CM = 12.0;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;
const unsigned long TELEMETRY_INTERVAL_MS = 100;

Servo servo1;
Servo servo2;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

String inputLine = "";
volatile int currentLeftSpeed = 0;
volatile int currentRightSpeed = 0;
int servo1CurrentAngle = SERVO_1_OPEN_ANGLE;
int servo2CurrentAngle = SERVO_2_OPEN_ANGLE;

float gyroZBias = 0.0;
float yawDeg = 0.0;
unsigned long lastGyroMicros = 0;
unsigned long lastTelemetryMs = 0;

void IRAM_ATTR onLeftEncoderTick() {
  if (currentLeftSpeed >= 0) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void IRAM_ATTR onRightEncoderTick() {
  if (currentRightSpeed >= 0) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);
  setupMotorEnablePins();
  stopMotors();

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  if (LEFT_ENCODER_PIN >= 0) {
    pinMode(LEFT_ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), onLeftEncoderTick, RISING);
  }
  if (RIGHT_ENCODER_PIN >= 0) {
    pinMode(RIGHT_ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), onRightEncoderTick, RISING);
  }

  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  initMPU6050();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO_1_PIN, 500, 2500);
  servo2.attach(SERVO_2_PIN, 500, 2500);
  writeServosNow();

  Serial.println("<READY>");
}

void loop() {
  readSerialCommands();
  updateYaw();

  unsigned long nowMs = millis();
  if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = nowMs;
    float distanceCm = readDistanceCm();
    if (distanceCm > 0 && distanceCm < STOP_DISTANCE_CM) {
      stopMotors();
    }
    sendTelemetry(distanceCm);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.startsWith("<") || !cmd.endsWith(">")) {
    return;
  }

  if (cmd.startsWith("<V,")) {
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    int endBracket = cmd.indexOf('>');
    if (firstComma < 0 || secondComma < 0 || endBracket < 0) {
      return;
    }

    int leftSpeed = cmd.substring(firstComma + 1, secondComma).toInt();
    int rightSpeed = cmd.substring(secondComma + 1, endBracket).toInt();
    setMotorSpeeds(leftSpeed, rightSpeed);
  } else if (cmd == "<G,OPEN>") {
    releaseObjectSlowly();
  } else if (cmd == "<G,CLOSE>") {
    gripObjectSlowly();
  } else if (cmd == "<PING>") {
    Serial.println("<PONG>");
  } else if (cmd == "<RESET_TICKS>") {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
    yawDeg = 0.0;
  }
}

void setupMotorEnablePins() {
  if (MOTOR_LEFT_ENABLE_PIN >= 0) {
    pinMode(MOTOR_LEFT_ENABLE_PIN, OUTPUT);
    analogWrite(MOTOR_LEFT_ENABLE_PIN, 0);
  }

  if (MOTOR_RIGHT_ENABLE_PIN >= 0) {
    pinMode(MOTOR_RIGHT_ENABLE_PIN, OUTPUT);
    analogWrite(MOTOR_RIGHT_ENABLE_PIN, 0);
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  currentLeftSpeed = constrain(leftSpeed, -255, 255);
  currentRightSpeed = constrain(rightSpeed, -255, 255);
  driveLeftMotor(currentLeftSpeed);
  driveRightMotor(currentRightSpeed);
}

void driveLeftMotor(int speed) {
  int magnitude = abs(speed);
  if (speed > 0) {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  } else if (speed < 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }

  if (MOTOR_LEFT_ENABLE_PIN >= 0) {
    analogWrite(MOTOR_LEFT_ENABLE_PIN, magnitude);
  }
}

void driveRightMotor(int speed) {
  int magnitude = abs(speed);
  if (speed > 0) {
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, HIGH);
  } else if (speed < 0) {
    digitalWrite(MOTOR_IN3_PIN, HIGH);
    digitalWrite(MOTOR_IN4_PIN, LOW);
  } else {
    digitalWrite(MOTOR_IN3_PIN, LOW);
    digitalWrite(MOTOR_IN4_PIN, LOW);
  }

  if (MOTOR_RIGHT_ENABLE_PIN >= 0) {
    analogWrite(MOTOR_RIGHT_ENABLE_PIN, magnitude);
  }
}

void stopMotors() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  driveLeftMotor(0);
  driveRightMotor(0);
}

void gripObjectSlowly() {
  moveServosSlowly(SERVO_1_CLOSED_ANGLE, SERVO_2_CLOSED_ANGLE);
}

void releaseObjectSlowly() {
  moveServosSlowly(SERVO_1_OPEN_ANGLE, SERVO_2_OPEN_ANGLE);
}

void writeServosNow() {
  servo1.write(servo1CurrentAngle);
  servo2.write(servo2CurrentAngle);
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

    writeServosNow();
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

void initMPU6050() {
  writeMPURegister(0x6B, 0x00);
  delay(100);
  calibrateGyroZ();
  lastGyroMicros = micros();
}

void calibrateGyroZ() {
  const int samples = 500;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readMPUWord(0x47);
    delay(3);
  }
  gyroZBias = sum / (float)samples;
}

void updateYaw() {
  unsigned long nowMicros = micros();
  if (lastGyroMicros == 0) {
    lastGyroMicros = nowMicros;
    return;
  }

  float dt = (nowMicros - lastGyroMicros) / 1000000.0;
  lastGyroMicros = nowMicros;

  int16_t rawGyroZ = readMPUWord(0x47);
  float gyroZDegPerSec = (rawGyroZ - gyroZBias) / 131.0;
  yawDeg += gyroZDegPerSec * dt;
}

int16_t readMPUWord(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)2);

  if (Wire.available() < 2) {
    return 0;
  }

  int16_t highByte = Wire.read();
  int16_t lowByte = Wire.read();
  return (highByte << 8) | lowByte;
}

void writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void sendTelemetry(float distanceCm) {
  long leftSnapshot;
  long rightSnapshot;
  noInterrupts();
  leftSnapshot = leftTicks;
  rightSnapshot = rightTicks;
  interrupts();

  Serial.print("<T,");
  Serial.print(distanceCm, 1);
  Serial.print(",");
  Serial.print(leftSnapshot);
  Serial.print(",");
  Serial.print(rightSnapshot);
  Serial.print(",");
  Serial.print(yawDeg, 2);
  Serial.println(">");
}
