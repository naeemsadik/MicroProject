const int MOTOR_IN1_PIN = 25;
const int MOTOR_IN2_PIN = 33;
const int MOTOR_IN3_PIN = 32;
const int MOTOR_IN4_PIN = 27;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("L298N motor test starting...");
  Serial.println("Make sure ENA and ENB are jumpered HIGH on the L298N.");

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);

  stopMotors();
}

void loop() {
  Serial.println("Left motor forward");
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
  delay(3000);

  Serial.println("Stop");
  stopMotors();
  delay(1500);

  Serial.println("Left motor backward");
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
  delay(3000);

  Serial.println("Stop");
  stopMotors();
  delay(1500);

  Serial.println("Right motor forward");
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, HIGH);
  delay(3000);

  Serial.println("Stop");
  stopMotors();
  delay(1500);

  Serial.println("Right motor backward");
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, HIGH);
  digitalWrite(MOTOR_IN4_PIN, LOW);
  delay(3000);

  Serial.println("Stop");
  stopMotors();
  delay(3000);
}

void stopMotors() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
}
