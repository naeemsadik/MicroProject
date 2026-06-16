const int ULTRASONIC_TRIG_PIN = 23;
const int ULTRASONIC_ECHO_PIN = 22;

const unsigned long ULTRASONIC_TIMEOUT_US = 30000;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Ultrasonic test starting...");
  Serial.println("Move your hand in front of the sensor and watch the distance.");

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

void loop() {
  float distanceCm = readDistanceCm();

  if (distanceCm < 0) {
    Serial.println("Distance: invalid reading");
  } else {
    Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }

  delay(500);
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
