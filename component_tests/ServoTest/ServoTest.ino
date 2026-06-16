#include <ESP32Servo.h>

const int SERVO_1_PIN = 18;
const int SERVO_2_PIN = 19;

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Servo test starting...");
  Serial.println("Servos should move between 20, 90, and 160 degrees.");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);

  servo1.attach(SERVO_1_PIN, 500, 2500);
  servo2.attach(SERVO_2_PIN, 500, 2500);
}

void loop() {
  Serial.println("20 degrees");
  servo1.write(20);
  servo2.write(20);
  delay(2000);

  Serial.println("90 degrees");
  servo1.write(90);
  servo2.write(90);
  delay(2000);

  Serial.println("160 degrees");
  servo1.write(160);
  servo2.write(160);
  delay(2000);
}
