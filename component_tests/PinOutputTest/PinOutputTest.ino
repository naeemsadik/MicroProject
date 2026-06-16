const int TEST_PINS[] = {18, 19, 23, 25, 33, 32, 27};
const int TEST_PIN_COUNT = sizeof(TEST_PINS) / sizeof(TEST_PINS[0]);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32 pin output test starting...");
  Serial.println("Each listed pin will turn HIGH for 2 seconds, then LOW.");
  Serial.println("Use a multimeter or LED with resistor from the pin to GND.");

  for (int i = 0; i < TEST_PIN_COUNT; i++) {
    pinMode(TEST_PINS[i], OUTPUT);
    digitalWrite(TEST_PINS[i], LOW);
  }
}

void loop() {
  for (int i = 0; i < TEST_PIN_COUNT; i++) {
    int pin = TEST_PINS[i];

    Serial.print("GPIO ");
    Serial.print(pin);
    Serial.println(" HIGH");
    digitalWrite(pin, HIGH);
    delay(2000);

    Serial.print("GPIO ");
    Serial.print(pin);
    Serial.println(" LOW");
    digitalWrite(pin, LOW);
    delay(1000);
  }
}
