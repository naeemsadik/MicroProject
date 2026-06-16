/*
 * Warehouse Robot - Standalone ESP32 Demo
 *
 * This is the legacy stand-alone demo sketch. It runs a fixed timed
 * sequence on boot, services a small WiFi admin panel, and otherwise
 * listens for USB serial commands.
 *
 * For RPi4-driven navigation, upload the sketch in:
 *   autonomous_rover/esp32/rover_firmware/rover_firmware.ino
 *
 * Pin map (matches uppdatedPinDiagram.md):
 *   Servo 1        -> GPIO18
 *   Servo 2        -> GPIO19
 *   Ultrasonic TRIG -> GPIO12
 *   Ultrasonic ECHO -> GPIO13
 *   L298N IN1 (L)   -> GPIO25
 *   L298N IN2 (L)   -> GPIO33
 *   L298N IN3 (R)   -> GPIO32
 *   L298N IN4 (R)   -> GPIO27
 *   MPU6050 SDA     -> GPIO21
 *   MPU6050 SCL     -> GPIO22
 *
 * NOTE: This file is kept for the standalone timed demo only.
 *       When controlled by the RPi4, use rover_firmware.ino instead.
 */

#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
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
// Leave as -1 if the driver enable jumpers are installed.
const int MOTOR_LEFT_ENABLE_PIN = -1;
const int MOTOR_RIGHT_ENABLE_PIN = -1;

const int COMPASS_SDA_PIN = 21;
const int COMPASS_SCL_PIN = 22;

// -------------------- Gripper tuning --------------------
const int SERVO_1_OPEN_ANGLE = 25;
const int SERVO_1_CLOSED_ANGLE = 95;
const int SERVO_2_OPEN_ANGLE = 155;
const int SERVO_2_CLOSED_ANGLE = 85;
const int SERVO_STEP_DELAY_MS = 25;
const int SERVO_SETTLE_DELAY_MS = 500;

// -------------------- Ultrasonic tuning --------------------
const float OBJECT_DISTANCE_THRESHOLD_CM = 20.0;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;

// -------------------- Movement durations --------------------
const unsigned long FORWARD_DURATION_MS = 10000;
const unsigned long RIGHT_1_DURATION_MS = 5000;
const unsigned long LEFT_DURATION_MS = 3000;
const unsigned long RIGHT_2_DURATION_MS = 2000;
const unsigned long BACKWARD_DURATION_MS = 5000;

// -------------------- WiFi admin panel --------------------
const char* WIFI_SSID = "No Internet";
const char* WIFI_PASSWORD = "Pass1theke9";
const char* ADMIN_HOSTNAME = "warehouse-robot";

WebServer server(80);

String logBuffer = "";
String robotState = "Booting";
float latestDistanceCm = -1.0;
unsigned long lastDistanceUpdateMs = 0;
const unsigned long DISTANCE_UPDATE_INTERVAL_MS = 1000;
const int MAX_LOG_CHARS = 7000;

Servo servo1;
Servo servo2;

int servo1CurrentAngle = SERVO_1_OPEN_ANGLE;
int servo2CurrentAngle = SERVO_2_OPEN_ANGLE;

void setup() {
  Serial.begin(115200);
  delay(1000);

  logLine("");
  logLine("ESP32 warehouse robot starting...");
  setupAdminPanel();

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
  logLine("MPU6050 I2C initialized on SDA GPIO21 and SCL GPIO22.");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO_1_PIN, 500, 2500);
  servo2.attach(SERVO_2_PIN, 500, 2500);
  writeServosNow();
  robotDelay(SERVO_SETTLE_DELAY_MS);

  logLine("Opening gripper...");
  releaseObjectSlowly();
  robotDelay(SERVO_SETTLE_DELAY_MS);

  float distanceCm = readDistanceCm();
  if (distanceCm < 0) {
    latestDistanceCm = -1.0;
    logLine("Ultrasonic distance: invalid reading");
    logLine("Warning: no valid object distance found. Continuing anyway.");
  } else {
    latestDistanceCm = distanceCm;
    logLine("Ultrasonic distance: " + String(distanceCm, 1) + " cm");

    if (distanceCm > OBJECT_DISTANCE_THRESHOLD_CM) {
      logLine("Warning: object is farther than threshold. Continuing anyway.");
    }
  }

  logLine("Closing gripper...");
  gripObjectSlowly();
  robotDelay(SERVO_SETTLE_DELAY_MS);

  logLine("Moving forward for 10 seconds...");
  moveForward();
  robotDelay(FORWARD_DURATION_MS);

  logLine("Turning right for 5 seconds...");
  turnRight();
  robotDelay(RIGHT_1_DURATION_MS);

  logLine("Turning left for 3 seconds...");
  turnLeft();
  robotDelay(LEFT_DURATION_MS);

  logLine("Turning right for 2 seconds...");
  turnRight();
  robotDelay(RIGHT_2_DURATION_MS);

  logLine("Moving backward for 5 seconds...");
  moveBackward();
  robotDelay(BACKWARD_DURATION_MS);

  logLine("Stopping motors...");
  stopMotors();
  robotDelay(500);

  logLine("Releasing gripper...");
  releaseObjectSlowly();

  robotState = "Idle";
  logLine("Sequence complete. Robot is idle.");
}

void loop() {
  serviceAdminPanel();
  delay(10);
}

void moveForward() {
  robotState = "Moving forward";
  setLeftMotorsForward();
  setRightMotorsForward();
}

void moveBackward() {
  robotState = "Moving backward";
  setLeftMotorsBackward();
  setRightMotorsBackward();
}

void turnRight() {
  robotState = "Turning right";
  setLeftMotorsForward();
  setRightMotorsBackward();
}

void turnLeft() {
  robotState = "Turning left";
  setLeftMotorsBackward();
  setRightMotorsForward();
}

void stopMotors() {
  robotState = "Stopped";
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
    robotDelay(SERVO_STEP_DELAY_MS);
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

void setupAdminPanel() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  logLine("Connecting to WiFi: " + String(WIFI_SSID));

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    logLine("WiFi connected.");
    logLine("Admin panel: http://" + WiFi.localIP().toString());
    if (MDNS.begin(ADMIN_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      logLine("Admin mDNS: http://" + String(ADMIN_HOSTNAME) + ".local");
    } else {
      logLine("mDNS setup failed. Use the IP address instead.");
    }
  } else {
    logLine("WiFi connection failed. Admin panel unavailable until restart.");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/clear", handleClear);
  server.begin();
  logLine("Admin web server started.");
}

void serviceAdminPanel() {
  server.handleClient();

  if (millis() - lastDistanceUpdateMs >= DISTANCE_UPDATE_INTERVAL_MS) {
    lastDistanceUpdateMs = millis();
    latestDistanceCm = readDistanceCm();
  }
}

void robotDelay(unsigned long durationMs) {
  unsigned long startMs = millis();
  while (millis() - startMs < durationMs) {
    serviceAdminPanel();
    delay(10);
  }
}

void logLine(const String& message) {
  Serial.println(message);
  String entry = "[" + String(millis() / 1000.0, 1) + "s] " + message + "\n";
  logBuffer += entry;

  if (logBuffer.length() > MAX_LOG_CHARS) {
    logBuffer.remove(0, logBuffer.length() - MAX_LOG_CHARS);
  }
}

void handleRoot() {
  String html = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Warehouse Robot Admin</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #111827; color: #e5e7eb; }
    header { padding: 16px; background: #0f172a; border-bottom: 1px solid #334155; }
    h1 { margin: 0; font-size: 22px; }
    main { padding: 16px; max-width: 900px; margin: 0 auto; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; margin-bottom: 16px; }
    .card { background: #1f2937; border: 1px solid #374151; border-radius: 8px; padding: 14px; }
    .label { color: #9ca3af; font-size: 13px; margin-bottom: 6px; }
    .value { font-size: 22px; font-weight: 700; }
    pre { background: #020617; color: #d1d5db; padding: 14px; border-radius: 8px; overflow: auto; min-height: 320px; white-space: pre-wrap; }
    button { background: #2563eb; color: white; border: 0; border-radius: 6px; padding: 10px 12px; font-weight: 700; }
  </style>
</head>
<body>
  <header><h1>Warehouse Robot Admin</h1></header>
  <main>
    <div class="grid">
      <div class="card"><div class="label">State</div><div id="state" class="value">-</div></div>
      <div class="card"><div class="label">Distance</div><div id="distance" class="value">-</div></div>
      <div class="card"><div class="label">Uptime</div><div id="uptime" class="value">-</div></div>
      <div class="card"><div class="label">WiFi</div><div id="wifi" class="value">-</div></div>
    </div>
    <button onclick="clearLog()">Clear Log</button>
    <pre id="log"></pre>
  </main>
  <script>
    async function refresh() {
      const res = await fetch('/data');
      const data = await res.json();
      document.getElementById('state').textContent = data.state;
      document.getElementById('distance').textContent = data.distance_cm < 0 ? 'Invalid' : data.distance_cm.toFixed(1) + ' cm';
      document.getElementById('uptime').textContent = data.uptime_s.toFixed(1) + ' s';
      document.getElementById('wifi').textContent = data.ip;
      document.getElementById('log').textContent = data.log;
    }
    async function clearLog() {
      await fetch('/clear');
      refresh();
    }
    setInterval(refresh, 1000);
    refresh();
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"state\":\"" + jsonEscape(robotState) + "\",";
  json += "\"distance_cm\":" + String(latestDistanceCm, 1) + ",";
  json += "\"uptime_s\":" + String(millis() / 1000.0, 1) + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"log\":\"" + jsonEscape(logBuffer) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleClear() {
  logBuffer = "";
  logLine("Admin log cleared.");
  server.send(200, "text/plain", "OK");
}

String jsonEscape(const String& input) {
  String output = "";
  output.reserve(input.length() + 16);

  for (unsigned int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (c == '"') {
      output += "\\\"";
    } else if (c == '\\') {
      output += "\\\\";
    } else if (c == '\n') {
      output += "\\n";
    } else if (c == '\r') {
      output += "\\r";
    } else if (c == '\t') {
      output += "\\t";
    } else {
      output += c;
    }
  }

  return output;
}
