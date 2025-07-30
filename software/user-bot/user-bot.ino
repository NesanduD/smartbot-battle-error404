#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// ====== WiFi Config ======
const char* ssid = "espadmin";
const char* password = "12345678";
const char* server = "http://10.10.30.107:5000/score";

// ====== IR Sensors ======
#define IR_FRONT 22
#define IR_BACK 23

// ====== Motor Pins ======
#define LEFT_MOTOR_FORWARD  26
#define LEFT_MOTOR_BACKWARD 25
#define RIGHT_MOTOR_FORWARD 33
#define RIGHT_MOTOR_BACKWARD 32

// ====== Servo for Punch ======
#define SERVO_PIN 18
Servo punchServo;

// ====== Bluetooth Serial ======
BluetoothSerial SerialBT;

bool alertSent = false;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Robot");  // Bluetooth name

  // IR sensor setup
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_BACK, INPUT);

  // Motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Servo setup
  punchServo.attach(SERVO_PIN);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
}

void loop() {
  // === IR Logic ===
  bool frontDetected = (digitalRead(IR_FRONT) == LOW);
  bool backDetected = (digitalRead(IR_BACK) == LOW);

  if (frontDetected && backDetected && !alertSent) {
    Serial.println("Defeat detected!");
    sendScoreUpdate();
    alertSent = true;
  }

  if (!frontDetected || !backDetected) {
    alertSent = false;
  }

  // === Bluetooth Control ===
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.print("Received: ");
    Serial.println(cmd);

    switch (cmd) {
      case 'w': moveForward(); break;
      case 's': moveBackward(); break;
      case 'a': turnLeft(); break;
      case 'd': turnRight(); break;
      case 'p': punch(); break;
      default: stopMotors(); break;
    }
  }

  delay(100); // Debounce
}

// ========== Actions ==========

void sendScoreUpdate() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(server);
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"robot\":\"R1\",\"event\":\"edge_detected\",\"score\":1}";
    int httpResponseCode = http.POST(payload);

    Serial.print("POST Response code: ");
    Serial.println(httpResponseCode);

    http.end();
  } else {
    Serial.println("WiFi not connected, cannot send score.");
  }
}

void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void punch() {
  punchServo.write(0);    // Start position
  delay(100);
  punchServo.write(90);   // Punch forward
  delay(200);
  punchServo.write(0);    // Return
}
