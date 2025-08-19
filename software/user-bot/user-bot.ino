#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// ==== WiFi Credentials ====
const char* ssid = "espadmin";
const char* password = "12345678";
const char* serverURL = "http://192.168.137.1:5000/score";

// ==== IR Sensor Pins ====
#define IR_FRONT 34
#define IR_BACK  35

// ==== IR Thresholds ====
#define FRONT_THRESHOLD 1000
#define BACK_THRESHOLD  1000

// ==== Motor Pins ====
#define LEFT_MOTOR_FORWARD  25
#define LEFT_MOTOR_BACKWARD 26
#define RIGHT_MOTOR_FORWARD 32
#define RIGHT_MOTOR_BACKWARD 33

// ==== Servo Pin ====
#define SERVO_PIN 18

// ==== Flags ====
bool defeated = false;

// ==== Bluetooth ====
BluetoothSerial SerialBT;

// ==== Servo ====
Servo punchServo;

// ==== Function to stop motors ====
void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// ==== Function to send score ====
void sendScore() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");
    int responseCode = http.POST("{\"robot\":\"R2\",\"event\":\"edge_detected\",\"score\":1}");
    Serial.print("📤 Score sent. Response code: ");
    Serial.println(responseCode);
    http.end();
  } else {
    Serial.println("❌ WiFi not connected. Score not sent.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("🔌 Starting setup...");

  // Start Bluetooth
  if (SerialBT.begin("ESP32")) {
    Serial.println("✅ Bluetooth started as 'ESP32'");
  } else {
    Serial.println("❌ Bluetooth failed to start!");
  }

  // IR sensors
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_BACK, INPUT);

  // Motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Setup Servo
  punchServo.attach(SERVO_PIN);
  punchServo.write(0); // Initial position

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("📡 Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
  } else {
    Serial.println("\n❌ WiFi Failed!");
  }
}

void loop() {
  int frontVal = analogRead(IR_FRONT);
  int backVal = analogRead(IR_BACK);

  Serial.print("IR_FRONT: ");
  Serial.print(frontVal);
  Serial.print("   |   IR_BACK: ");
  Serial.println(backVal);

  // Check defeat condition
  if (!defeated && frontVal > FRONT_THRESHOLD && backVal > BACK_THRESHOLD) {
    Serial.println("💀 Defeated! Stopping, sending score, and pausing for 10 seconds...");
    stopMotors();
    sendScore();
    defeated = true;

    // Pause everything for 10 seconds
    unsigned long pauseStart = millis();
    while (millis() - pauseStart < 10000) {
      delay(100);
    }

    Serial.println("🔄 Restarting robot logic after 10 seconds...");
    defeated = false;
    return;
  }

  if (defeated) {
    return; // while paused, skip everything
  }

  // Optional: Check if Bluetooth is connected
  if (!SerialBT.hasClient()) {
    Serial.println("⚠️ No Bluetooth client connected.");
  }

  // Receive Bluetooth command
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.print("📥 Received command: ");
    Serial.println(cmd);

    if (cmd == 'f') {
      digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    } else if (cmd == 'b') {
      digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
      digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    } else if (cmd == 'l') {
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    } else if (cmd == 'r') {
      digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    } else if (cmd == 's') {
      stopMotors();
    } else if (cmd == 'p') {
      Serial.println("🤜 Punching!");
      punchServo.write(180);
      delay(500);
      punchServo.write(0);
    } else {
      Serial.println("❓ Unknown command.");
    }
  }

  delay(50); // shorter delay for faster response
}