#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>  // ESP32-specific servo library

// ====== WiFi Config ======
const char* ssid = "espadmin";
const char* password = "12345678";
const char* server = "http://192.168.209.168:5000/score"; // Ensure this IP is correct for your server

// ====== IR Sensors (Analog) ======
#define IR_FRONT 34
#define IR_BACK 35
const int FRONT_THRESHOLD = 10;  // Adjust these thresholds based on your sensor readings
const int BACK_THRESHOLD = 100;

// ====== Motor Pins ======
#define LEFT_MOTOR_FORWARD  25
#define LEFT_MOTOR_BACKWARD 26
#define RIGHT_MOTOR_FORWARD 32
#define RIGHT_MOTOR_BACKWARD 33

// ====== Servo for Punch ======
#define SERVO_PIN 18
Servo punchServo;

// defeated flag
bool defeated = false;

// ====== Bluetooth Serial ======
BluetoothSerial SerialBT;

char lastCommand = ' ';  // To track current movement

// ====== Defeat Detection Counter ======
int defeatCounter = 0;
const int DEFEAT_TRIGGER_COUNT = 3; // Number of consecutive readings above threshold to trigger defeat

// ==== Function declarations ====
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void punch();
void sendScoreUpdate();

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Robot"); // Bluetooth device name

  // Set up IR sensor pins
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_BACK, INPUT);

  // Set up motor control pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Attach servo to its pin
  punchServo.attach(SERVO_PIN);

  // Initialize WiFi connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
    WiFi.begin(ssid, password);  // Re-attempt for robustness
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to WiFi!");
    Serial.print("📶 IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Failed to connect to WiFi!");
  }
}

void loop() {
  // Read current values from IR sensors
  int frontValue = analogRead(IR_FRONT);
  int backValue = analogRead(IR_BACK);
  Serial.print("Front IR: "); Serial.print(frontValue);
  Serial.print(" | Back IR: "); Serial.println(backValue);

  if (!defeated) {
    // IR defeat detection logic: Both IR values are MORE than their thresholds
    if (frontValue > FRONT_THRESHOLD && backValue > BACK_THRESHOLD) {
      defeatCounter++;
    } else {
      defeatCounter = 0; // Reset counter if conditions are not met
    }

    // If the defeat condition has been met for enough consecutive readings
    if (defeatCounter >= DEFEAT_TRIGGER_COUNT) {
      Serial.println("💥 Defeat detected via IR thresholds (both sensors above threshold)!");
      stopMotors(); // Stop the robot immediately
      sendScoreUpdate(); // Send score to the server
      defeated = true;  // Lock the system: robot is now defeated permanently
    }

    // Bluetooth control (only active if robot is not defeated)
    if (SerialBT.available()) {
      char cmd = SerialBT.read();
      Serial.print("Received: ");
      Serial.println(cmd);

      switch (cmd) {
        case 'f': // Move Forward
          moveForward();
          lastCommand = 'f';
          break;
        case 'b': // Move Backward
          moveBackward();
          lastCommand = 'b';
          break;
        case 'l': // Turn Left
          turnLeft();
          lastCommand = 'l';
          break;
        case 'r': // Turn Right
          turnRight();
          lastCommand = 'r';
          break;
        case 's': // Stop Motors
          stopMotors();
          lastCommand = 's';
          break;
        case 'p': // Punch action
          punch();
          break;
        default: // If an unknown command, stop
          stopMotors();
          lastCommand = ' '; // Clear last command if invalid
          break;
      }
    } else {
      // Resume last movement if no new Bluetooth command is received
      if (lastCommand != 's' && lastCommand != ' ') { 
        switch (lastCommand) {
          case 'f': moveForward(); break;
          case 'b': moveBackward(); break;
          case 'l': turnLeft(); break;
          case 'r': turnRight(); break;
        }
      } else {
        stopMotors(); // If last command was 's' or ' ', keep motors stopped.
      }
    }
  } else {
    // If the robot has been defeated, ensure motors remain stopped
    stopMotors();
  }

  delay(100); // Small delay to debounce readings and commands
}

// Movement functions implementations
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
  punchServo.write(180);  // Move servo to punch position
  delay(500);
  punchServo.write(0);    // Return servo to rest position
  delay(500);
}

void sendScoreUpdate() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(server);
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"robot\":\"R1\",\"event\":\"edge_detected\",\"score\":1}";
    int httpResponseCode = http.POST(payload);

    Serial.print("📤 POST Response code: ");
    Serial.println(httpResponseCode);

    http.end();
  } else {
    Serial.println("❌ WiFi not connected, cannot send score.");
  }
}
