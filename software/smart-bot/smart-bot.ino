#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ==== Motor Pins ====
#define LEFT_MOTOR_FORWARD  23
#define LEFT_MOTOR_BACKWARD 22
#define RIGHT_MOTOR_FORWARD 21
#define RIGHT_MOTOR_BACKWARD 19

// ==== Servo ====
#define SERVO_PIN 18

// ==== Color Sensor Pins ====
#define SDA_PIN 25
#define SCL_PIN 26

// ==== Ultrasonic Sensor Pins ====
#define TRIG_TOP_LEFT       13
#define ECHO_TOP_LEFT       32
#define TRIG_TOP_RIGHT      5 
#define ECHO_TOP_RIGHT      15
#define TRIG_BOTTOM_LEFT    4
#define ECHO_BOTTOM_LEFT    2 
#define TRIG_BOTTOM_RIGHT   14
#define ECHO_BOTTOM_RIGHT   27

// ==== IR Sensor Pins ====
#define IR_FRONT 35
#define IR_BACK  34

// ==== Constants ====
#define ENEMY_RANGE_CM 50
#define PUNCH_DELAY 500
#define IR_ANALOG_THRESHOLD 1000
#define IR_CONFIRM_TIME 10
#define IR_REACT_TIME 1500

// ==== WiFi Settings ====
const char* ssid = "espadmin";
const char* password = "12345678";
const char* server = "http://192.168.137.1:5000/score";
bool alertSent = false;

// ==== State ====
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo hammer;

enum BotState {
  STATE_START,
  STATE_SEARCH,
  STATE_CHASE,
  STATE_ATTACK,
  STATE_RED_ESCAPE,
  STATE_ROTATE_TO_ENEMY,
  STATE_DEFEATED
};

bool rotatingToFrontEnemy = false;
BotState currentState = STATE_START;

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_TOP_LEFT, OUTPUT); pinMode(ECHO_TOP_LEFT, INPUT);
  pinMode(TRIG_TOP_RIGHT, OUTPUT); pinMode(ECHO_TOP_RIGHT, INPUT);
  pinMode(TRIG_BOTTOM_LEFT, OUTPUT); pinMode(ECHO_BOTTOM_LEFT, INPUT);
  pinMode(TRIG_BOTTOM_RIGHT, OUTPUT); pinMode(ECHO_BOTTOM_RIGHT, INPUT);

  // Color Sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  colorSensor.begin();

  // Servo
  hammer.attach(SERVO_PIN);
  hammer.write(0);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500); Serial.print("."); retry++;
    WiFi.begin(ssid, password);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to WiFi");
  } else {
    Serial.println("\n❌ WiFi Connection Failed");
  }
}

// ==== Movement Functions ====
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

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void rotateLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void rotateRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void rotateSlowLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void rotateSlowRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// ==== Sensor Utils ====
float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2.0;
}

String detectColorZone(uint16_t &r, uint16_t &g, uint16_t &b) {
  uint16_t c;
  colorSensor.getRawData(&r, &g, &b, &c);
  if (g > r * 1.2 && g > b * 1.2) return "GREEN";
  if (r > g * 1.2 && r > b * 1.2) return "RED";
  if (b > r * 1.2 && b > g * 1.2) return "BLUE";
  return "UNKNOWN";
}

bool confirmIRDetection(int pin) {
  unsigned long startTime = millis();
  while (millis() - startTime < IR_CONFIRM_TIME) {
    if (analogRead(pin) < IR_ANALOG_THRESHOLD) return false;
  }
  return true;
}

void punchHammer() {
  hammer.write(180);
  delay(PUNCH_DELAY);
  hammer.write(0);
  delay(PUNCH_DELAY);
}

void sendScoreUpdate() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(server);
    http.addHeader("Content-Type", "application/json");
    String payload = "{\"robot\":\"R2\",\"event\":\"edge_detected\",\"score\":1}";
    int response = http.POST(payload);
    Serial.print("📤 POST Response: "); Serial.println(response);
    http.end();
  } else {
    Serial.println("❌ WiFi not connected");
  }
}

// ==== FSM Loop ====
void loop() {

  // === HIGH PRIORITY: IR edge detection ===
  int frontIR = analogRead(IR_FRONT);
  int backIR  = analogRead(IR_BACK);

  if (frontIR > IR_ANALOG_THRESHOLD && backIR > IR_ANALOG_THRESHOLD && !alertSent) {
    Serial.println("⚠️ BOTH IRs triggered → Likely out of ring. Stopping.");
    stopMotors();
    sendScoreUpdate();
    alertSent = true;
    currentState = STATE_DEFEATED;
    return;
  }

  if (alertSent) {
    stopMotors();
    return;
  }

  if (frontIR > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_FRONT)) {
    Serial.println("🚨 FRONT IR triggered → Backing off.");
    moveBackward();
    delay(IR_REACT_TIME);
    stopMotors();
    return;
  }

  if (backIR > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_BACK)) {
    Serial.println("🚨 BACK IR triggered → Moving forward.");
    moveForward();
    delay(IR_REACT_TIME);
    stopMotors();
    return;
  }


  uint16_t r, g, b;
  String zone = detectColorZone(r, g, b);

  float fLeft = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
  float fRight = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
  float bLeft = readDistanceCM(TRIG_BOTTOM_LEFT, ECHO_BOTTOM_LEFT);
  float bRight = readDistanceCM(TRIG_BOTTOM_RIGHT, ECHO_BOTTOM_RIGHT);

  bool enemyFront = (fLeft < ENEMY_RANGE_CM && fLeft > 0) || (fRight < ENEMY_RANGE_CM && fRight > 0);
  bool enemyBack  = (bLeft < ENEMY_RANGE_CM && bLeft > 0) || (bRight < ENEMY_RANGE_CM && bRight > 0);
  bool enemyVeryCloseFront = (fLeft < 10 && fLeft > 0) || (fRight < 10 && fRight > 0);

  switch (currentState) {
    case STATE_START:
      Serial.println("🟢 STATE_START → Moving to green...");
      moveForward();
      if (zone == "GREEN") {
        stopMotors();
        currentState = STATE_SEARCH;
        Serial.println("✅ Entered GREEN → Start scanning");
        delay(300);
      }
      break;

    case STATE_SEARCH:
      Serial.println("🔍 STATE_SEARCH → Rotating...");
      if (enemyFront) {
        currentState = STATE_CHASE;
        stopMotors();
      } else if (enemyBack) {
        currentState = STATE_ROTATE_TO_ENEMY;
        rotatingToFrontEnemy = false;  // 🔄 Reset the rotation flag
        stopMotors();
      } else {
        rotateSlowLeft();
      }
      break;

    case STATE_ROTATE_TO_ENEMY:
  Serial.println("🔄 STATE_ROTATE_TO_ENEMY → Scanning for front enemy");

  // Start rotation if not already rotating
  if (!rotatingToFrontEnemy) {
    rotatingToFrontEnemy = true;
    Serial.println("🚦 Starting continuous rotation...");
  }

  // Keep rotating until front sensors detect enemy
  rotateSlowLeft();  // You can use right or decide based on rear sensor later

  // Check continuously if front sensors confirm enemy
  if (enemyFront) {
    Serial.println("🎯 Enemy now in front → Switching to CHASE mode");
    rotatingToFrontEnemy = false;
    stopMotors();
    currentState = STATE_CHASE;
  }

  break;


    case STATE_CHASE:{
    Serial.println("🏃 STATE_CHASE → Aligning to enemy...");

    // Threshold distance for enemy detection in cm
    const int detectionDistance = 50;

    bool enemyFrontClose = (fLeft < detectionDistance || fRight < detectionDistance);
    bool enemyBackClose = (bLeft < detectionDistance || bRight < detectionDistance);

    if (enemyVeryCloseFront) {
      currentState = STATE_ATTACK;
      stopMotors();
    } 
    else if (!enemyFront && !enemyBack) {
      // No enemy detected in front or back
      currentState = STATE_SEARCH;
      stopMotors();
    } 
    else if (enemyBackClose && !enemyFrontClose) {
      // Enemy detected behind → rotate until front detects enemy
      Serial.println("🔄 Enemy behind → Rotating to find enemy in front");
      rotateSlowRight();  // Rotate right slowly, or pick direction depending on your logic
    } 
    else if (enemyFrontClose) {
      // Enemy confirmed in front → chase with front sensor guidance
      int turnThreshold = 10;  // Threshold to decide turning

      int frontDiff = fLeft - fRight;

      if (frontDiff > turnThreshold) {
        Serial.println("↪️ Enemy right → Rotate right");
        rotateSlowRight();
      } 
      else if (frontDiff < -turnThreshold) {
        Serial.println("↩️ Enemy left → Rotate left");
        rotateSlowLeft();
      } 
      else {
        Serial.println("⬆️ Enemy in center → Move forward");
        moveForward();
      }

      // If in RED zone and enemy not very close, escape
      if (zone == "RED" && !enemyVeryCloseFront) {
        Serial.println("🟥 Red zone detected → Escaping");
        currentState = STATE_RED_ESCAPE;
        stopMotors();
      }
    } 
    else {
      // Fallback if no conditions met, just stop
      stopMotors();
    }
    break;}


    case STATE_ATTACK:
      Serial.println("👊 STATE_ATTACK → Punching");
      moveForward();
      punchHammer();
      stopMotors();
      currentState = STATE_SEARCH;
      break;

    case STATE_RED_ESCAPE:
      Serial.println("🚨 STATE_RED_ESCAPE → Reversing");
      moveBackward();
      delay(1000);
      stopMotors();
      currentState = STATE_SEARCH;
      break;

    case STATE_DEFEATED:
      Serial.println("💀 STATE_DEFEATED → Robot stopped");
      stopMotors();
      return;
  }

  delay(50);
}
