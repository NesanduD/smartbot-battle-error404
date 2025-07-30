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
#define PUSHING_RANGE_CM 15
#define PUNCH_DELAY 500
#define IR_ANALOG_THRESHOLD 1000
#define IR_CONFIRM_TIME 50
#define IR_REACT_TIME 1500

// ==== WiFi Settings ====
const char* ssid = "espadmin";
const char* password = "12345678";
const char* server = "http://192.168.8.172:5000/score";

bool alertSent = false;

// ==== State ====
bool inGreenZone = false;

// ==== Objects ====
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo hammer;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  pinMode(TRIG_TOP_LEFT, OUTPUT); pinMode(ECHO_TOP_LEFT, INPUT);
  pinMode(TRIG_TOP_RIGHT, OUTPUT); pinMode(ECHO_TOP_RIGHT, INPUT);
  pinMode(TRIG_BOTTOM_LEFT, OUTPUT); pinMode(ECHO_BOTTOM_LEFT, INPUT);
  pinMode(TRIG_BOTTOM_RIGHT, OUTPUT); pinMode(ECHO_BOTTOM_RIGHT, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  colorSensor.begin();

  hammer.attach(SERVO_PIN);
  hammer.write(0);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
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

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void rotateLeftSlow() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  delay(300);
  stopMotors();
}

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
  colorSensor.setInterrupt(true);

  if (g > r * 1.2 && g > b * 1.2) return "GREEN";
  if (r > g * 1.2 && r > b * 1.2) return "RED";
  if (b > r * 1.2 && b > g * 1.2) return "BLUE";

  return "UNKNOWN";
}

void punchHammer() {
  hammer.write(180);
  delay(PUNCH_DELAY);
  hammer.write(0);
  delay(PUNCH_DELAY);
}

bool confirmIRDetection(int pin) {
  unsigned long startTime = millis();
  while (millis() - startTime < IR_CONFIRM_TIME) {
    if (analogRead(pin) <= IR_ANALOG_THRESHOLD) return false;
  }
  return true;
}

void sendScoreUpdate() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(server);
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"robot\":\"R2\",\"event\":\"edge_detected\",\"score\":1}";
    int httpResponseCode = http.POST(payload);

    Serial.print("POST Response code: ");
    Serial.println(httpResponseCode);

    http.end();
  } else {
    Serial.println("WiFi not connected, cannot send score.");
  }
}

void loop() {
  int frontIR = analogRead(IR_FRONT);
  int backIR  = analogRead(IR_BACK);

  Serial.print("Front IR: "); Serial.print(frontIR);
  Serial.print(" | Back IR: "); Serial.println(backIR);

  if (frontIR > IR_ANALOG_THRESHOLD && backIR > IR_ANALOG_THRESHOLD &&
      confirmIRDetection(IR_FRONT) && confirmIRDetection(IR_BACK)) {
    stopMotors();
    Serial.println("🛑 BOTH IR triggered! System lost. Sending signal...");
    if (!alertSent) {
      sendScoreUpdate();
      alertSent = true;
    }
    while (true) {
      stopMotors();
      delay(1000);
    }
  } else {
    alertSent = false;
  }

  if (frontIR > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_FRONT)) {
    Serial.println("⬛ FRONT IR DETECTED → GO BACK");
    moveBackward();
    delay(IR_REACT_TIME);
    stopMotors();
    return;
  }

  if (backIR > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_BACK)) {
    Serial.println("⬛ BACK IR DETECTED → GO FORWARD");
    moveForward();
    delay(IR_REACT_TIME);
    stopMotors();
    return;
  }

  uint16_t r, g, b;
  String currentZone = detectColorZone(r, g, b);

  if (!inGreenZone) {
    moveForward();
    if (currentZone == "GREEN") {
      stopMotors();
      inGreenZone = true;
      Serial.println("✅ Green zone reached. Begin scan.");
      delay(500);
    }
  } else {
    float frontLeft = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
    float frontRight = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
    float backLeft = readDistanceCM(TRIG_BOTTOM_LEFT, ECHO_BOTTOM_LEFT);
    float backRight = readDistanceCM(TRIG_BOTTOM_RIGHT, ECHO_BOTTOM_RIGHT);

    bool enemyFront = (frontLeft > 0 && frontLeft < ENEMY_RANGE_CM) ||
                      (frontRight > 0 && frontRight < ENEMY_RANGE_CM);
    bool enemyBack = (backLeft > 0 && backLeft < ENEMY_RANGE_CM) ||
                     (backRight > 0 && backRight < ENEMY_RANGE_CM);

    if (currentZone == "RED" &&
        ((frontLeft > 0 && frontLeft < PUSHING_RANGE_CM) ||
         (frontRight > 0 && frontRight < PUSHING_RANGE_CM))) {
      Serial.println("🤜 PUSHING ENEMY OUT OF RING");
      while (true) {
        moveForward();
        if (analogRead(IR_FRONT) > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_FRONT)) {
          Serial.println("⬛ FRONT IR HIT → BACK OFF AFTER PUSH");
          moveBackward();
          delay(IR_REACT_TIME);
          stopMotors();
          break;
        }
        delay(50);
      }
      return;
    }

    if (enemyFront) {
      Serial.println("🎯 Enemy in front. Charging...");
      moveForward();
      delay(200);
      punchHammer();
    } else if (enemyBack) {
      Serial.println("🔄 Enemy behind. Rotating...");
      while (true) {
        rotateLeftSlow();
        float newFL = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
        float newFR = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
        if ((newFL > 0 && newFL < ENEMY_RANGE_CM) ||
            (newFR > 0 && newFR < ENEMY_RANGE_CM)) {
          Serial.println("✅ Enemy now in front.");
          break;
        }
      }
    } else {
      Serial.println("🔄 Scanning for enemy...");
      while (true) {
        rotateLeftSlow();
        float newFL = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
        float newFR = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
        float newBL = readDistanceCM(TRIG_BOTTOM_LEFT, ECHO_BOTTOM_LEFT);
        float newBR = readDistanceCM(TRIG_BOTTOM_RIGHT, ECHO_BOTTOM_RIGHT);
        if ((newFL > 0 && newFL < ENEMY_RANGE_CM) ||
            (newFR > 0 && newFR < ENEMY_RANGE_CM) ||
            (newBL > 0 && newBL < ENEMY_RANGE_CM) ||
            (newBR > 0 && newBR < ENEMY_RANGE_CM)) {
          Serial.println("✅ Enemy found.");
          break;
        }
      }
    }
  }

  delay(50);
}
