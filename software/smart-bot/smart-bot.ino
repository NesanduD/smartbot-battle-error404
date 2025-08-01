
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
#define IR_CONFIRM_TIME 10         // in milliseconds
#define IR_REACT_TIME 1500          // move time if confirmed

// ==== WiFi Settings ====
const char* ssid = "espadmin";
const char* password = "12345678";
const char* server = "http://192.168.209.168:5000/score";
bool alertSent = false;

// ==== State ====
bool inGreenZone = false;

// ==== Objects ====
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo hammer;

// ==== Setup ====
void setup() 
{
  Serial.begin(115200);

  // Motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_TOP_LEFT, OUTPUT); pinMode(ECHO_TOP_LEFT, INPUT);
  pinMode(TRIG_TOP_RIGHT, OUTPUT); pinMode(ECHO_TOP_RIGHT, INPUT);
  pinMode(TRIG_BOTTOM_LEFT, OUTPUT); pinMode(ECHO_BOTTOM_LEFT, INPUT);
  pinMode(TRIG_BOTTOM_RIGHT, OUTPUT); pinMode(ECHO_BOTTOM_RIGHT, INPUT);

  // Color sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  colorSensor.begin();

  // Servo
  hammer.attach(SERVO_PIN);
  hammer.write(0);  // Initial hammer position

WiFi.begin(ssid, password);
Serial.print("Connecting to WiFi");

int retry = 0;
while (WiFi.status() != WL_CONNECTED && retry < 20) {
  delay(500);
  Serial.print(".");
  retry++;
  WiFi.begin(ssid, password);
}

if (WiFi.status() == WL_CONNECTED) {
  Serial.println("\n✅ Connected to WiFi!");
  Serial.print("📶 IP Address: ");
  Serial.println(WiFi.localIP());
} else {
  Serial.println("\n❌ Failed to connect to WiFi!");
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
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
}

// ==== Utility Functions ====
float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
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


// ==== Confirm Continuous IR Detection ====
bool confirmIRDetection(int pin) {
  unsigned long startTime = millis();
  while (millis() - startTime < IR_CONFIRM_TIME) {
    int val = analogRead(pin);
    if (val <= IR_ANALOG_THRESHOLD) {
      return false;  // If it drops below during confirm window, reject
    }
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

    Serial.print("📤 POST Response code: ");
    Serial.println(httpResponseCode);

    http.end();
  } else {
    Serial.println("❌ WiFi not connected, cannot send score.");
  }
}


// ==== Main Loop ====
void loop() 
{

  // ==== IR Sensor Analog Logic with Confirmation ====
int frontIR = analogRead(IR_FRONT);
int backIR = analogRead(IR_BACK);

if (alertSent) 
{
  stopMotors(); // Freeze bot
  return;       // Skip rest of the loop
}

Serial.print("Front IR: "); Serial.print(frontIR);
Serial.print(" | Back IR: "); Serial.println(backIR);

// === DEFEAT Detection ===
if (frontIR > IR_ANALOG_THRESHOLD && backIR > IR_ANALOG_THRESHOLD && !alertSent) {
  Serial.println("💀 DEFEAT DETECTED — Both IR sensors triggered!");
  stopMotors();
  sendScoreUpdate();
  alertSent = true;
  return;  // stop the loop completely
}

// Reset alert if lifted
if (frontIR < IR_ANALOG_THRESHOLD && backIR < IR_ANALOG_THRESHOLD) {
  alertSent = false;  // Ready for next match
}

  if (frontIR > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_FRONT)) {
    Serial.println("✅ Confirmed FRONT IR detection → Going BACKWARD...");
    moveBackward();
    delay(IR_REACT_TIME);
    stopMotors();
    return;
  }

  if (backIR > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_BACK)) {
    Serial.println("✅ Confirmed BACK IR detection → Going FORWARD...");
    moveForward();
    delay(IR_REACT_TIME);
    stopMotors();
    return;
  }

  // ==== Color Detection ====
  uint16_t r, g, b;
  String currentZone = detectColorZone(r, g, b);

  Serial.println("==== ZONE & COLOR INFO ====");
  Serial.print("Detected Zone: "); Serial.println(currentZone);
  Serial.print("R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.println(b);

  if (!inGreenZone) {
    moveForward();
    if (currentZone == "GREEN") {
      stopMotors();
      inGreenZone = true;
      Serial.println("✅ Green zone reached. Begin enemy scan.");
      delay(500);
    }
  } else {
    if (currentZone == "RED") {
  Serial.println("🚨 Red zone detected!");

  float frontLeft = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
  float frontRight = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
  bool enemyVeryCloseInFront = (frontLeft > 0 && frontLeft < 10) || (frontRight > 0 && frontRight < 10);

  if (enemyVeryCloseInFront) {
    Serial.println("🛡️ Enemy detected within 10cm IN RED zone → Pushing...");

    while (true) {
  moveForward();        // Keep pushing
  punchHammer();        // Keep punching while pushing

  int frontIR = analogRead(IR_FRONT);
  if (frontIR > IR_ANALOG_THRESHOLD && confirmIRDetection(IR_FRONT)) {
    Serial.println("⬛ Front IR detected BLACK line → Retreating...");
    moveBackward();
    delay(IR_REACT_TIME);
    stopMotors();
    break;
  }

  delay(100);  // Delay to give servo time and reduce spam
}


    return;
  } else {
    Serial.println("🚨 In RED zone without pushing → Backing off.");
    moveBackward();
    delay(1500);
    stopMotors();
    return;
  }
}


    // ==== Ultrasonic readings ====
    float frontLeft = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
    float frontRight = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
    float backLeft = readDistanceCM(TRIG_BOTTOM_LEFT, ECHO_BOTTOM_LEFT);
    float backRight = readDistanceCM(TRIG_BOTTOM_RIGHT, ECHO_BOTTOM_RIGHT);

    Serial.println("==== ULTRASONIC READINGS (cm) ====");
    Serial.print("Front Left: "); Serial.println(frontLeft);
    Serial.print("Front Right: "); Serial.println(frontRight);
    Serial.print("Back Left: "); Serial.println(backLeft);
    Serial.print("Back Right: "); Serial.println(backRight);

    bool enemyFront = (frontLeft > 0 && frontLeft < ENEMY_RANGE_CM) ||
                      (frontRight > 0 && frontRight < ENEMY_RANGE_CM);
    bool enemyBack = (backLeft > 0 && backLeft < ENEMY_RANGE_CM) ||
                     (backRight > 0 && backRight < ENEMY_RANGE_CM);

    if (enemyFront) {
      Serial.println("🎯 Enemy detected IN FRONT. Charging...");
      moveForward();
      delay(200);
      punchHammer();
    } else if (enemyBack) {
      Serial.println("🔄 Enemy detected BEHIND. Rotating to face...");
      while (true) {
        rotateLeft();
        float newFrontLeft = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
        float newFrontRight = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
        if ((newFrontLeft > 0 && newFrontLeft < ENEMY_RANGE_CM) ||
            (newFrontRight > 0 && newFrontRight < ENEMY_RANGE_CM)) {
          Serial.println("✅ Enemy now in front. Charging...");
          break;
        }
      }
    } else {
      Serial.println("😐 No enemy detected. Rotating until enemy found...");
      while (true) {
        rotateLeft();
        float newFL = readDistanceCM(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
        float newFR = readDistanceCM(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
        float newBL = readDistanceCM(TRIG_BOTTOM_LEFT, ECHO_BOTTOM_LEFT);
        float newBR = readDistanceCM(TRIG_BOTTOM_RIGHT, ECHO_BOTTOM_RIGHT);
        if ((newFL > 0 && newFL < ENEMY_RANGE_CM) ||
            (newFR > 0 && newFR < ENEMY_RANGE_CM) ||
            (newBL > 0 && newBL < ENEMY_RANGE_CM) ||
            (newBR > 0 && newBR < ENEMY_RANGE_CM)) {
          Serial.println("✅ Enemy found while rotating.");
          break;
        }
      }
    }
  }

  Serial.println("==================================");
  delay(50);
} 