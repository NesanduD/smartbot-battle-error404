#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>

// ==== Motor Pins ====
#define LEFT_MOTOR_FORWARD  23
#define LEFT_MOTOR_BACKWARD 22
#define RIGHT_MOTOR_FORWARD 21
#define RIGHT_MOTOR_BACKWARD 19

// ==== Servo ====
#define SERVO_PIN 18

// ==== Ultrasonic Sensors ====
#define TRIG_FRONT 13
#define ECHO_FRONT 32
#define TRIG_BACK 4
#define ECHO_BACK 2

// ==== Color Sensor Pins ====
#define SDA_PIN 25
#define SCL_PIN 26

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

Servo punchServo;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  tcs.begin();

  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  punchServo.attach(SERVO_PIN);
  punchServo.write(90); // Neutral position
}

// ===== Motor Functions =====
void stopMoving() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
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

void moveForwardLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void moveForwardRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// ===== Sensor Functions =====
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  return distance;
}

String getColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c == 0) return "unknown";
  float rRatio = (float)r / c;
  float gRatio = (float)g / c;
  float bRatio = (float)b / c;

  if (gRatio > 0.38 && gRatio > rRatio && gRatio > bRatio) return "green";
  if (bRatio > 0.35 && bRatio > gRatio && bRatio > rRatio) return "blue";
  if (rRatio > 0.4 && rRatio > gRatio && rRatio > bRatio) return "red";
  return "unknown";
}

// ===== Logic States =====
bool greenConfirmed = false;
unsigned long greenStartTime = 0;

void loop() {
  String color = getColor();
  long frontDistance = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long backDistance = readUltrasonic(TRIG_BACK, ECHO_BACK);

  Serial.print("Color: "); Serial.print(color);
  Serial.print(" | Front: "); Serial.print(frontDistance);
  Serial.print(" cm | Back: "); Serial.print(backDistance); Serial.println(" cm");

  if (!greenConfirmed) {
    if (color == "green") {
      if (greenStartTime == 0) greenStartTime = millis();
      if (millis() - greenStartTime >= 500) greenConfirmed = true;
    } else {
      greenStartTime = 0;
      moveForward();
    }
  } else {
    if (color == "red") {
      moveBackward();
      delay(500);
      rotateRight();
      delay(300);
      stopMoving();
    } else if (frontDistance < 50) {
      moveForward();
    } else if (backDistance < 50) {
      rotateLeft();
    } else {
      if (color == "green") {
        int randAction = random(0, 4);
        switch (randAction) {
          case 0: rotateLeft(); break;
          case 1: rotateRight(); break;
          case 2: moveForwardLeft(); break;
          case 3: moveForwardRight(); break;
        }
        delay(400);
        stopMoving();
      } else {
        rotateLeft();
      }
    }
  }
  delay(50);
}
