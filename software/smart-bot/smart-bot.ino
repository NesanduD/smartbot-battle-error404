#include <Wire.h>
#include <ESP32Servo.h>

// ==== Motor Pins ====
#define LEFT_MOTOR_FORWARD  23
#define LEFT_MOTOR_BACKWARD 22
#define RIGHT_MOTOR_FORWARD 21
#define RIGHT_MOTOR_BACKWARD 19

// ==== Servo ====
#define SERVO_PIN 18

// ==== Ultrasonic Sensors ====
#define TRIG_FRONT_LEFT   13
#define ECHO_FRONT_LEFT   32
#define TRIG_FRONT_RIGHT  5
#define ECHO_FRONT_RIGHT  15
#define TRIG_BACK_LEFT    4
#define ECHO_BACK_LEFT    2
#define TRIG_BACK_RIGHT   14
#define ECHO_BACK_RIGHT   27

#define CHASE_DISTANCE 50 // cm

Servo punchServo;

// ==== State Variables ====
bool rotatingFromBack = false;

// ==== Setup ====
void setup() {
  Serial.begin(115200);

  // Motors
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Ultrasonics
  pinMode(TRIG_FRONT_LEFT, OUTPUT);  pinMode(ECHO_FRONT_LEFT, INPUT);
  pinMode(TRIG_FRONT_RIGHT, OUTPUT); pinMode(ECHO_FRONT_RIGHT, INPUT);
  pinMode(TRIG_BACK_LEFT, OUTPUT);   pinMode(ECHO_BACK_LEFT, INPUT);
  pinMode(TRIG_BACK_RIGHT, OUTPUT);  pinMode(ECHO_BACK_RIGHT, INPUT);

  // Servo
  punchServo.attach(SERVO_PIN);
  punchServo.write(90);

  Serial.println("✅ Robot ready with chase-memory logic.");
}

// ==== Loop ====
void loop() {
  chaseTarget();
  delay(100);
}

// ==== Movement ====
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void stopMoving() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// ==== Ultrasonic Read ====
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 1000;
  return duration * 0.034 / 2;
}

// ==== Main Logic ====
void chaseTarget() {
  long distFL = readUltrasonic(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
  long distFR = readUltrasonic(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
  long distBL = readUltrasonic(TRIG_BACK_LEFT, ECHO_BACK_LEFT);
  long distBR = readUltrasonic(TRIG_BACK_RIGHT, ECHO_BACK_RIGHT);

  long minFront = min(distFL, distFR);
  long minBack = min(distBL, distBR);

  Serial.print("FL: "); Serial.print(distFL);
  Serial.print(" FR: "); Serial.print(distFR);
  Serial.print(" BL: "); Serial.print(distBL);
  Serial.print(" BR: "); Serial.println(distBR);

  if (minFront <= CHASE_DISTANCE) {
    Serial.println("🟢 Front Detected → Move Forward");
    moveForward();
    rotatingFromBack = false;
  } 
  else if (rotatingFromBack) {
    Serial.println("↻ Continuing Rotation → Awaiting Front Detection");
    turnRight();
  } 
  else if (minBack <= CHASE_DISTANCE) {
    Serial.println("🟠 Back Detected → Start Rotating");
    rotatingFromBack = true;
    turnRight();
  } 
  else {
    Serial.println("🔄 No Detection → Rotate to Search");
    rotatingFromBack = false;
    turnRight();
  }
}
