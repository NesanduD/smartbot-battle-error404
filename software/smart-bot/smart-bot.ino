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

// ==== Other Sensors ====
#define PROX_PIN 33
#define LEFT_IR_SENSOR 35
#define RIGHT_IR_SENSOR 34

#define CHASE_DISTANCE 50 // cm

Servo punchServo;

// ==== State Variables ====
bool rotatingFromBack = false;
bool lastProxState = LOW;

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

  // Sensors
  pinMode(PROX_PIN, INPUT);
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);

  // Servo
  punchServo.attach(SERVO_PIN);
  punchServo.write(90); // Initial position

  Serial.println("✅ Robot ready with chase + punch + edge detection.");
}

// ==== Loop ====
void loop() {
  checkProximitySensor();    // Punch if opponent detected
  checkEdgeSensors();        // IR check
  chaseTarget();             // Movement logic
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

// ==== Punch Logic ====
void checkProximitySensor() {
  bool currentState = digitalRead(PROX_PIN);

  if (currentState == HIGH && lastProxState == LOW) {
    Serial.println("💥 Opponent Detected → Punch!");
    punchServo.write(0);   // Punch
    delay(250);
    punchServo.write(90);  // Reset
  }

  lastProxState = currentState;
}

// ==== IR Sensor Check ====
void checkEdgeSensors() {
  bool leftEdge = digitalRead(LEFT_IR_SENSOR) == HIGH;
  bool rightEdge = digitalRead(RIGHT_IR_SENSOR) == HIGH;

  if (leftEdge && rightEdge) {
    Serial.println("⚠️ BOTH IR SENSORS DETECTED EDGE");
  }
}
