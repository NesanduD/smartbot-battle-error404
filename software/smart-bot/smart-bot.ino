// ==== Pin Definitions ====
// Motor Driver Pins (L298N Dual H-Bridge)
#define LEFT_MOTOR_FORWARD  23  // Left motor forward pin
#define LEFT_MOTOR_BACKWARD 22  // Left motor backward pin
#define RIGHT_MOTOR_FORWARD 21 // Right motor forward pin
#define RIGHT_MOTOR_BACKWARD 19 // Right motor backward pin

// Servo Pin for Punch Arm
#define SERVO_PIN 32

// Ultrasonic Sensor Pins (For Movement Control)
// Updated sensor placement: top-left, top-right, bottom-left, bottom-right
#define TRIG_TOP_LEFT 13
#define ECHO_TOP_LEFT 2
#define TRIG_TOP_RIGHT 4
#define ECHO_TOP_RIGHT 32
#define TRIG_BOTTOM_LEFT 5
#define ECHO_BOTTOM_LEFT 15
#define TRIG_BOTTOM_RIGHT 14
#define ECHO_BOTTOM_RIGHT 27

// Proximity Sensor Pin (For Attack Control)
#define PROX_PIN 35  // Digital Output from Proximity Sensor

// IR Sensor Pins for Edge Detection
#define LEFT_IR_SENSOR 33
#define RIGHT_IR_SENSOR 34

// Sensor Thresholds
#define OBSTACLE_DISTANCE 20 // cm

// ==== Function Prototypes ====
// Movement Control
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMoving();

// Sensor Handling
void checkUltrasonicSensors();
long readUltrasonic(int trigPin, int echoPin);
void checkProximitySensor();  // Punch Arm activation
void checkEdgeDetection();    // Edge detection for falling

// External Features (Only Prototypes)
void activatePunchArm(); // Servo Punch
void detectColor();      // Color Strategy

#include <Servo.h>
Servo punchServo;

void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Ultrasonic Pins
  pinMode(TRIG_TOP_LEFT, OUTPUT);
  pinMode(ECHO_TOP_LEFT, INPUT);
  pinMode(TRIG_TOP_RIGHT, OUTPUT);
  pinMode(ECHO_TOP_RIGHT, INPUT);
  pinMode(TRIG_BOTTOM_LEFT, OUTPUT);
  pinMode(ECHO_BOTTOM_LEFT, INPUT);
  pinMode(TRIG_BOTTOM_RIGHT, OUTPUT);
  pinMode(ECHO_BOTTOM_RIGHT, INPUT);

  // Proximity Sensor
  pinMode(PROX_PIN, INPUT);

  // IR Sensor Pins for Edge Detection
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);

  // Servo Init
  punchServo.attach(SERVO_PIN);
  punchServo.write(90); // Initial Position
}

void loop() {
  checkProximitySensor();    // Punch Arm Activation
  checkUltrasonicSensors();  // Movement Control
  checkEdgeDetection();      // Edge Detection Logic
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

void stopMoving() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// ==== Ultrasonic Sensor Handling ==== 
void checkUltrasonicSensors() {
  long topLeftDist = readUltrasonic(TRIG_TOP_LEFT, ECHO_TOP_LEFT);
  long topRightDist = readUltrasonic(TRIG_TOP_RIGHT, ECHO_TOP_RIGHT);
  long bottomLeftDist = readUltrasonic(TRIG_BOTTOM_LEFT, ECHO_BOTTOM_LEFT);
  long bottomRightDist = readUltrasonic(TRIG_BOTTOM_RIGHT, ECHO_BOTTOM_RIGHT);

  // Basic movement control based on ultrasonic sensor readings
  if (topLeftDist < OBSTACLE_DISTANCE || topRightDist < OBSTACLE_DISTANCE ||
      bottomLeftDist < OBSTACLE_DISTANCE || bottomRightDist < OBSTACLE_DISTANCE) {
    moveBackward();
    delay(300);
    turnLeft();
    delay(300);
  } else if (topLeftDist < OBSTACLE_DISTANCE) {
    turnRight();
  } else if (topRightDist < OBSTACLE_DISTANCE) {
    turnLeft();
  } else if (bottomLeftDist < OBSTACLE_DISTANCE) {
    moveBackward();
  } else if (bottomRightDist < OBSTACLE_DISTANCE) {
    moveBackward();
  } else {
    moveForward();
  }
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

// ==== Proximity Sensor Punch Control ==== 
void checkProximitySensor() {
  if (// Add all of the conditions) {
    activatePunchArm(); // activate punch arm if all the conditions met
  }
}

// ==== Punch Arm Function ==== 
void activatePunchArm() {
  // Punch code
  // Reset code
}

// ==== Edge Detection Function ==== 
void checkEdgeDetection() {
  // Read the 2 IR sensors to detect edges
  // If an edge is detected on forward side, reverse the robot
  // Logic to find out if robot completely outside, then stop the robot and call the function lost();
}
