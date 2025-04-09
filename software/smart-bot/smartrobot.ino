// ==== Pin Definitions ====
// Motor Driver Pins (L298N Dual H-Bridge)
#define LEFT_MOTOR_FORWARD  23  // Left motor forward pin
#define LEFT_MOTOR_BACKWARD 22  // Left motor backward pin
#define RIGHT_MOTOR_FORWARD 21 // Right motor forward pin
#define RIGHT_MOTOR_BACKWARD 19 // Right motor backward pin

// Servo Pin for Punch Arm
#define SERVO_PIN 32

// Ultrasonic Sensor Pins (For Movement Control)
#define TRIG_FRONT 4
#define ECHO_FRONT 5
#define TRIG_LEFT 18
#define ECHO_LEFT 19
#define TRIG_RIGHT 21
#define ECHO_RIGHT 22
#define TRIG_BACK 23
#define ECHO_BACK 34

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
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

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
  long frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long leftDist  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long rightDist = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  long backDist  = readUltrasonic(TRIG_BACK, ECHO_BACK);

  // Basic movement control based on ultrasonic sensor readings
  if (frontDist < OBSTACLE_DISTANCE) {
    moveBackward();
    delay(300);
    turnLeft();
    delay(300);
  } else if (leftDist < OBSTACLE_DISTANCE) {
    turnRight();
  } else if (rightDist < OBSTACLE_DISTANCE) {
    turnLeft();
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
}

// ==== Color Detection Strategy ====
void detectColor() {
  // Color sensor reading and strategy switching logic goes here
}
