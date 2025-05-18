#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// Ensure Bluetooth is enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;
Servo punchingArm;

// Motor Driver Pins
#define LEFT_MOTOR_FORWARD   23
#define LEFT_MOTOR_BACKWARD  22
#define RIGHT_MOTOR_FORWARD  21
#define RIGHT_MOTOR_BACKWARD 19
#define SERVO_PIN 18

unsigned long lastCommandTime = 0;
const unsigned long timeout = 30000; // 30 seconds

void setup() {
  Serial.begin(115200);
  SerialBT.begin("UserRobot-BT");

  // Motor pins as output
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Servo setup
  punchingArm.setPeriodHertz(50); 
  punchingArm.attach(SERVO_PIN, 500, 2500);
  punchingArm.write(0);

  stopMotors();

  Serial.println("UserRobot ready | Commands: f b l r s p");
}

void loop() {
  // Handle Bluetooth disconnection
  if (!SerialBT.hasClient()) {
    stopMotors();  // Safety stop
    return;
  }

  // Handle command input
  if (SerialBT.available()) {
    char receivedChar = SerialBT.read();
    lastCommandTime = millis();  // Reset inactivity timer

    // Flush any junk data
    while (SerialBT.available()) SerialBT.read();

    // Ignore non-letter inputs
    if (!isalpha(receivedChar)) {
      Serial.println("Ignored non-alphabet character");
      return;
    }

    // Execute command
    switch (receivedChar) {
      case 'f': moveForward();  break;
      case 'b': moveBackward(); break;
      case 'l': turnLeft();     break;
      case 'r': turnRight();    break;
      case 's': stopMotors();   break;
      case 'p': activatePunchingArm(); break;
      default:
        Serial.println("Unknown command");
    }
  }

  // Stop robot if no commands for a while
  if (millis() - lastCommandTime > timeout) {
    stopMotors();
  }

  delay(10); // Yield to background tasks
}

// Movement Functions
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  Serial.println("Forward");
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  Serial.println("Backward");
}

void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  Serial.println("Left");
}

void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  Serial.println("Right");
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  Serial.println("Stop");
}

void activatePunchingArm() {
  punchingArm.write(90);
  delay(300);
  punchingArm.write(0);
  Serial.println("Punch");
}
