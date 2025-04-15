#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// Check if Bluetooth is enabled in the Arduino IDE
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;
Servo punchingArm; // Servo motor object

// Motor Driver Pins
#define LEFT_MOTOR_FORWARD  23  // Left motor forward pin
#define LEFT_MOTOR_BACKWARD 22  // Left motor backward pin
#define RIGHT_MOTOR_FORWARD 21 // Right motor forward pin
#define RIGHT_MOTOR_BACKWARD 19 // Right motor backward pin
#define SERVO_PIN 18 // Servo motor control pin

void setup() {
  // Initialize Serial Monitor with 115200 baud rate
  Serial.begin(115200);
  
  // Initialize Bluetooth Serial with a device name
  SerialBT.begin("UserRobot-BT");
  
  // Set motor driver pins as outputs
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  
  // Attach servo to its pin
  punchingArm.setPeriodHertz(50); // Standard 50Hz servo PWM
  punchingArm.attach(SERVO_PIN, 500, 2500); // Attach with min/max pulse width
  punchingArm.write(0); // Set initial position
  
  // Turn off motors initially
  stopMotors();
  
  // Print a message to confirm Bluetooth is ready
  Serial.println("UserRobot is ready to receive movement commands");
  Serial.println("Commands: 'f' (forward), 'b' (backward), 'l' (left), 'r' (right), 's' (stop), 'p' (punch)");
}

void loop() {
  // Check if data is available from Bluetooth
  if (SerialBT.available()) {
    // Read incoming character
    char receivedChar = SerialBT.read();
    
    // Process the received character
    switch(receivedChar) {
      case 'f': // Move forward
        moveForward();
        Serial.println("Moving forward");
        SerialBT.println("Moving forward");
        break;
      
      case 'b': // Move backward
        moveBackward();
        Serial.println("Moving backward");
        SerialBT.println("Moving backward");
        break;
      
      case 'l': // Turn left
        turnLeft();
        Serial.println("Turning left");
        SerialBT.println("Turning left");
        break;
      
      case 'r': // Turn right
        turnRight();
        Serial.println("Turning right");
        SerialBT.println("Turning right");
        break;
      
      case 's': // Stop
        stopMotors();
        Serial.println("Stopping");
        SerialBT.println("Stopping");
        break;
      
      case 'p': // Punching arm
        activatePunchingArm();
        Serial.println("Punching!");
        SerialBT.println("Punching!");
        break;
      
      default:
        Serial.println("Invalid command received");
        SerialBT.println("Invalid command. Use 'f', 'b', 'l', 'r', 's', 'p'");
    }
  }
  
  // Small delay to prevent overwhelming the serial communication
  delay(20);
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

// Function to move the robot backward
void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// Function to stop the robot
void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// Function to activate the punching arm
void activatePunchingArm() {
  punchingArm.write(90); // Move servo to 90 degrees (punch position)
  delay(300); // Hold position for a short duration
  punchingArm.write(0); // Return to original position
}
