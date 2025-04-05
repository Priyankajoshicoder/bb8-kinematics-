// Define the motor control pins
#define MOT_A1_PIN 10
#define MOT_A2_PIN 9
#define MOT_B1_PIN 6
#define MOT_B2_PIN 5

// Define Bluetooth module pins (HC-06)
#define BT_RX 2  // Connect to HC-06 TX
#define BT_TX 3  // Connect to HC-06 RX

#include <SoftwareSerial.h>
SoftwareSerial bluetooth(BT_RX, BT_TX); // RX, TX for HC-06

int speedVal = 200;  // Default motor speed (0-255)
int turnSpeed = 100; // Reduced speed for turning

void setup() {
  // Set all the motor control pins as OUTPUT
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Ensure motors start in stopped position
  stopMotors();

  // Initialize Serial Monitor & Bluetooth
  Serial.begin(9600);
  bluetooth.begin(9600);  // HC-06 default baud rate

  Serial.println("Bluetooth Car Controller Ready");
  Serial.println("Waiting for commands...");
}

void loop() {
  char command = '\0';  // Default no command

  // Check for Serial Monitor input
  if (Serial.available() > 0) {
    command = Serial.read();
  }

  // Check for Bluetooth HC-06 input
  if (bluetooth.available() > 0) {
    command = bluetooth.read();
  }

  // Process command only if something is received
  if (command != '\0') {
    processCommand(command);
  }
}

void processCommand(char command) {
  command = toupper(command);  // Convert to uppercase
  Serial.print("Received Command: ");
  Serial.println(command);  // Debug output

  switch (command) {
    case 'W': 
    case 'F': 
      forward(); 
      Serial.println("Moving Forward");
      break;
    case 'B': 
      backward(); 
      Serial.println("Moving Backward");
      break;
    case 'A': 
    case 'L': 
      turnLeft(); 
      Serial.println("Turning Left in a radius");
      break;
    case 'D': 
    case 'R': 
      turnRight(); 
      Serial.println("Turning Right in a radius");
      break;
    case 'X': 
    case 'Q': 
      stopMotors(); 
      Serial.println("Stopped");
      break;
    default: 
      Serial.println("Invalid Command - Motors Stopped");
      stopMotors(); // Stop on unknown input
      break;
  }
}

void setMotor(int pwm_A, int pwm_B) {
  // Stop motors before changing direction
  analogWrite(MOT_A1_PIN, 0);
  analogWrite(MOT_A2_PIN, 0);
  analogWrite(MOT_B1_PIN, 0);
  analogWrite(MOT_B2_PIN, 0);

  delay(10); // Small delay to ensure stopping

  // Motor A
  if (pwm_A > 0) {
    analogWrite(MOT_A1_PIN, pwm_A);
    analogWrite(MOT_A2_PIN, 0);
  } else if (pwm_A < 0) {
    analogWrite(MOT_A1_PIN, 0);
    analogWrite(MOT_A2_PIN, -pwm_A);
  }

  // Motor B
  if (pwm_B > 0) {
    analogWrite(MOT_B1_PIN, pwm_B);
    analogWrite(MOT_B2_PIN, 0);
  } else if (pwm_B < 0) {
    analogWrite(MOT_B1_PIN, 0);
    analogWrite(MOT_B2_PIN, -pwm_B);
  }
}

void forward() {
  setMotor(speedVal, speedVal);
}

void backward() {
  setMotor(-speedVal, -speedVal);
}

// Modify turning functions to create a radius turn instead of abrupt stops
void turnLeft() {
  setMotor(turnSpeed, speedVal); // Left motor moves slower, right motor moves normally
}

void turnRight() {
  setMotor(speedVal, turnSpeed); // Right motor moves slower, left motor moves normally
}

void stopMotors() {
  setMotor(0, 0);
}
