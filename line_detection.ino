#include <Arduino.h>

// ----------------------------
// Motor Pin Definitions
// ----------------------------
int motorAPin_A = 8;    // Left motor direction pin
int motorAPin_B = 9;    // Left motor speed (PWM)
int motorBPin_A = 4;    // Right motor direction pin
int motorBPin_B = 5;    // Right motor speed (PWM)

// ----------------------------
// Photocell Sensor Definition
// ----------------------------
const int SENSOR_LEFT = A3;    // Left photocell sensor
const int SENSOR_MIDDLE = A4;  // Middle photocell sensor
const int SENSOR_RIGHT = A5;   // Right photocell sensor

// ----------------------------
// Sensor Threshold and Baseline Values
// ----------------------------
const int THRESHOLD = 15;   // Adjust based on testing 
const int LEFT_BASE = 38;   // Adjust based on testing (left sensor base value)
const int MIDDLE_BASE = 40; // Adjust based on testing (middle sensor base value)
const int RIGHT_BASE = 45;  // Adjust based on testing (right sensor base value)

// ----------------------------
// Motor Control Functions
// ----------------------------

// Moves both motors forward
void moveMotorForward() {
  digitalWrite(motorAPin_A, HIGH);  // Set left motor forward
  analogWrite(motorAPin_B, LOW);   // Disable reverse

  digitalWrite(motorBPin_A, HIGH);  // Set right motor forward
  analogWrite(motorBPin_B, LOW);   // Disable reverse
}

// Moves both motors backward
void moveMotorBackward() {
  digitalWrite(motorAPin_A, LOW);   // Disable forward
  digitalWrite(motorAPin_B, 100);  // Set left motor backward

  digitalWrite(motorBPin_A, LOW);   // Disable forward
  digitalWrite(motorBPin_B, 100);  // Set right motor backward
}

// Turn right with pulsed correction
void adjustRight() {
  for (int i = 0; i < 5; i++) { 
    // Left wheel moves forward slowly
    analogWrite(motorAPin_A, 100);
    digitalWrite(motorAPin_B, LOW);
    // Right wheel moves forward at full speed
    analogWrite(motorBPin_A, 128);
    digitalWrite(motorBPin_B, LOW);
    delay(50);
    // moveMotorForward();
  }
}

void turnRight() {
  for (int i = 0; i < 10; i++) { 
    // Left wheel FORWARD
    analogWrite(motorAPin_A, 255);
    digitalWrite(motorAPin_B, HIGH);
    // RIGHT WHEEL SLOW 
    analogWrite(motorBPin_A, 100);
    digitalWrite(motorBPin_B, LOW);
    delay(50);
  }
}

// Turn left with pulsed correction
void adjustLeft() {
  for (int i = 0; i < 5; i++) { 
    // Left wheel moves forward at full speed
    analogWrite(motorAPin_A, 128);
    digitalWrite(motorAPin_B, LOW);
    // Right wheel moves forward slowly
    analogWrite(motorBPin_A, 100);
    digitalWrite(motorBPin_B, LOW);
    delay(50);
    // moveMotorForward();
  }
}


void turnLeft() {
  for (int i = 0; i < 10; i++) { 
    // Left wheel moves forward at full speed
    analogWrite(motorAPin_A, 100);
    digitalWrite(motorAPin_B, LOW);
    // Right wheel moves forward slowly
    analogWrite(motorBPin_A, 2);
    digitalWrite(motorBPin_B, LOW);
    delay(50);
    moveMotorForward();
  }
}

void setup() {
  Serial.begin(9600);
  
  // Set motor pins as outputs
  pinMode(motorAPin_A, OUTPUT);
  pinMode(motorAPin_B, OUTPUT);
  pinMode(motorBPin_A, OUTPUT);
  pinMode(motorBPin_B, OUTPUT);
  
  // Set sensor pins as inputs
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MIDDLE, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
}

void loop() {
  // Read sensor values
  int rawLeft = analogRead(SENSOR_LEFT);
  int rawMiddle = analogRead(SENSOR_MIDDLE);
  int rawRight = analogRead(SENSOR_RIGHT);

  // Print sensor values for debugging
  Serial.print("Left: ");
  Serial.print(rawLeft);
  Serial.print("  Middle: ");
  Serial.print(rawMiddle);
  Serial.print("  Right: ");
  Serial.println(rawRight);

  // Decision logic based on sensor readings
  // Compare the right sensor with its baseline and the left sensor with its baseline
  if ((rawMiddle > MIDDLE_BASE + THRESHOLD) && (rawRight >= (RIGHT_BASE + THRESHOLD))) {
    Serial.println("Turning RIGHT");
    turnRight();
    
  }
  else if ((rawMiddle > MIDDLE_BASE + THRESHOLD) && (rawLeft >= (LEFT_BASE + THRESHOLD))) {
    Serial.println("Turning LEFT");
    turnLeft();
  }
  else if (rawRight >= (RIGHT_BASE + THRESHOLD)) {
    Serial.println("Adjusting Right");
    adjustRight();
  }
  else if (rawLeft >= (LEFT_BASE + THRESHOLD)) {
    Serial.println("ADjusting Left");
    adjustLeft();
  }
  else {
    // moveMotorForward();
    // Serial.println("Moving Forward");
  }
  
  delay(75); // Small delay to improve stability
}
