#include <Arduino.h>

// Motor pins
const int LEFT_FORWARD = 8;
const int LEFT_BACKWARD = 9;
const int RIGHT_FORWARD = 4;
const int RIGHT_BACKWARD = 5;

// Sensor pins
const int TRIG_FRONT = 12;
const int ECHO_FRONT = 11;
const int TRIG_LEFT = 2;
const int ECHO_LEFT = 3;

// LED pins
const int LED_FRONT = 13;

// Distance Thresholds
const int STOP_DISTANCE = 10;  // Stop if obstacle ahead (cm)
const int TARGET_LEFT = 15;    // Desired distance from the wall (cm)
const int TOLERANCE = 0;       // Acceptable error in wall distance (cm)

// PID Constants (for proportional control)
const float Kp = 3.0;          // Adjusts how aggressively the robot corrects

// Motor Speeds
const int BASE_SPEED = 130;    // Normal speed
const int TURN_SPEED = 25;     // Slower speed for turns

void setup() {
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(LED_FRONT, OUTPUT);
  
  Serial.begin(9600);
}

// Function to get distance from ultrasonic sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert time to distance (cm)
}

// --- Motor control functions ---
void moveForward() {
  Serial.println("Moving Forward...");
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, LOW);
  
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, LOW);
}

void moveBackward() {
  Serial.println("Moving Backward...");
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, HIGH);
  
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, HIGH);
}

void turnLeft() {
  Serial.println("Turning Left...");
  digitalWrite(LEFT_FORWARD, LOW);   // Stop left motor
  digitalWrite(LEFT_BACKWARD, LOW);
  
  digitalWrite(RIGHT_FORWARD, 25); // Right motor keeps moving
  digitalWrite(RIGHT_BACKWARD, LOW);
}

void turnRight() {
  Serial.println("Turning Right...");
  digitalWrite(RIGHT_FORWARD, LOW);  // Stop right motor
  digitalWrite(RIGHT_BACKWARD, LOW);
  
  digitalWrite(LEFT_FORWARD, 25);  // Left motor keeps moving
  digitalWrite(LEFT_BACKWARD, LOW);
}

void stopRobot() {
  Serial.println("Stopping...");
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, LOW);
  
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, LOW);
}

void loop() {
  // Read sensor distances
  int frontDist = getDistance(TRIG_FRONT, ECHO_FRONT);
  int leftDist = getDistance(TRIG_LEFT, ECHO_LEFT);
  
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" cm, Left: ");
  Serial.print(leftDist);
  Serial.println(" cm");

  // 🚨 Step 1: Check for obstacles in front
  if (frontDist > 0 && frontDist < STOP_DISTANCE) {  
    Serial.println("Obstacle detected! Moving back and adjusting.");
    digitalWrite(LED_FRONT, HIGH);
    
    moveBackward();
    delay(500);  // Move back slightly
    
    turnRight();
    delay(500);  // Small left turn
    
    moveForward(); // Resume movement
    return;
  }
  digitalWrite(LED_FRONT, LOW);

  // 🚀 Step 2: Wall Following (Proportional Control)
  int error = TARGET_LEFT - leftDist;
  int correction = Kp * error;  // Adjust turning based on distance

  Serial.print("Correction: ");
  Serial.println(correction);

  // If too close to the wall → turn right (slow down left motor)
  if (leftDist < TARGET_LEFT - TOLERANCE) {
    Serial.println("Too close to the wall! Adjusting right.");
    turnRight();
    delay(300);
    moveForward();
  } 
  // If too far from the wall → turn left (slow down right motor)
  else if (leftDist > TARGET_LEFT + TOLERANCE) {
    Serial.println("Too far from the wall! Adjusting left.");
    turnLeft();
    delay(300);
    moveForward();
  } 
  // If distance is fine, move straight
  else {
    moveForward();
  }
}