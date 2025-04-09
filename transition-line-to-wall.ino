#include <Servo.h>
/**
 * Arduino Robot with Line Following and Wall Following capabilities
 * Features transition from line following to wall following mode
 */

// Motor Pins
int motorAPin_A = 8;    // Left Forward
int motorAPin_B = 9;    // Left Backward
int motorBPin_A = 7;    // Right Forward
int motorBPin_B = 6;    // Right Backward

// IR Sensor Pins
const int L_IR = A2;    // Left IR Sensor
const int M_IR = A1;    // Middle IR Sensor
const int R_IR = A0;    // Right IR Sensor

// Ultrasonic Sensor Pins
#define TRIG_FRONT 13     // Front ultrasonic trigger
#define ECHO_FRONT 10     // Front ultrasonic echo
#define TRIG_LEFT 2       // Left ultrasonic trigger
#define ECHO_LEFT 3       // Left ultrasonic echo
#define TRIG_RIGHT 11     // Right ultrasonic trigger
#define ECHO_RIGHT 5     // Right ultrasonic echo

// Stage control flags
bool startLine = true;   // Start with line following
bool startWall = false;  // Wall following initially disabled
bool startSweep = false; // Sweeping mode initially disabled

// Constants for Wall Following
float Kp = 1.15;
float Ki = 0.0;
float Kd = 0.25;
const int TARGET_DISTANCE = 15;   // Target distance from wall in cm
const int FRONT_THRESHOLD = 15;   // Distance to trigger turning (front sensor)
const int MAX_DISTANCE = 200;     // Maximum measurable distance
const int BASE_SPEED_WALL = 150;       // Base motor speed (0-255)
const int MAX_SPEED = 200;        // Maximum motor speed
float lastErrorLeft = 0;
float lastErrorRight = 0;
float integralLeft = 0;
float integralRight = 0;
long durationFront, durationLeft, durationRight;
int distanceFront, distanceLeft, distanceRight;
float filteredFront = 0;
float filteredLeft = 0;
float filteredRight = 0;
#define FILTER_FACTOR 0.3  // Range: 0.0 to 1.0 (higher = smoother but slower to react)
const int WALL_DETECTION_THRESHOLD = 20; // Distance in cm that indicates a wall is close enough

// Constants for Line Following
#define BASE_SPEED_LINE 200           // Base PWM value (0-255)
#define RIGHT_COMPENSATION 30    // Additional power to right motor to compensate for left lean
#define CORRECTION_FACTOR 0.8    // How strongly to correct based on sensor readings
int leftSpeedLine;
int rightSpeedLine;
unsigned long previousMillis = 0;
const long adjustInterval = 100;  // Adjust every 100ms

// Constants for Object Detection
const int FORWARD_TIME = 4000;  // Time to move forward in milliseconds
const int TURN_TIME = 550;      // Time to turn left in milliseconds
const int RIGHT_TURN_TIME = 550; // Time to turn right in milliseconds
const int MIN_DISTANCE_OBJECT_DETECTION = 20;    // Minimum distance to trigger right turn (cm)
const int MAX_DISTANCE_OBJECT_DETECTION = 200;   // Maximum valid distance reading (cm)

// Global variable for counting consecutive detections
int objectDetectionCount = 0;
const int REQUIRED_COUNT = 3;  // Number of consecutive detections needed

void setup() {
  // Set motor pins as outputs
  pinMode(motorAPin_A, OUTPUT);
  pinMode(motorAPin_B, OUTPUT);
  pinMode(motorBPin_A, OUTPUT);
  pinMode(motorBPin_B, OUTPUT);
  
  // Set IR sensor pins as inputs
  pinMode(L_IR, INPUT);
  pinMode(M_IR, INPUT);
  pinMode(R_IR, INPUT);
  
  // Setup ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Initialize motor speeds with compensation
  leftSpeedLine = BASE_SPEED_LINE;
  rightSpeedLine = BASE_SPEED_LINE + RIGHT_COMPENSATION;

  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Stop motors at startup
  stopMotors();
  
  // Wait for a moment before starting
  delay(2000);
}

void loop() {
  // Check which mode we're currently in
  if (startLine) {
    lineFollowingMode();
    
    // Check if wall is detected to transition
    readUltrasonicSensors(); // Read ultrasonic sensors
    
    // If either left or right wall is detected within threshold
    if (distanceLeft < WALL_DETECTION_THRESHOLD || distanceRight < WALL_DETECTION_THRESHOLD) {
      // Transition to wall following mode
      startLine = false;
      startWall = true;
      startSweep = false;
      
      Serial.println("MODE TRANSITION: Line Following → Wall Following");
      
      // Brief pause to stabilize before mode switch
      stopMotors();
      delay(500);
    }
  } 
  else if (startWall) {
    wallFollowingMode();
    
    // Add any transitions from wall following to other modes here if needed
  }
  else if (startSweep) {
    sweepMode();
    
    // Add any transitions from sweep mode to other modes here if needed
  }
  
  // Small delay for stability
  delay(50);
}

void lineFollowingMode() {
  unsigned long currentMillis = millis();
  
  // Drive forward with periodic adjustments
  moveForward(leftSpeedLine, rightSpeedLine);
  
  // Adjust direction at regular intervals
  if (currentMillis - previousMillis >= adjustInterval) {
    previousMillis = currentMillis;
    adjustDirection();
  }
}

float getUltrasonicDistance(int trigPin, int echoPin) {
  // Clear the trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trig pin high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin
  float duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in centimeters
  return duration * 0.034 / 2;
}

void adjustDirection() {
  // Get distances from left and right sensors
  float leftDistance = getUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);
  float rightDistance = getUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // Print current state for debugging
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right: ");
  Serial.print(rightDistance);
  Serial.print(" cm, L Speed: ");
  Serial.print(leftSpeedLine);
  Serial.print(", R Speed: ");
  Serial.println(rightSpeedLine);
  
  // Only adjust if both sensors are detecting something (within 200cm)
  if (leftDistance < 200 && rightDistance < 200) {
    int adjustment = (int)((rightDistance - leftDistance) * CORRECTION_FACTOR);
    
    // Apply adjustments to motor speeds
    leftSpeedLine = constrain(BASE_SPEED_LINE + adjustment, BASE_SPEED_LINE - 50, BASE_SPEED_LINE + 50);
    rightSpeedLine = constrain(BASE_SPEED_LINE + RIGHT_COMPENSATION - adjustment, 
                          BASE_SPEED_LINE + RIGHT_COMPENSATION - 50, 
                          BASE_SPEED_LINE + RIGHT_COMPENSATION + 50);
  } else {
    // Reset to base values with compensation if no valid readings
    leftSpeedLine = BASE_SPEED_LINE;
    rightSpeedLine = BASE_SPEED_LINE + RIGHT_COMPENSATION;
  }
}

void wallFollowingMode() {
    // Read distance from all ultrasonic sensors
  readUltrasonicSensors();
  
  // Print sensor readings for debugging
  printUltrasonicSensorReadings();
  
  // Wall following logic with obstacle avoidance
  followWall();
  
  // Small delay for stability
  delay(50);

}

void readUltrasonicSensors() {
  // Read front ultrasonic sensor
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  durationFront = pulseIn(ECHO_FRONT, HIGH, 30000);
  distanceFront = durationFront * 0.034 / 2;
  
  // Read left ultrasonic sensor
  digitalWrite(TRIG_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_LEFT, LOW);
  durationLeft = pulseIn(ECHO_LEFT, HIGH, 30000);
  distanceLeft = durationLeft * 0.034 / 2;
  
  // Read right ultrasonic sensor
  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_RIGHT, LOW);
  durationRight = pulseIn(ECHO_RIGHT, HIGH, 30000);
  distanceRight = durationRight * 0.034 / 2;
  
  // Clamp raw distance values first like you already do
  if (distanceFront <= 0 || distanceFront > MAX_DISTANCE) distanceFront = MAX_DISTANCE;
  if (distanceLeft <= 0 || distanceLeft > MAX_DISTANCE) distanceLeft = MAX_DISTANCE;
  if (distanceRight <= 0 || distanceRight > MAX_DISTANCE) distanceRight = MAX_DISTANCE;

  // Apply exponential filter
  filteredFront = FILTER_FACTOR * filteredFront + (1 - FILTER_FACTOR) * distanceFront;
  filteredLeft = FILTER_FACTOR * filteredLeft + (1 - FILTER_FACTOR) * distanceLeft;
  filteredRight = FILTER_FACTOR * filteredRight + (1 - FILTER_FACTOR) * distanceRight;
}

void printUltrasonicSensorReadings() {
  // Print sensor readings for debugging
  Serial.print("WALL MODE - Front: ");
  Serial.print(filteredFront);
  Serial.print(" cm | Left: ");
  Serial.print(filteredLeft);
  Serial.print(" cm | Right: ");
  Serial.print(filteredRight);
  Serial.println(" cm");
}


void followWall() {
  // Check if there's an obstacle in front
  if (filteredFront < FRONT_THRESHOLD) {
    // Obstacle detected in front, turn to avoid it
    moveBackward(BASE_SPEED_WALL);
    delay(300);
    if (filteredLeft > filteredRight) {
      // More space on the left, turn left
      turnRight();
      Serial.println("Obstacle ahead! Turning left.");
    } else {
      // More space on the right, turn right
      turnLeft();
      Serial.println("Obstacle ahead! Turning right.");
    }
    return;
  }
  
  // Determine which wall to follow
  if (filteredLeft < filteredRight && filteredLeft < 50) {
    // Follow left wall if it's closer
    followLeftWall();
  } else if (filteredRight < 50) {
    // Follow right wall if it's closer
    followRightWall();
  } else {
    // No wall detected nearby, go straight and look for a wall
    Serial.println("No wall detected, moving forward.");
  }
}

void followLeftWall() {
  // PID control for left wall following
  int error = TARGET_DISTANCE - filteredLeft;
  
  // Calculate integral
  integralLeft = integralLeft + error;
  integralLeft = constrain(integralLeft, -100, 100);  // Prevent integral windup
  
  // Calculate derivative
  float derivative = error - lastErrorLeft;
  
  // Calculate PID output
  float pidOutput = (Kp * error) + (Ki * integralLeft) + (Kd * derivative);
  
  // Adjust motor speeds based on PID output
  int leftSpeedWall = BASE_SPEED_WALL - pidOutput;
  int rightSpeedWall = BASE_SPEED_WALL + pidOutput;
  
  // Constrain motor speeds to valid range
  leftSpeedWall = constrain(leftSpeedWall, 0, MAX_SPEED);
  rightSpeedWall = constrain(rightSpeedWall, 0, MAX_SPEED);
  
  // Move the robot
  moveForward(leftSpeedWall, rightSpeedWall);
  
  // Store current error for next iteration
  lastErrorLeft = error;
  
  Serial.print("Following left wall | PID Output: ");
  Serial.print(pidOutput);
  Serial.print(" | Motors L/R: ");
  Serial.print(leftSpeedWall);
  Serial.print("/");
  Serial.println(rightSpeedWall);
}

void followRightWall() {
  // PID control for right wall following
  int error = TARGET_DISTANCE - filteredRight;
  
  // Calculate integral
  integralRight = integralRight + error;
  integralRight = constrain(integralRight, -100, 100);  // Prevent integral windup
  
  // Calculate derivative
  float derivative = error - lastErrorRight;
  
  // Calculate PID output
  float pidOutput = (Kp * error) + (Ki * integralRight) + (Kd * derivative);
  
  // Adjust motor speeds based on PID output
  int leftSpeedWall = BASE_SPEED_WALL + pidOutput;
  int rightSpeedWall = BASE_SPEED_WALL - pidOutput;
  
  // Constrain motor speeds to valid range
  leftSpeedWall = constrain(leftSpeedWall, 0, MAX_SPEED);
  rightSpeedWall = constrain(rightSpeedWall, 0, MAX_SPEED);
  
  // Move the robot
  moveForward(leftSpeedWall, rightSpeedWall);
  
  // Store current error for next iteration
  lastErrorRight = error;
  
  Serial.print("Following right wall | PID Output: ");
  Serial.print(pidOutput);
  Serial.print(" | Motors L/R: ");
  Serial.print(leftSpeedWall);
  Serial.print("/");
  Serial.println(rightSpeedWall);
}

// Placeholder for the third mode - if needed
void sweepMode() {
  Serial.println("SWEEP MODE");
  
    // Move forward
  moveForward(255, 255);
  delay(FORWARD_TIME);
  
  // Turn left
  turnRight();
  delay(TURN_TIME);
  
  // Move forward again
  moveForward(255, 255);
  
  // Continue moving forward and checking right sensor
  while (true) {
    float rightDistance = measureRightDistance();
    Serial.print("Right distance: ");
    Serial.println(rightDistance);
    
    // If object detected on the right, turn right
    if (rightDistance > 0 && rightDistance < MIN_DISTANCE_OBJECT_DETECTION) {
      Serial.println("Object detected! Turning right");
      
      // Stop and then turn right
      stopMotors();
      turnLeft();
      delay(RIGHT_TURN_TIME);
      
      // Stop after turning
      stopMotors();
      break;  // Exit the loop after turning right
    }
    
    delay(100);  // Short delay between sensor readings
  }
  
  // Stop after completing all movements
  stopMotors();
  
  // Pause indefinitely
  while(true) {
    delay(1000);
  }
}

// Measure distance from right ultrasonic sensor (in cm)
float measureRightDistance() {
  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_RIGHT, LOW);
  
  long duration = pulseIn(ECHO_RIGHT, HIGH, 25000); // Timeout after 25ms
  float distance = duration * 0.034 / 2;
  
  if (distance <= 0 || distance > MAX_DISTANCE_OBJECT_DETECTION) {
    return -1;  // Invalid reading
  }
  return distance;
}

// Basic movement functions
void moveForward(int leftSpeedWall, int rightSpeedWall) {
  analogWrite(motorAPin_A, leftSpeedWall);
  analogWrite(motorAPin_B, 0);
  analogWrite(motorBPin_A, rightSpeedWall);
  analogWrite(motorBPin_B, 0);
}

void moveBackward(int speed) {
  analogWrite(motorAPin_A, 0);
  analogWrite(motorAPin_B, speed);
  analogWrite(motorBPin_A, 0);
  analogWrite(motorBPin_B, speed);
}

void turnLeft() {
  analogWrite(motorAPin_A, 0);
  analogWrite(motorAPin_B, BASE_SPEED_WALL);
  analogWrite(motorBPin_A, BASE_SPEED_WALL);
  analogWrite(motorBPin_B, 0);
  delay(500);  // Adjust this value to control turning duration
}

void turnRight() {
  analogWrite(motorAPin_A, BASE_SPEED_WALL);
  analogWrite(motorAPin_B, 0);
  analogWrite(motorBPin_A, 0);
  analogWrite(motorBPin_B, BASE_SPEED_WALL);
  delay(500);  // Adjust this value to control turning duration
}

void stopMotors() {
  analogWrite(motorAPin_A, 0);
  analogWrite(motorAPin_B, 0);
  analogWrite(motorBPin_A, 0);
  analogWrite(motorBPin_B, 0);
}
