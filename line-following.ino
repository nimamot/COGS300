// Motor control pins
#define LEFT_FORWARD 8    // Controls left motor forward
#define LEFT_BACKWARD 9   // Controls left motor backward
#define RIGHT_FORWARD 7   // Controls right motor forward
#define RIGHT_BACKWARD 6  // Controls right motor backward
// Ultrasonic sensor pins
#define TRIG_FRONT 13     // Front ultrasonic trigger
#define ECHO_FRONT 10     // Front ultrasonic echo
#define TRIG_LEFT 2       // Left ultrasonic trigger
#define ECHO_LEFT 3       // Left ultrasonic echo
#define TRIG_RIGHT 12     // Right ultrasonic trigger
#define ECHO_RIGHT 5      // Right ultrasonic echo
// IR sensor pins
#define L_IR A2           // Left IR sensor
#define M_IR A1           // Middle IR sensor
#define R_IR A0           // Right IR sensor

// Motor speed adjustment parameters
#define BASE_SPEED 200           // Base PWM value (0-255)
#define RIGHT_COMPENSATION 30    // Additional power to right motor to compensate for left lean
#define CORRECTION_FACTOR 0.8    // How strongly to correct based on sensor readings

// Variables for self-correction
int leftSpeed;
int rightSpeed;
unsigned long previousMillis = 0;
const long adjustInterval = 100;  // Adjust every 100ms

void setup() {
  // Set motor control pins as outputs
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  
  // Set ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Set IR sensor pins as inputs
  pinMode(L_IR, INPUT);
  pinMode(M_IR, INPUT);
  pinMode(R_IR, INPUT);
  
  // Initialize motor speeds with compensation
  leftSpeed = BASE_SPEED;
  rightSpeed = BASE_SPEED + RIGHT_COMPENSATION;
  
  Serial.begin(9600);  // Start serial for debugging
  delay(2000);         // Startup delay
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Drive forward with periodic adjustments
  driveForward(leftSpeed, rightSpeed);
  
  // Adjust direction at regular intervals
  if (currentMillis - previousMillis >= adjustInterval) {
    previousMillis = currentMillis;
    adjustDirection();
  }
}

void driveForward(int leftPWM, int rightPWM) {
  // Set motors to move forward
  analogWrite(LEFT_FORWARD, leftPWM);
  digitalWrite(LEFT_BACKWARD, LOW);
  analogWrite(RIGHT_FORWARD, rightPWM);
  digitalWrite(RIGHT_BACKWARD, LOW);
}

void stopMotors() {
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, LOW);
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, LOW);
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
  Serial.print(leftSpeed);
  Serial.print(", R Speed: ");
  Serial.println(rightSpeed);
  
  // Only adjust if both sensors are detecting something (within 200cm)
  if (leftDistance < 200 && rightDistance < 200) {
    int adjustment = (int)((rightDistance - leftDistance) * CORRECTION_FACTOR);
    
    // Apply adjustments to motor speeds
    leftSpeed = constrain(BASE_SPEED + adjustment, BASE_SPEED - 50, BASE_SPEED + 50);
    rightSpeed = constrain(BASE_SPEED + RIGHT_COMPENSATION - adjustment, 
                          BASE_SPEED + RIGHT_COMPENSATION - 50, 
                          BASE_SPEED + RIGHT_COMPENSATION + 50);
  } else {
    // Reset to base values with compensation if no valid readings
    leftSpeed = BASE_SPEED;
    rightSpeed = BASE_SPEED + RIGHT_COMPENSATION;
  }
}
