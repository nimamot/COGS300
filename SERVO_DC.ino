const int motorIN1 = 9; 
const int motorIN2 = 10; 
const int potPin = A0;  

int targetAngle = -1;  
int currentAngle = 0;   
int lastPotValue = 0;  
const int stepSize = 1; 
const int moveTime = 20; 
const int motorSpeed = 138; 

void setup() {
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  Serial.begin(9600);
  Serial.println("Enter a target angle (0-180) in Serial Monitor.");
  analogWrite(motorIN1, 0);
  analogWrite(motorIN2, 0);
  lastPotValue = analogRead(potPin); 
}

void loop() {
  if (Serial.available()) {
    int inputAngle = Serial.parseInt(); 
        if (inputAngle >= 0 && inputAngle <= 180) {
      targetAngle = inputAngle;
      Serial.print("Target Angle Set: ");
      Serial.println(targetAngle);
      Serial.println("Now turn the potentiometer to move the motor.");
    } else {
      Serial.println("Invalid input! Enter a number between 0 and 180.");
    }
  }
  int potValue = analogRead(potPin);

  currentAngle = map(potValue, 0, 1023, 0, 180);  

  if (abs(potValue - lastPotValue) > 5) { 

    Serial.print("Current Angle: ");
    Serial.println(currentAngle);

    if (currentAngle < targetAngle - stepSize) {  
      moveMotorOneStep(true);  // Move forward
    } 
    else if (currentAngle > targetAngle + stepSize) {  
      moveMotorOneStep(false); // Move backward
    } 
    else {  
      analogWrite(motorIN1, 0);
      analogWrite(motorIN2, 0);
      Serial.println(" Target Angle Reached! ");
      targetAngle = -1;  
    }

    lastPotValue = potValue;  
  }

  delay(50);
}

void moveMotorOneStep(bool forward) {
  if (forward) {
    analogWrite(motorIN1, motorSpeed);
    analogWrite(motorIN2, 0);
  } 
  else {
    analogWrite(motorIN1, 0);
    analogWrite(motorIN2, motorSpeed);
  }

  delay(moveTime); 
  analogWrite(motorIN1, 0);
  analogWrite(motorIN2, 0);
}