
#include "CytronMotorDriver.h"

// define encoder pins
#define encoder0PinA 8
#define encoder0PinB 9
#define encoder1PinA 10
#define encoder1PinB 11

//IR Sensor 
#define rightIR A0
#define leftIR A2
#define frontIR A1
int rightIRGPIO = 2;
float rightDistance, leftDistance, frontDistance = 0;

//set-up encoder positions
int encoder0Pos = 0;
int encoder1Pos = 0;
unsigned int tmp_Pos = 1;

//global PID gians
double kP = 0.1;
double kI = 0;
double kD = 0;

// setup PID values
double errorENCRight, errorENCLeft;
double lastErrorRight, lastErrorLeft;
double cumErrorRIGHT, cumErrorLEFT; 
double rateErrorRIGHT, rateErrorLEFT;
boolean atTargetRight, atTargetLeft = false;

boolean A0_set;
boolean B0_set;

boolean A1_set;
boolean B1_set;

//setup motors
CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

//Path Planning
int Path[4] = {10, 0, 10, 10};
int stepCur = 0;


void setup() {
//setup encoder pins as inputs
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);

  // encoder pin on interrupt 8 (pin 8)
  attachInterrupt(8, doEncoder0A, CHANGE);
  // encoder pin on interrupt 9 (pin 9)
  attachInterrupt(9, doEncoder0B, CHANGE);
    // encoder pin on interrupt 10 (pin 10)
  attachInterrupt(10, doEncoder1A, CHANGE);
  // encoder pin on interrupt 11 (pin 11)
  attachInterrupt(11, doEncoder1B, CHANGE);

//IR sensor set enable
  digitalWrite(2, HIGH);
  Serial.begin (9600);
}


void loop() {
  rightDistance = analogRead(rightIR)*(-0.1005)+72.866;
  leftDistance = analogRead(leftIR)*(-0.1005)+72.866;
  frontDistance = analogRead(frontIR)*(-0.1005)+72.866;
 // Serial.print("Position Right: "); Serial.print(encoder0Pos, DEC); Serial.print("Position Left: "); Serial.print(encoder1Pos, DEC);
   Serial.print("Right IR"); Serial.print(rightDistance);Serial.print(" LeftIR"); Serial.print(leftDistance);Serial.print(" Front IR"); Serial.print(frontDistance);
   Serial.println();
  // Serial.print(stepCur);
  //Serial.print(centerDifference());


  if(stepCur > 3){
    Stop();
  }else{
    if(Path[stepCur] == 0){
      Serial.print("Turn Left");
      turnLeftNoSense();
      delay(1000);
      stepCur++;
    }
    else if (Path[stepCur] == 1){
      Serial.print("Turn Right");
      turnRightNoSense();
      delay(1000);
      stepCur++;
    }
    else{
      driveStraightDistance(Path[stepCur]);
    }
  }
  

 // driveStraight();
  
}

//simple right turn
void turnRightNoSense(){
  
  motor1.setSpeed(-10);
  motor2.setSpeed(50);
  
}

// simple left turn
void turnLeftNoSense(){
  
  motor1.setSpeed(50);
  motor2.setSpeed(-10);
  
}


// pid left turn
void turnLeft(){

  int targetRight = -2300;
  int targetLeft = 2300;
  
  errorENCRight = targetRight - encoder0Pos;
  errorENCLeft = targetLeft - encoder1Pos;

  cumErrorRIGHT += errorENCRight;
  cumErrorLEFT += errorENCLeft;

  rateErrorRIGHT = (errorENCRight - lastErrorRight);
  rateErrorLEFT = (errorENCLeft - lastErrorLeft);
  
  double right = ((kP*errorENCRight) + (kI*cumErrorRIGHT) + (kD*rateErrorRIGHT));
  double left = ((kP*errorENCLeft)+(kI*cumErrorLEFT)+(kD*rateErrorLEFT));

  double rightOut = constrain(right, -20, 20);
  double leftOut = constrain(left, -20, 20);
  Serial.print("Right"); Serial.print(rightOut);
  Serial.print("Left"); Serial.print(-leftOut);

//checks if it is withing tolerance to finish command
  if(abs(errorENCRight) <=100){
    motor1.setSpeed(0); 
    atTargetRight = true;
  }else{
    motor1.setSpeed(rightOut); 
  }
  if(abs(errorENCLeft) <=100){
    motor2.setSpeed(0); 
    atTargetLeft = true;
  }else{
    motor2.setSpeed(-leftOut); 
  }

  if(atTargetRight and atTargetLeft){
    stepCur ++;
    cumErrorLEFT = 0;
    cumErrorRIGHT = 0;
    encoder0Pos = 0;
    encoder1Pos = 0;
    rateErrorRIGHT = 0;
    rateErrorLEFT = 0;
  }
  
}
// PID drive straight
void driveStraight(){
  if(rightDistance > 14){
    rightDistance = 7;
  }
  
  double right = 30 - ((rightDistance - 7)*2); //uses error from right IR to find midpoint of maze
  double left = -30 - ((rightDistance - 7)*2);
  motor1.setSpeed(right);
  motor2.setSpeed(left);

  Serial.print("Right"); Serial.print(right);
  Serial.print("Left"); Serial.print(left);
}

// uses PID to drive straight for a distance
void driveStraightDistance(double setpoint){
  double target = setpoint/(2*3.14*3)*210*12*2;
  if(rightDistance > 14){
    rightDistance = 7;
  }
  
  errorENCRight = target - encoder0Pos;
  errorENCLeft = target - encoder1Pos;

  cumErrorRIGHT += errorENCRight;
  cumErrorLEFT += errorENCLeft;

  rateErrorRIGHT = (errorENCRight - lastErrorRight);
  rateErrorLEFT = (errorENCLeft - lastErrorLeft);
  
  double right = ((kP*errorENCRight) + (kI*cumErrorRIGHT) + (kD*rateErrorRIGHT));
  double left = ((kP*errorENCLeft)+(kI*cumErrorLEFT)+(kD*rateErrorLEFT));

  double rightOut = constrain(right, -30, 30);
  double leftOut = constrain(left, -30, 30);
  Serial.print("Right"); Serial.print(rightOut-((rightDistance - 7)*0.5));
  Serial.print("Left"); Serial.print(-leftOut-((rightDistance - 7)*0.5));

//checks if position is in tolerance to finish command
  if(abs(errorENCRight) <=100){
    motor1.setSpeed(0); 
    atTargetRight = true;
  }else{
    motor1.setSpeed(rightOut - ((rightDistance - 7)*0.5)); 
  }
  if(abs(errorENCLeft) <=100){
    motor2.setSpeed(0); 
    atTargetLeft = true;
  }else{
    motor2.setSpeed(-leftOut - ((rightDistance - 7)*0.5)); 
  }

  if(atTargetRight and atTargetLeft){
    stepCur ++;
    cumErrorLEFT = 0;
    cumErrorRIGHT = 0;
    encoder0Pos = 0;
    encoder1Pos = 0;
    rateErrorRIGHT = 0;
    rateErrorLEFT = 0;
  }
  
  /*
  if(rightDistance > 14){
    rightDistance = 8;
  }
  
  double right = 30 - ((rightDistance - 8)*2);
  double left = -30 - ((rightDistance - 8)*2);
  motor1.setSpeed(right);
  motor2.setSpeed(left);

  Serial.print("Right"); Serial.print(right);
  Serial.print("Left"); Serial.print(left);
  */
}

//stops motors
void Stop(){
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

// Interrupt on A changing state
void doEncoder0A() {
  // Test transition
  A0_set = digitalRead(encoder0PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder0Pos += (A0_set != B0_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoder0B() {
  // Test transition
  B0_set = digitalRead(encoder0PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder0Pos += (A0_set == B0_set) ? +1 : -1;
}

void doEncoder1A() {
  // Test transition
  A1_set = digitalRead(encoder1PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoder1B() {
  // Test transition
  B1_set = digitalRead(encoder1PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder1Pos += (A1_set == B1_set) ? +1 : -1;
}

/*
void DriveStraight_PID(double setpoint){

  errorENCRight = setpoint - encoder0Pos;
  errorENCLeft = setpoint - encoder1Pos;

  cumErrorRIGHT += errorENCRight;
  cumErrorLEFT += errorENCLeft;

  rateErrorRIGHT = (errorENCRight - lastErrorRight);
  rateErrorLEFT = (errorENCLeft - lastErrorLeft);
  
  double right = (kP*errorENCRight) + (kI*cumErrorRIGHT) + (kD*rateErrorRIGHT);
  double left = (kP*errorENCLeft)+(kI*cumErrorLEFT)+(kD*rateErrorLEFT);

  double rightOut = constrain(right, -100, 100);
  double leftOut = constrain(left, -100, 100);

  if(abs(errorENCRight) <=10){
    motor1.setSpeed(0); 
    atTargetRight = true;
  }else{
    motor1.setSpeed(-rightOut); 
  }
  if(abs(errorENCLeft) <=10){
    motor2.setSpeed(0); 
    atTargetLeft = true;
  }else{
    motor2.setSpeed(leftOut); 
  }

  Serial.print(errorENCRight);
  Serial.println();
  Serial.print(lastErrorRight);
  Serial.println();
  Serial.print(cumErrorRIGHT);
  Serial.println();
  Serial.print(rateErrorRIGHT);
  Serial.println();
  Serial.print(rightOut);
  Serial.println();

  lastErrorRight = errorENCRight;
  lastErrorLeft = errorENCLeft;
  
}
*/
