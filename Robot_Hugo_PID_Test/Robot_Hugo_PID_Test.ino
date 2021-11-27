#include <AutoPID.h>
#include "CytronMotorDriver.h"

int analogLeft = A0;
int analogRight = A1;
int analogFront = A2;
int Gpio = 11;  

//encoder
#define encoder0PinA 22
#define encoder0PinB 23
#define encoder1PinA 2
#define encoder1PinB 3

volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;

boolean A0_set;
boolean B0_set;
boolean A1_set;
boolean B1_set;

CytronMD motorLeft(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motorRight(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

int segment = 1;

int frontThreshold = 12;
int leftThreshold = 20;
int rightThreshold = 20;

double top_kP = 5;
double bottom_kP = 4;
int top_dist = 8;
int bottom_dist = 7;

/*
 * ******************************SMOOTHING************************************************************
 */

 const int numReadings = 5;
 
 double readingsFront[numReadings];
 double readingsLeft[numReadings];
 double readingsRight[numReadings];
 
 int readIndex = 0;
 
 double totalFront = 0;
 double totalLeft = 0;
 double totalRight = 0;
  
 double averageFront = 0;
 double averageLeft = 0;
 double averageRight = 0; 
 /*
  * ***************************************************************************************************
  */

/*
 * **************************PID***************************************************************
 */

 #define OUTPUT_MIN 0
 #define OUTPUT_MAX 255
 #define KP 5
 #define KI 0
 #define KD 0.5

 double setpoint, outputVal;

 AutoPID forwardLeftWallFollow(&averageLeft, &setpoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
 AutoPID forwardRightWallFollow(&averageRight, &setpoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
/*
 * *****************************************************************************************************
 */

void setup() {
  pinMode(Gpio, OUTPUT);
  digitalWrite(Gpio, HIGH);
  
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);

  attachInterrupt(encoder0PinA, doEncoderA0, CHANGE);
  attachInterrupt(encoder0PinB, doEncoderB0, CHANGE);
  attachInterrupt(encoder1PinA, doEncoderA1, CHANGE);
  attachInterrupt(encoder1PinB, doEncoderB1, CHANGE);

  for(int x =0; x <numReadings; x++){
    readingsFront[x] = 0;
  }
  
  Serial.begin(9600); //  setup serial

}

void loop() {
  updateSensors();
  switch (segment) {
  
    case 1:
      forwardLeftWallFollow.run();
      if(averageFront > frontThreshold){
        Forward(top_kP, top_dist);
      }
        else{
          Stop();
          delay(100);
        }
      break;
    
    default:
      updateSensors();
      // if nothing else matches, do the default
      // default is optional
      Serial.println(segment);
  }

}

void updateSensors(){

  totalFront = totalFront - readingsFront[readIndex];
  totalLeft = totalLeft - readingsLeft[readIndex];
  totalRight = totalRight - readingsRight[readIndex];
  
  //distanceLeft = -0.1005*(analogRead(analogLeft)) + 72.866;// read the input pin
  //distanceRight = -0.1005*(analogRead(analogRight)) + 72.866;
  readingsFront[readIndex] = -0.1005*(analogRead(analogFront)) + 72.866;
  readingsLeft[readIndex] = -0.1005*(analogRead(analogLeft)) + 72.866;
  readingsRight[readIndex] = -0.1005*(analogRead(analogRight)) + 72.866;

  totalFront = totalFront + readingsFront[readIndex];
  totalLeft = totalLeft + readingsLeft[readIndex];
  totalRight = totalRight + readingsRight[readIndex];
  
  readIndex++;

  if(readIndex >= numReadings){
    readIndex = 0;
  }
  averageFront = totalFront / numReadings;
  averageLeft = totalLeft / numReadings;
  averageRight = totalRight / numReadings;

  Serial.print("Left "); Serial.print(averageLeft); Serial.print(" ");
  Serial.print("Front "); Serial.print(averageFront); Serial.print(" ");
  Serial.print("Right "); Serial.print(averageRight); Serial.print(" ");
  Serial.print("Left Enc "); Serial.print(encoder0Pos);
  Serial.print(" Right Enc "); Serial.print(encoder1Pos);
  
  Serial.println(" ");


}

void Forward(double kP, int dist){
  // create speed values to send to motor driver. Uses speed of 30 as base value and adjusts by error using P gain.
  Serial.print(outputVal);
  
  float left = 100;
  float right = -100;
  motorLeft.setSpeed(left);
  motorRight.setSpeed(right);
}

void Stop(){
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void doEncoderA0() {
  // Test transition
  A0_set = digitalRead(encoder0PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder0Pos += (A0_set != B0_set) ? +1 : -1;
}
void doEncoderB0() {
  // Test transition
  B0_set = digitalRead(encoder0PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder0Pos += (A0_set == B0_set) ? +1 : -1;
}
void doEncoderA1() {
  // Test transition
  A1_set = digitalRead(encoder1PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
}
void doEncoderB1() {
  // Test transition
  B1_set = digitalRead(encoder1PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder1Pos += (A1_set == B1_set) ? +1 : -1;
}
