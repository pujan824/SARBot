/*
 * ********************************************************************************Libraries********************************************************
 */
#include "CytronMotorDriver.h"
#include <Servo.h> 
/*
 * **************************************************************************************************************************************************
 */
 
/*
 * ********************************* IR Sensor ******************************************************************************************************
 */
int analogLeft = A3;
int analogRight = A1;
int analogFront = A2;
int analogObject = A0;

 // was having power issues from 3.3V out so switch to using DO pins for 3.3V out instead of spliced cable
#define IR0_Power 53
#define IR1_Power 51
#define IR2_Power 49
#define IR3_Power 47
#define IR4_Power 45

#define IR0_EN 52
#define IR1_EN 50
#define IR2_EN 48
#define IR3_EN 46
#define IR4_EN 44
/*
 * **************************************************************************************************************************************************
 */

/*
 * **********************************************************Encoders********************************************************************************
 */
#define encoder0PinA 22
#define encoder0PinB 23
#define encoder1PinA 24
#define encoder1PinB 25

volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;

boolean A0_set;
boolean B0_set;
boolean A1_set;
boolean B1_set;

/*
 * **************************************************************************************************************************************************
 */

/*
 * *************************************************Motor Objects ***********************************************************************************
 */

CytronMD motorLeft(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motorRight(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

/*
 * **************************************************************************************************************************************************
 */

 /*
  * ***************************************************************Maze Segments*********************************************************************
  */
int segment = 1;

int frontThreshold = 14;
int leftThreshold = 20;
int rightThreshold = 30;

double top_kP = 5;
double bottom_kP = 4;
int top_dist = 9;
int bottom_dist = 7;
/*
 * **************************************************************************************************************************************************
 */

/*
 * ******************************SMOOTHING************************************************************
 */

 const int numReadings = 5;
 
 double readingsFront[numReadings];
 double readingsLeft[numReadings];
 double readingsRight[numReadings];
 double readingsObject[numReadings];
 
 int readIndex = 0;
 
 double totalFront = 0;
 double totalLeft = 0;
 double totalRight = 0;
 double totalObject = 0;
  
 double averageFront = 0;
 double averageLeft = 0;
 double averageRight = 0; 
 double averageObject = 0; 
 /*
  * ***************************************************************************************************
  */

/*
 * **************************PID***************************************************************
 */

 #define OUTPUT_MIN -150
 #define OUTPUT_MAX 150
double KP = 3.1;
double KI = 0;
double KD = 0.2;

float error = 0;
float error_sum = 0;
float error_change = 0;
float prev_error = 0;

bool start = true;

 double setpoint, outputVal;

/*
 * *****************************************************************************************************
 */

 /*
  * ********************************************SERVO*********************************************
  */
  Servo servo_arm;
  Servo servo_grab;
  bool objectFound = false;

void setup() {

  // set all IR sensors to output HIGH for Vin and enable
  pinMode(IR0_Power, OUTPUT);
  digitalWrite(IR0_Power, HIGH);
  pinMode(IR0_EN, OUTPUT);
  digitalWrite(IR0_EN, HIGH);
  pinMode(IR1_Power, OUTPUT);
  digitalWrite(IR1_Power, HIGH);
  pinMode(IR1_EN, OUTPUT);
  digitalWrite(IR1_EN, HIGH);
  pinMode(IR2_Power, OUTPUT);
  digitalWrite(IR2_Power, HIGH);
  pinMode(IR2_EN, OUTPUT);
  digitalWrite(IR2_EN, HIGH);
  pinMode(IR3_Power, OUTPUT);
  digitalWrite(IR3_Power, HIGH);
  pinMode(IR3_EN, OUTPUT);
  digitalWrite(IR3_EN, HIGH);
  pinMode(IR4_Power, OUTPUT);
  digitalWrite(IR4_Power, HIGH);
  pinMode(IR4_EN, OUTPUT);
  digitalWrite(IR4_EN, HIGH);


  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);

  attachInterrupt(encoder0PinA, doEncoderA0, CHANGE);
  attachInterrupt(encoder0PinB, doEncoderB0, CHANGE);
  attachInterrupt(encoder1PinA, doEncoderA1, CHANGE);
  attachInterrupt(encoder1PinB, doEncoderB1, CHANGE);

  //initialize smoothing algortithm with all 0

  for(int x =0; x <numReadings; x++){
    readingsFront[x] = 0;
    readingsLeft[x] = 0;
    readingsRight[x] = 0;
    readingsObject[x] = 0;
  }

  servo_arm.attach(13);
  servo_grab.attach(12);

  servo_arm.write(115);
  delay(300);
  
  Serial.begin(9600); //  setup serial
  

}

void loop() {
  updateSensors();
  Serial.println(segment);
  switch (segment) {
  
    case 1:

    // check if there is a wall in front
      if(averageFront > frontThreshold){
        // uses object sensing to see if there is an obsticle. If found, throw it behind, otherwise go straight
        if(objectFound){
          Stop();
          yeet();
          objectFound = false;
          delay(2000);
        }
        else{
          ForwardLeftWall(top_dist);
        }
      }
        else{
          // predefined function to end segment when wall in front is detected
          finishSegment();
        }
      break;
      
    case 2:
    //command for right turn
      turnRight90(100);
      finishSegment();
      
      break;

    case 3:
      if(averageFront > frontThreshold){
        if(objectFound){
          Stop();
          yeet();
          objectFound = false;
          delay(2000);
        }
        else{
          ForwardLeftWall(top_dist);
        }
      }
        else{
          finishSegment();
        }
      break;
    case 4:
      turnRight90(0);
      finishSegment();
      
      break;

    case 5:
      if(averageFront > frontThreshold){
          ForwardLeftWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;

    case 6:
      turnRight90(0);
      finishSegment();
      
      break;

    case 7:
      if(averageFront > frontThreshold){
          ForwardLeftWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;

    case 8:
      turnRight90(0);
      finishSegment();
      
      break;

    case 9:
      if(averageRight < rightThreshold or ((encoder0Pos + encoder1Pos)/2) < 13000){
          ForwardLeftWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;

    case 10:
      if((encoder0Pos + encoder1Pos)/2 < 2000 or averageRight<rightThreshold){
          ForwardLeftWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;
      
    case 11:
      turnRight90(0);
      finishSegment();
      
      break;

    case 12:
      if(averageFront > frontThreshold){
          ForwardLeftWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;
      
    case 13:
      turnRight90(0);
      finishSegment();
      
      break;

    case 14:
      if(averageFront > frontThreshold){
          ForwardRightWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;

    case 15:
      turnLeft90(0);
      finishSegment();
      
      break;

    case 16:
      if(averageFront > (frontThreshold-2)){
          ForwardRightWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;

    case 17:
      turnLeft90(0);
      finishSegment();
      
      break;

    case 18:
      if(averageFront > frontThreshold){
          ForwardRightWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;

    case 19:
      turnLeft90(0);
      finishSegment();
      
      break;

    case 20:
      if(averageLeft < leftThreshold or ((encoder0Pos + encoder1Pos)/2) < 4000){
          ForwardRightWall(top_dist);
      }
        else{
          finishSegment();
        }
      break;

    case 21:
      turnLeft90(0);
      finishSegment();
      
      break;

    case 22:
      if( (not objectFound) or ((encoder0Pos + encoder1Pos)/2) < 7900){
          ForwardRightWall(top_dist);
      }
        else{
          Stop();
          holdObject();
          finishSegment();
        }
      break;

      
    default:
      updateSensors();
      Serial.println(segment);
      // if nothing else matches, do the default
      // default is optional
  }

}

void updateSensors(){

  // uses smoothing algorithm to average readings and eliminate spikes in readings. 
  totalFront = totalFront - readingsFront[readIndex];
  totalLeft = totalLeft - readingsLeft[readIndex];
  totalRight = totalRight - readingsRight[readIndex];
  totalObject = totalObject - readingsObject[readIndex];
  
  //distanceLeft = -0.1005*(analogRead(analogLeft)) + 72.866;// read the input pin
  //distanceRight = -0.1005*(analogRead(analogRight)) + 72.866;
  readingsFront[readIndex] = -0.1005*(analogRead(analogFront)) + 72.866;
  readingsLeft[readIndex] = -0.1005*(analogRead(analogLeft)) + 72.866;
  readingsRight[readIndex] = -0.1005*(analogRead(analogRight)) + 72.866;
  readingsObject[readIndex] = -0.1005*(analogRead(analogObject)) + 72.866;

  totalFront = totalFront + readingsFront[readIndex];
  totalLeft = totalLeft + readingsLeft[readIndex];
  totalRight = totalRight + readingsRight[readIndex];
  totalObject = totalObject + readingsObject[readIndex];
  
  readIndex++;

  if(readIndex >= numReadings){
    readIndex = 0;
  }
  averageFront = totalFront / numReadings;
  averageLeft = totalLeft / numReadings;
  averageRight = totalRight / numReadings;
  averageObject = totalObject / numReadings;

//  averageLeft = -0.1005*(analogRead(analogLeft)) + 72.866;// read the input pin
//  averageRight = -0.1005*(analogRead(analogRight)) + 72.866;
//  averageFront = -0.1005*(analogRead(analogFront)) + 72.866;
//  averageObject = -0.1005*(analogRead(analogObject)) + 72.866;

  if(averageObject < 10.5){
    objectFound = true;
  }


  Serial.print("Left Distance: "); Serial.print(averageLeft); Serial.print(" ");
  Serial.print("Front "); Serial.print(averageFront); Serial.print(" ");
  Serial.print("Right "); Serial.print(averageRight); Serial.print(" ");
  Serial.print("Object "); Serial.print(averageObject); Serial.print(" ");
  Serial.print("Left Enc "); Serial.print(encoder0Pos);
  Serial.print(" Right Enc "); Serial.print(encoder1Pos);
  Serial.println(" ");


}

void ForwardLeftWall(int setpoint){
  // update sensors again to get most recent results
  updateSensors();

  //temperary value to store distance so we don't change original variable
  float templeft;

  //if the value is above 65, it likely is in a deadzone and wall is close (0 distance)
  if (averageLeft >= 65){
    templeft = 0;
  }
  //if it is greater than 20 there is likely a hole in the wall and it should be ignored
  else if (averageLeft >= 20){
    templeft = setpoint;
  }
  // all other values are valid
  else{
    templeft = averageLeft;
  }

  // calculate PID errors
  error = templeft - setpoint;
  error_sum += error;

  outputVal = KP*error + KI*error_sum + KD*(error-prev_error);
  //Serial.print("Output Value");Serial.print(outputVal);
  
  // constrain the output to a max value to not go too fast
  float left = 100 - outputVal;
  left = constrain(left, OUTPUT_MIN, OUTPUT_MAX);
  float right = -100 - outputVal;
  right = constrain(right, OUTPUT_MIN, OUTPUT_MAX);

  
  //Serial.print(" LEFT SPEED: ");Serial.print(left);Serial.print(" ");
  //Serial.print(" RIGHT SPEED: ");Serial.print(right);Serial.print(" ");
  // set motor speeds and update previous error
  motorLeft.setSpeed(left);
  motorRight.setSpeed(right);
  prev_error = error;
}

void ForwardRightWall(int setpoint){.
  updateSensors();
// same as left wall follow
  float tempright;

  if (averageRight >= 65){
    tempright = setpoint;
  }
  else if (averageRight >=71){
    tempright = 0;
  }
  else if (averageRight >= 20){
    tempright = setpoint;
  }
  else{
    tempright = averageLeft;
  }
  error = tempright - setpoint;
  error_sum += error;

  outputVal = KP*error + KI*error_sum + KD*(error-prev_error);
  //Serial.print("Output Value");Serial.print(outputVal);
  
  
  float left = 100 - outputVal;
  left = constrain(left, OUTPUT_MIN, OUTPUT_MAX);
  float right = -100 - outputVal;
  right = constrain(right, OUTPUT_MIN, OUTPUT_MAX);

  
  Serial.print(" LEFT SPEED: ");Serial.print(left);Serial.print(" ");
  Serial.print(" RIGHT SPEED: ");Serial.print(right);Serial.print(" ");
  motorLeft.setSpeed(-right);
  motorRight.setSpeed(-left);
  prev_error = error;
   uses temp value for current distance to left wall to not change actual read value

  
//  float tempRight;
//  // if distance is greater than 14, there is no wall. Set actual to target so no aadjustments are made if robot continues forwards
//  if(averageRight > 50){ 
//    tempRight = 0;
//  }
//  else if( averageRight > 20){
//    tempRight = 8;
//  }
//  else{
//    tempRight = averageRight;
//  }
//
//  error = 8 - tempRight;
//
//  // create speed values to send to motor driver. Uses speed of 30 as base value and adjusts by error using P gain.
//  float left = 100 - (((error)*3)+((error-prev_error)*0.2));
//  float right = -100 - (((error)*3)+ ((error-prev_error)*0.2));
//  motorLeft.setSpeed(left);
//  motorRight.setSpeed(right);
//  Serial.print(left);
//  Serial.print(right);
//  prev_error = error; 
}

void Stop(){
  // stops moving
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void finishSegment(){
  // finish each segment with updating counter and reset encoder count
  Stop();
  delay(100);
  segment++;
  resetEncoders();
}

void resetEncoders(){
  // set count to 0
  encoder0Pos = 0;
  encoder1Pos = 0;
}

void turnRight90(int offset){
  // move left wheel at constant speed until reaches angle
  resetEncoders();
  while(encoder0Pos < 4000 + offset){
    motorLeft.setSpeed(100);
    motorRight.setSpeed(0);  
    Serial.print("Left Enc "); Serial.print(encoder0Pos);
    Serial.print(" Right Enc "); Serial.print(encoder1Pos);
    Serial.println();
  }
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    delay(200);
}

void turnLeft90(int offset){
  // moves right wheel until reaches angle
  resetEncoders();
  while(encoder1Pos < 4000 + offset){
    motorLeft.setSpeed(0);
    motorRight.setSpeed(-75);  
    Serial.print("Left Enc "); Serial.print(encoder0Pos);
    Serial.print(" Right Enc "); Serial.print(encoder1Pos);
    Serial.println();
  }
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0);
    delay(200);
}

void grabReady(){
  // moves arm into grab position
  servo_grab.write(180);
  delay(200);
  servo_arm.write(200);
  delay(700);
}

void grab(){
  // grabs the object
  servo_grab.write(10);
  delay(500);
  servo_arm.write(10);
  delay(1000);
}

void holdObject(){
  // combines functions to hust hold the object in front 
  grabReady();
  //driveDistanceLeftWall(1000);
  grab();
}

void yeet(){
  // holds object then throws from back
  holdObject();
  servo_grab.write(180);
  delay(1000);
  servo_arm.write(110);
  delay(300);
}

void resetGrabber(){
  // reset to tipping point
  servo_arm.write(90);
  delay(100);
  servo_grab.write(90);
  delay(100);
  updateSensors();
}

//encoder isr
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
