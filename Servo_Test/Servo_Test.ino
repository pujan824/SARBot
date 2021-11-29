#include <Servo.h>          //Servo library
 
Servo servo_arm;        //initialize a servo object for the connected servo  
Servo servo_grab;
                
int angle = 0;
 
void setup() 
{ 
  servo_arm.attach(13);      // attach the signal pin of servo to pin9 of arduino
  servo_grab.attach(12);
  Serial.begin(9600);
} 
  
void loop() 
{ 
  Serial.print(angle);
  for(angle = 0; angle < 180; angle += 1)    // command to move from 0 degrees to 180 degrees 
  {                                  
    servo_arm.write(angle); //command to rotate the servo to the specified angle
    servo_grab.write(angle);
    delay(5);      
    Serial.println(angle);                 
  } 
 
  delay(1000);
  
  for(angle = 180; angle>=1; angle-=5)     // command to move from 180 degrees to 0 degrees 
  {                                
    servo_arm.write(angle);              //command to rotate the servo to the specified angle
    servo_grab.write(angle);
    delay(5);         
    Serial.println(angle);              
  } 

    delay(1000);
}
