/*
 * Ultrasonic Sensor interfacing with ROS 
 * This program assumes that the speed of sound in my basement is roughly 343 m/s or 0.0343 cm/us
 * 
 * By Sam Lovett 
 * Started: 5/5/2020
 * Completed: 5/5/2020
 */

//Definition of pin numbers
const int trigPin = 11;
const int echoPin = 10;

//Definition of variables
long pulseLength;
long publishTimer;
double distance;



void setup() {
  //Set baud rate and initialize pins
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


void loop() {

  if(millis()> publishTimer){
    
    //Resets trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
  
    //Sends out ultrasonic pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);
  
    //Records the length of time of the pulse in order to calculate the distance
    pulseLength = pulseIn(echoPin, HIGH);
  
    //Calculates the distance in cm
    distance = pulseLength*0.034/2;

    Serial.print("Distance: ");
    Serial.print(pulseLength);
    Serial.println();
  

  }
  
  
}
