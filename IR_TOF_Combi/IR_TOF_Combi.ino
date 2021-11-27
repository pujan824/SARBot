#include "Adafruit_VL53L0X.h"
// This is a test code to get sensor data and see function of averaging + time delay

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

int analogPin = A0;
int a2 = A1;
int Gpio = 23;
int val= 0;
int val2 = 0; 
int distance, distance2;  
int SENSOR_SAMPLES = 10;

void setup() {
  pinMode(Gpio, OUTPUT);
  digitalWrite(Gpio, HIGH);
  Serial.begin(9600); //  setup serial
  
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
  
}

void loop() {
  val = analogRead(analogPin);
  val2 = analogRead(a2);
  double distance = -0.1005*(val) + 72.866;// read the input pin
  double distance2 = -0.1005*(val2) + 72.866;
  Serial.print(distance2);Serial.print(" ");
  Serial.println(distance);

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  
  // ReadSensor(distance);
  // debug value
}


/* float ReadSensor (int analogPin){
  unsigned int sensor_sum = 0;
  for (int i = 0;i < SENSOR_SAMPLES; ++i)
  {
    sensor_sum += analogRead(analogPin);
  }
  return (sensor_sum/SENSOR_SAMPLES)*(-0.1)+73;
}
*/
