// Test code to see if encoders are functioning correctly


// define DI pins
#define encoder0PinA 2
#define encoder0PinB 3

//initialize positions
int encoder0Pos = 0;
unsigned int tmp_Pos = 1;

boolean A_set;
boolean B_set;


void setup() {

  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(encoder0PinA, doEncoderA, CHANGE);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(encoder0PinB, doEncoderB, CHANGE);

  Serial.begin (9600);
}


void loop() {
    Serial.print("Position Right: "); Serial.print(encoder0Pos);
    Serial.println();

}
// Interrupt on A changing state
void doEncoderA() {
  // Test transition
  A_set = digitalRead(encoder0PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder0Pos += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB() {
  // Test transition
  B_set = digitalRead(encoder0PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder0Pos += (A_set == B_set) ? +1 : -1;
}
