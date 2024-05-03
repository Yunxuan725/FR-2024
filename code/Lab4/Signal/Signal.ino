int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;
float last_time;

void setup() {
  Serial.begin(115200);
  pinMode(motorR1, OUTPUT);
  analogWrite(motorR1, 0); 
  pinMode(motorR2, OUTPUT);
  analogWrite(motorR2, 0);
  pinMode(motorL1, OUTPUT);
  analogWrite(motorL1, 0);
  pinMode(motorL2, OUTPUT);
  analogWrite(motorL2, 0);
  last_time = micros();
}
void loop() {
  //analogWrite(motorL1,127); 
  //analogWrite(motorL2,127); 
 // analogWrite(motorR1,127); 
  analogWrite(motorR2,127); 
  Serial.println((micros()-last_time)/1000);
  last_time = micros();
}