#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SHUTDOWN_PIN1 4
#define SHUTDOWN_PIN2 13 

SFEVL53L1X distanceSensorF(Wire, SHUTDOWN_PIN1); // Front sensor
SFEVL53L1X distanceSensorS(Wire, SHUTDOWN_PIN2); // Side sensor
int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;

void setup(void) {
  Serial.begin(115200);
  pinMode(motorR1, OUTPUT);
  analogWrite(motorR1, 0); 
  pinMode(motorR2, OUTPUT);
  analogWrite(motorR2, 0);
  pinMode(motorL1, OUTPUT);
  analogWrite(motorL1, 0);
  pinMode(motorL2, OUTPUT);
  analogWrite(motorL2, 0);
  pinMode(SHUTDOWN_PIN1, OUTPUT);
  digitalWrite(SHUTDOWN_PIN1, LOW); // Ensure first sensor is off
  pinMode(SHUTDOWN_PIN2, OUTPUT);
  digitalWrite(SHUTDOWN_PIN2, LOW);
  delay(10);
  Wire.begin();
  digitalWrite(SHUTDOWN_PIN1, HIGH);
  delay(10);
  distanceSensorF.setI2CAddress(0x32); // New I2C address for the front sensor
  if (distanceSensorF.begin() != 0) {
    Serial.println("Front sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }

  digitalWrite(SHUTDOWN_PIN2, HIGH); 
  delay(10); 
  if (distanceSensorS.begin() != 0) {
    Serial.println("Side sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }

  Serial.println("Both VL53L1X sensors online!");
  distanceSensorF.setDistanceModeShort();
  distanceSensorS.setDistanceModeShort();
  distanceSensorS.startRanging(); 
  distanceSensorF.startRanging(); 
}


void loop(void){
  distanceSensorS.startRanging(); 
  distanceSensorF.startRanging(); 
  if (distanceSensorS.checkForDataReady() && distanceSensorF.checkForDataReady() ){
    int distanceS = distanceSensorS.getDistance();
    int distanceF = distanceSensorF.getDistance();
    Serial.print("Time(ms): ");
    Serial.print(millis());
    Serial.print("|Front Distance(mm): ");
    Serial.print(distanceF);
    Serial.print("|Side Distance(mm): ");
    Serial.println(distanceS);
  }
  distanceSensorS.stopRanging(); 
  distanceSensorF.stopRanging(); 

}
