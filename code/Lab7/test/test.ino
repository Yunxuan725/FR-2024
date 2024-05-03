#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#define BLE_UUID_TEST_SERVICE "f8651f9b-7212-45db-87e7-6b2bfcf2a743"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
#define SERIAL_PORT Serial
#define AD0_VAL 0
#define SHUTDOWN_PIN1 4
#define SHUTDOWN_PIN2 13

BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);
RobotCommand robot_cmd(":|");
EString tx_estring_value;
ICM_20948_I2C myICM;
SFEVL53L1X distanceSensorF(Wire, SHUTDOWN_PIN1); // Front sensor
SFEVL53L1X distanceSensorS(Wire, SHUTDOWN_PIN2); // Side sensor

int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;
volatile bool isSpeedChanged = false;

float tx_float_value = 0.0;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

const int maxData = 5000; //max data size for storing IMU or ToF data
int maxTime = 3000; //max time duration in ms to read distance data
volatile bool isCollectingToF = false;
volatile bool isSendingToF = false;

unsigned long ToFTime = 0; 
unsigned long currentTime = 0; 
int indexToF = 0; 

float timeToF[maxData];
int fDistance[maxData], sDistance[maxData];
int speed[maxData];
int speedL=0, speedR=0;
int t1, t2;
volatile bool l1 = true;
volatile bool l2 = true;

enum CommandTypes {
    ECHO,
    MOVE,
    GET_DISTANCE,
};

void setup(){
  Wire.begin();
  Wire.setClock(400000);
  SERIAL_PORT.begin(115200);
  pinMode(motorR1, OUTPUT);
  analogWrite(motorR1, 0); 
  pinMode(motorR2, OUTPUT);
  analogWrite(motorR2, 0);
  pinMode(motorL1, OUTPUT);
  analogWrite(motorL1, 0);
  pinMode(motorL2, OUTPUT);
  analogWrite(motorL2, 0);
  //BLE Setup
  BLE.begin();
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);
  BLE.addService(testService);
  tx_characteristic_float.writeValue(0.0);
  tx_estring_value.clear();
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());
  BLE.advertise();
  
  //ToFs Setup
  pinMode(SHUTDOWN_PIN1, OUTPUT);
  digitalWrite(SHUTDOWN_PIN1, LOW); 
  pinMode(SHUTDOWN_PIN2, OUTPUT);
  digitalWrite(SHUTDOWN_PIN2, LOW);
  delay(10);
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
  Serial.println("Both ToF sensors online!");
  distanceSensorF.setDistanceModeShort();
  distanceSensorS.setDistanceModeShort();
  //Ready
  pinMode(LED_BUILTIN, OUTPUT);
  blink(2);
  Serial.println("Ready!");
}

void loop(){
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    while (central.connected()) {
      write_data(); // Send data
      read_data(); // Read data  
      if (isCollectingToF) {save_ToF();}
      if (isSpeedChanged) {motor_move(speedL, speedR);}
      if (isSendingToF) {send_ToF();}
    }
    Serial.println("Disconnected");
  }  
}

void handle_command(){   
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(), rx_characteristic_string.valueLength());
  bool success;
  int cmd_type = -1;
  // Get robot command type (an integer) and check if the last tokenization was successful
  success = robot_cmd.get_command_type(cmd_type);
  if (!success) {return;}

  switch (cmd_type) {
      default:
          Serial.print("Invalid Command Type: ");
          Serial.println(cmd_type);
          break;

      case ECHO:   
          char char_arr[MAX_MSG_SIZE];
          success = robot_cmd.get_next_value(char_arr);
          if (!success)
              return;
          tx_estring_value.clear();
          tx_estring_value.append("Robot says: ");
          tx_estring_value.append(char_arr);
          tx_estring_value.append(" :)");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          Serial.println(tx_estring_value.c_str());
          break;
      
      case MOVE:
        if (!robot_cmd.get_next_value(speedL))
          return;
        if (!robot_cmd.get_next_value(speedR))
          return;
        isSpeedChanged = true;
        break;

      case GET_DISTANCE:
        Serial.println("Recording ToF data with time stemps");
        if (!robot_cmd.get_next_value(t1))
          return;
        if (!robot_cmd.get_next_value(t2))
          return;
        if (!robot_cmd.get_next_value(maxTime))
          return;
        ToFTime = millis();
        indexToF = 0;
        isCollectingToF = true;
        isSendingToF = false;
        tx_estring_value.clear();
        distanceSensorS.startRanging(); 
        distanceSensorF.startRanging(); 
        break;

  }
}

void save_ToF() {
    if (distanceSensorS.checkForDataReady() && distanceSensorF.checkForDataReady()){
      Serial.println(speedL);
      timeToF[indexToF] = (float)millis();
      fDistance[indexToF] = (int)distanceSensorF.getDistance();
      sDistance[indexToF] = (int)distanceSensorS.getDistance();
      speed[indexToF] = speedL;
      distanceSensorS.clearInterrupt();
      distanceSensorF.clearInterrupt();
      indexToF++;
    }
    currentTime = millis() - ToFTime;
    if (currentTime > t1 && l1) {
      speedL = 100;
      speedR = (int)100/1.12;
      isSpeedChanged = true;
      l1 = false;
    }
    if (currentTime > t2 && l2) {
      speedL = 0;
      speedR = 0;
      isSpeedChanged = true;
      l2=false;
    }
    if (millis() - ToFTime > maxTime || indexToF >= maxData) {
      distanceSensorS.stopRanging();
      distanceSensorF.stopRanging();
      isCollectingToF = false;
      isSendingToF = true;
    }
}

void send_ToF() {
  motor_move(0,0);
  for(int i = 0; i<indexToF; i++) {
    tx_estring_value.clear();
    tx_estring_value.append("Time:");
    tx_estring_value.append(timeToF[i]);
    tx_estring_value.append("|Front Distance:");
    tx_estring_value.append(fDistance[i]);
    tx_estring_value.append("|Side Distance:");
    tx_estring_value.append(speed[i]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
  }
  tx_estring_value.clear();
  tx_estring_value.append("ToF data received.");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.println("ToF data sent.");
  isSendingToF = false;
  isCollectingToF = false;
  indexToF=0;
  ToFTime = 0;
  maxTime = 3000;
}

void motor_move(int speedL, int speedR){
  if (speedL >= 0 && speedR >= 0) { //Forward
    analogWrite(motorL1, speedL); 
    analogWrite(motorL2, 0);
    analogWrite(motorR1, 0);
    analogWrite(motorR2, speedR); 
  }else if (speedL <= 0 && speedR <= 0) { //Backward
    analogWrite(motorL1, 0);
    analogWrite(motorL2, -speedL); 
    analogWrite(motorR1, -speedR);
    analogWrite(motorR2, 0);
  }else if (speedL >= 0 && speedR <= 0) { //Pivot Right
    analogWrite(motorL1, speedL); 
    analogWrite(motorL2, 0);
    analogWrite(motorR1, -speedR);
    analogWrite(motorR2, 0);
  }else if (speedL <= 0 && speedR >= 0) { //Pivot Left
    analogWrite(motorL1, 0);
    analogWrite(motorL2, -speedL); 
    analogWrite(motorR1, 0);
    analogWrite(motorR2, speedR); 
  }
  isSpeedChanged = false;
}

void write_data(){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;    
        }
        previousMillis = currentMillis;
    }
}

void read_data(){
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void blink(unsigned char no){
  //Indicate success
  for(char i=0; i<=no-1; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }  
}