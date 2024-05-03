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

float tx_float_value = 0.0;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

volatile bool isCollectingIMU = false; //Read and store IMU data if true
volatile bool isSendingIMU = false; ////Send stored IMU data if true
volatile bool isCollectingToF = false;
volatile bool isSendingToF = false;
unsigned long startTime = 0; 
unsigned long ToFTime = 0; 
int dataIndex1 = 0; 
int dataIndex2 = 0; 
const int maxData = 5000; //max data size for storing IMU or ToF data
const int maxTime1 = 1000; //max time duration in ms to read IMU data
const int maxTime2 = 10000; //max time duration in ms to read distance data
float timeIMU[maxData], timeToF[maxData];
float xAcc[maxData], yAcc[maxData], zAcc[maxData];
float xGyr[maxData], yGyr[maxData], zGyr[maxData];
int fDistance[maxData], sDistance[maxData];
int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;
float setPoint = 304;
float currentDistance = 0.0;
float previousError = 0.0;
float errorSum = 0.0;
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
unsigned long lastPIDTime = 0;

enum CommandTypes {
    ECHO,
    GET_IMU_DATA,
    GET_DISTANCE,
    STOP,
    MOVE,
    SET_PID_VALUE
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
  //IMU Setup
  bool initialized = false;
  while (!initialized){
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok){
      Serial.println("Trying again...");
      delay(500);
    }else{
      initialized = true;
    }
  }
  Serial.println("IMU online!");
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
      if (isCollectingIMU) {save_IMU();}
      if (isSendingIMU) {send_IMU();}
      if (isCollectingToF) {save_ToF();}
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
          //isCollectingData = false;
          //isSendingData = true;
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

      case GET_IMU_DATA:
        Serial.println("Recording IMU data with time stemps");
        startTime = millis();
        dataIndex1 = 0; 
        isCollectingIMU = true;
        tx_estring_value.clear();
        break;
      
      case GET_DISTANCE:
        Serial.println("Reading distance data with time stemps");
        ToFTime = millis();
        dataIndex2 = 0;
        isCollectingToF = true;
        tx_estring_value.clear();
        break;

      case STOP:
        analogWrite(motorL1, 0); 
        analogWrite(motorL2, 0);
        analogWrite(motorR1, 0); 
        analogWrite(motorR2, 0);
        tx_estring_value.clear();
        tx_estring_value.append("STOP");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.println(tx_estring_value.c_str());
        break;
      
      case MOVE:
        motor_move();
        break;
      
      case SET_PID_VALUE:
        if (!robot_cmd.get_next_value(Kp))
      return;
        if (!robot_cmd.get_next_value(Ki))
      return;
        if (!robot_cmd.get_next_value(Kd))
      return;
      tx_estring_value.clear();
      tx_estring_value.append("New PID values are set: ");
      tx_estring_value.append(Kp);
      tx_estring_value.append(", ");
      tx_estring_value.append(Ki);
      tx_estring_value.append(", ");
      tx_estring_value.append(Kd);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.println(tx_estring_value.c_str());
  }
}

void save_IMU() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); 
    // Store the data with timestamp
    timeIMU[dataIndex1] = (float)millis();
    xAcc[dataIndex1] = myICM.accX();
    yAcc[dataIndex1] = myICM.accY();
    zAcc[dataIndex1] = myICM.accZ();
    xGyr[dataIndex1] = myICM.gyrX();
    yGyr[dataIndex1] = myICM.gyrY();
    zGyr[dataIndex1] = myICM.gyrZ();
    dataIndex1++;
  }
  if (dataIndex1 > maxData || (millis() - startTime) > maxTime1) {
    isCollectingIMU = false;
    isSendingIMU = true;
  }
}

void send_IMU() {
  for(int i = 0; i<dataIndex1; i++) {
    tx_estring_value.clear();
    tx_estring_value.append("Time:");
    tx_estring_value.append(timeIMU[i]);

    tx_estring_value.append("|ACC-X:");
    tx_estring_value.append(xAcc[i]);
    tx_estring_value.append("|ACC-Y:");
    tx_estring_value.append(yAcc[i]);
    tx_estring_value.append("|ACC-Z:");
    tx_estring_value.append(zAcc[i]);

    tx_estring_value.append("|Gyr-X:");
    tx_estring_value.append(xGyr[i]);
    tx_estring_value.append("|Gyr-Y:");
    tx_estring_value.append(yGyr[i]);
    tx_estring_value.append("|Gyr-Z:");
    tx_estring_value.append(zGyr[i]);

    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.println(tx_estring_value.c_str());
  }
  tx_estring_value.clear();
  tx_estring_value.append("IMU data received.");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.println("IMU data sent.");
  tx_estring_value.clear();
  isSendingIMU = false;
  isCollectingIMU = false;
  dataIndex1=0;
}

void save_ToF() {
  distanceSensorS.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensorF.startRanging(); 
  while (!distanceSensorS.checkForDataReady() || !distanceSensorF.checkForDataReady()){
    delay(1);
  }
  timeToF[dataIndex2] = (float)millis();
  fDistance[dataIndex2] = (int)distanceSensorF.getDistance();
  sDistance[dataIndex2] = (int)distanceSensorS.getDistance();
  dataIndex2++;
  if (millis() - ToFTime > maxTime2) {
    isCollectingToF = false;
    isSendingToF = true;
  }
}

void send_ToF() {
  for(int i = 0; i<dataIndex2; i++) {
    tx_estring_value.clear();
    tx_estring_value.append("Time:");
    tx_estring_value.append(timeToF[i]);
    tx_estring_value.append("|Front Distance:");
    tx_estring_value.append(fDistance[i]);
    tx_estring_value.append("|Side Distance:");
    tx_estring_value.append(sDistance[i]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.println(tx_estring_value.c_str());
  }
  tx_estring_value.clear();
  tx_estring_value.append("ToF data received.");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.println("ToF data sent.");
  isSendingToF = false;
  isCollectingToF = false;
  dataIndex2=0;
}

float calculatePID(float setPoint, float currentDistance) {
  unsigned long now = millis();
  float dt = (float)(now - lastPIDTime);
  float error = setPoint - currentDistance;
  float Pout = Kp * error; // Proportional term
  errorSum += error;
  float Iout = Ki * errorSum; // Integral term
  float errorDiff = error - previousError;
  float Dout = Kd * errorDiff; // Derivative term

  float output = Pout + Iout + Dout;

  // Restrict to max/min if needed here
  // For example: output = constrain(output, minOutput, maxOutput);

  // Save the current time and error for the next calculation
  lastPIDTime = now;
  previousError = error;

  return output;
}

void motor_move(){
  int dir, speedL, speedR;
  if (!robot_cmd.get_next_value(dir))
    return;
  if (!robot_cmd.get_next_value(speedL))
    return;
  if (!robot_cmd.get_next_value(speedR))
    return;
  tx_estring_value.clear();
  analogWrite(motorL1, 0); 
  analogWrite(motorL2, 0);
  analogWrite(motorR1, 0); 
  analogWrite(motorR2, 0);
  if(dir==1){
    analogWrite(motorL1, speedL); 
    analogWrite(motorR2, speedR);
    tx_estring_value.append("Forward:");
  }
  if(dir==2){
    analogWrite(motorL2, speedL);
    analogWrite(motorR1, speedR); 
    tx_estring_value.append("Backward:");
  }
  if(dir==2){
    analogWrite(motorL2, speedL);
    analogWrite(motorR1, speedR); 
    tx_estring_value.append("Backward:");
  }
  if(dir==3){
    analogWrite(motorL2, speedL);
    analogWrite(motorR2, speedR);
    tx_estring_value.append("Turn Left:");
  }
  if(dir==4){
    analogWrite(motorL1, speedL); 
    analogWrite(motorR1, speedR); 
    tx_estring_value.append("Turn Right:");
  }
  tx_estring_value.append(speedL);
  tx_estring_value.append(", ");
  tx_estring_value.append(speedR);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.println(tx_estring_value.c_str());
  tx_estring_value.clear();
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