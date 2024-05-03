#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"

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
int speedL, speedR;

float tx_float_value = 0.0;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

const int maxData = 5000; //max data size for storing IMU data
int maxTime = 3000; //max time duration in ms to read and store IMU data
volatile bool isCollectingIMU = false; //Read and store IMU data if true
volatile bool isSendingIMU = false; ////Send stored IMU data if true
volatile bool isCollectingToF = false;
volatile bool isSendingToF = false;
volatile bool isLPID = false;
volatile bool isOPID = false;
unsigned long IMUTime = 0; 
int indexIMU = 0; 
unsigned long ToFTime = 0; 
int indexToF = 0; 
int indexPID = 0;

float timeToF[maxData], timePID[maxData];
int fDistance[maxData], outputLPID[maxData];
float timeIMU[maxData], Yaw[maxData], errorYaw[maxData], outputOPID[maxData];
float setAngle = 0;
float gz = 0, lastYaw = 0;
float errorSumYaw = 0, previousErrorYaw = 0, dtIMU=0;
float Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.0;
int minPWMYaw = 130, maxPWMYaw = 210, maxESumYaw = 1000; //windup
int setDistance = 304;
float Kp_lin = 1.0;
int minPWMLin = 40;
int maxPWMLin = 100;
int count = 0;
float yawCorrection = 1.28;

enum CommandTypes {
    ECHO,
    MOVE,
    SET_YAW_PID,
    SET_LINEAR_PID,
    PID,
};

void setup(){
  Wire.begin();
  Wire.setClock(400000);
  SERIAL_PORT.begin(115200);
  motor_move(0,0);

  bool initialized = false;
  while (!initialized){
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok){
      Serial.println("IMU failed to begin. Trying again...");
      delay(500);
    }else{
      initialized = true;
      Serial.println("IMU sensor online!");
    }
  }

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
  distanceSensorF.setDistanceModeLong();
  distanceSensorS.setDistanceModeLong();

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
      if (isCollectingToF && isLPID) {save_ToF();}
      if (isCollectingIMU) {save_IMU();} 
      if (isOPID) {orient_PID();}
      motor_move(speedL, speedR);
      if (isSendingToF) {
        motor_move(0,0);
        send_ToF();
        if (indexPID>1) {send_PID();}
      }
      if (isSendingIMU) {
        motor_move(0,0);
        send_IMU();
      }
    }
    Serial.println("Disconnected");
    motor_move(0,0);
  }  
}

void handle_command(){   
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                            rx_characteristic_string.valueLength());
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
      tx_estring_value.append("Robot says -> ");
      tx_estring_value.append(char_arr);
      tx_estring_value.append(" :)");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());
      tx_estring_value.clear();
      break;

    case MOVE:
      if (!robot_cmd.get_next_value(speedL))
        return;
      if (!robot_cmd.get_next_value(speedR))
        return;
      break;

    case SET_YAW_PID:
      if (!robot_cmd.get_next_value(Kp_yaw))
        return;
      if (!robot_cmd.get_next_value(Ki_yaw))
        return;
      if (!robot_cmd.get_next_value(Kd_yaw))
        return;
      if (!robot_cmd.get_next_value(minPWMYaw))
        return;
      if (!robot_cmd.get_next_value(maxPWMYaw))
        return;
      if (!robot_cmd.get_next_value(maxESumYaw))
        return;
      if (!robot_cmd.get_next_value(yawCorrection))
        return;
      lastYaw=0;
      setAngle = 0;
      errorSumYaw = 0;
      previousErrorYaw = 0;
      tx_estring_value.clear();
      break;

    case SET_LINEAR_PID:
      if (!robot_cmd.get_next_value(Kp_lin))
        return;
      if (!robot_cmd.get_next_value(minPWMLin))
        return;
      if (!robot_cmd.get_next_value(maxPWMLin))
        return;
      if (!robot_cmd.get_next_value(setDistance))
        return;
      tx_estring_value.clear();
      break;
    
    case PID:
      Serial.println("PID start.");
      if (!robot_cmd.get_next_value(maxTime))
        return;
      ToFTime = millis();
      indexToF = 0;
      indexPID = 0;
      indexIMU = 0; 
      isCollectingToF = true;
      isSendingToF = false;
      isLPID = true;
      IMUTime = millis();
      isCollectingIMU = true;
      isSendingIMU = false;
      errorSumYaw = 0;
      previousErrorYaw = 0;
      tx_estring_value.clear();
      distanceSensorS.startRanging(); 
      distanceSensorF.startRanging(); 
      break;
  }
  
}

void save_ToF() {
    if (distanceSensorS.checkForDataReady() && distanceSensorF.checkForDataReady()){
      timeToF[indexToF] = (float)millis();
      fDistance[indexToF] = (int)distanceSensorF.getDistance();
      distanceSensorS.clearInterrupt();
      distanceSensorF.clearInterrupt();
      indexToF++;
    }
    if (millis() - ToFTime > maxTime || indexToF >= maxData) {
      distanceSensorS.stopRanging();
      distanceSensorF.stopRanging();
      speedL = 0;
      speedR = 0;
      isCollectingToF = false;
      isCollectingIMU = false;
      isSendingToF = true;
      isSendingIMU = true;
      lastYaw = Yaw[indexIMU-1];
    }
    linear_PID();
}

void linear_PID(){
  if (indexToF > 0){
    int currentDistance = extrapolateDistance();
    int errorL = currentDistance - setDistance;
    timePID[indexPID] = (float)millis();
    int speed2 = (int) (Kp_lin * errorL);
    speed2 += ((speed2 > 0) - (speed2 < 0)) * minPWMLin;
    speed2 = constrain(speed2, -maxPWMLin, maxPWMLin);
    outputLPID[indexPID] = speed2;
    speedL=speed2;
    speedR=(int) (speed2/1.12);
    indexPID++;
    if (!isOPID && errorL <= 0) {
      setAngle += 180;
      isLPID = false;
      isOPID = true;
    }
  }
}

int extrapolateDistance() {
  if (indexToF == 1) return fDistance[indexToF-1];
  int dx = fDistance[indexToF-1] - fDistance[indexToF-2];
  int dt = timeToF[indexToF-1] - timeToF[indexToF-2];
  float slope = (float) dx/dt;
  float timeDiff = (float)(millis() - timeToF[indexToF-1]);
  int estimatedDistance = (int)(fDistance[indexToF-1] + slope * timeDiff); 
  return estimatedDistance;
}

void send_PID(){
    for(int i = 0; i<indexPID; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("Time:");
      tx_estring_value.append(timePID[i]);
      tx_estring_value.append("|PID output:");
      tx_estring_value.append(outputLPID[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    tx_estring_value.clear();
    tx_estring_value.append("PID data received.");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.println("PID data sent.");
    isLPID = false;
    indexPID=0;  
    motor_move(0,0);
}

void send_ToF() {
  speedL = 0;
  speedR = 0;
  for(int i = 0; i<indexToF; i++) {
    tx_estring_value.clear();
    tx_estring_value.append("Time:");
    tx_estring_value.append(timeToF[i]);
    tx_estring_value.append("|Front Distance:");
    tx_estring_value.append(fDistance[i]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.println(tx_estring_value.c_str());
  }
  tx_estring_value.clear();
  tx_estring_value.append("ToF data received.");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.println("ToF data sent.");
  isSendingToF = false;
  isCollectingToF = false;
  isCollectingIMU = false;
  indexToF=0;
  ToFTime = 0;
}

void save_IMU() {
    if (myICM.dataReady()) {
      myICM.getAGMT(); 
      timeIMU[indexIMU] = (float)millis();
      gz = myICM.gyrZ();
      processIMUData();
      indexIMU++;
    }
    if ((millis() - IMUTime) > maxTime || indexIMU >= maxData) {
      speedL = 0;
      speedR = 0;
      isCollectingIMU = false;
      isSendingIMU = true;
      isCollectingToF = false;
      isSendingToF = true;
      lastYaw = Yaw[indexIMU-1];
    }
}

void send_IMU() {
    motor_move(0,0);
    for(int i = 0; i<indexIMU; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("Time:");
      tx_estring_value.append(timeIMU[i]);
      tx_estring_value.append("|Yaw:");
      tx_estring_value.append(Yaw[i]);
      tx_estring_value.append("|PID output:");
      tx_estring_value.append(outputOPID[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    
    tx_estring_value.clear();
    tx_estring_value.append("IMU data received.");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.println("IMU data sent.");
    tx_estring_value.clear();
    isSendingIMU = false;
    isCollectingIMU = false;
    indexIMU=0;
    maxTime = 3000;
    IMUTime = 0;
    errorSumYaw= 0;
    previousErrorYaw=0;
    setAngle = 0;
}

void orient_PID(){
  if (indexIMU > 0){
    int speed = calculatePID(Yaw[indexIMU-1], setAngle, errorSumYaw, 
                              previousErrorYaw, Kp_yaw, Ki_yaw, Kd_yaw, 
                              minPWMYaw, maxPWMYaw, maxESumYaw);
    outputOPID[indexIMU-1] = speed;
    speedL=speed;
    speedR=-(int)speed/1.12;
    if (speed==0){
      count++;
    }else {
      count = 0;
    }

    if (count >4 && abs(setAngle - Yaw[indexIMU-1])<3){
      isLPID = true;
      isOPID = false;
      count = 0;
    }  
  }
}

float calculatePID(float currentValue, float setPoint, float& errorSum, 
                   float& previousError, float Kp, float Ki, float Kd, 
                   int minPWM, int maxPWM, int maxESum) {  
  float error = setPoint - currentValue ;
  float Pout = Kp * error; // Proportional term
  errorSum += error*dtIMU;
  errorSum = constrain(errorSum, -maxESum, maxESum);
  float Iout = Ki * errorSum; // Integral term
  float Dout = Kd * (error - previousError)/dtIMU; // Derivative term
  int output = (int) (Pout + Iout + Dout);
  previousError = error;
  //Convert the output PWM to 0 or 
  //the value between minimum PWM and maximum PWM(255)
  output += ((output > 0) - (output < 0)) * minPWM;
  output = constrain(output, -maxPWM, maxPWM);
  return output;
}

void processIMUData() {
  if (indexIMU == 0){
    Yaw[indexIMU] = lastYaw;
    return;
  }
  if (gz>-0.2 && gz<0.8) {gz=0;}
  dtIMU = (timeIMU[indexIMU] - timeIMU[indexIMU-1]) / 1000.0; 
  float yaw = Yaw[indexIMU-1] + gz * dtIMU * yawCorrection;
  Yaw[indexIMU] = yaw;
  Serial.println(gz);
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