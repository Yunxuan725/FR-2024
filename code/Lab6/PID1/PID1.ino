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

BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);
RobotCommand robot_cmd(":|");
EString tx_estring_value;
ICM_20948_I2C myICM;

int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;
volatile bool isSpeedChanged = false;

float tx_float_value = 0.0;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

const int maxData = 5000; //max data size for storing IMU data
int maxTime = 3000; //max time duration in ms to read and store IMU data
volatile bool isCollectingIMU = false; //Read and store IMU data if true
volatile bool isSendingIMU = false; ////Send stored IMU data if true
volatile bool isOPID = false;

unsigned long IMUTime = 0; 
int indexIMU = 0; 

float timeIMU[maxData];
float Yaw[maxData],setYaw[maxData], outputPID1[maxData];
float gz;
float lastYaw = 0;
int speedL, speedR;
float setAngle = 0;
float errorSumYaw = 0, previousErrorYaw = 0, dtIMU=0;
float Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.0;
int minPWMYaw = 130, maxPWMYaw = 210, maxESumYaw = 1000; //windup

enum CommandTypes {
    ECHO,
    MOVE,
    SET_YAW_PID,
    GET_ANGLE,
    SET_ANGLE,
    ORIENT_PID,
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
      save_IMU();
      if (isOPID) {orient_PID();}  
      if(isSpeedChanged) {motor_move(speedL, speedR);}
      send_IMU();
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
      isSpeedChanged = true;
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
      lastYaw=0;
      setAngle = 0;
      tx_estring_value.clear();
      tx_estring_value.append("New PID values: ");
      tx_estring_value.append(Kp_yaw);
      tx_estring_value.append(", ");
      tx_estring_value.append(Ki_yaw);
      tx_estring_value.append(", ");
      tx_estring_value.append(Kd_yaw);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.println(tx_estring_value.c_str());
      tx_estring_value.clear();
      break;

    case GET_ANGLE:
      Serial.println("Recording IMU data with time stemps");
      IMUTime = millis();
      indexIMU = 0;
      isCollectingIMU = true;
      isSendingIMU = false;
      tx_estring_value.clear();
      break;

    case SET_ANGLE:
      if (!robot_cmd.get_next_value(setAngle))
        return;
      errorSumYaw = 0;
      previousErrorYaw = 0;
      break;

    case ORIENT_PID:
      Serial.println("Orientation PID start.");
      if (!robot_cmd.get_next_value(maxTime))
        return;
      IMUTime = millis();
      indexIMU = 0; 
      isCollectingIMU = true;
      isSendingIMU = false;
      isOPID = true;
      errorSumYaw = 0;
      previousErrorYaw = 0;
      tx_estring_value.clear();
      break;
  }
}

void save_IMU() {
  if (isCollectingIMU) {
    if (myICM.dataReady()) {
      myICM.getAGMT(); 
      timeIMU[indexIMU] = (float)millis();
      gz = myICM.gyrZ();
      processIMUData();
      indexIMU++;
    }
    if ((millis() - IMUTime) > maxTime || indexIMU >= maxData) {
      isCollectingIMU = false;
      isSendingIMU = true;
      lastYaw = Yaw[indexIMU-1];
    }
  }
}

void send_IMU() {
  if (isSendingIMU) {
    motor_move(0,0);
    for(int i = 0; i<indexIMU; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("Time:");
      tx_estring_value.append(timeIMU[i]);
      tx_estring_value.append("|Yaw:");
      tx_estring_value.append(Yaw[i]);
      if (isOPID) {
        tx_estring_value.append("|setYaw:");
        tx_estring_value.append(setYaw[i]);
        tx_estring_value.append("|PID output:");
        tx_estring_value.append(outputPID1[i]);
      }
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
    isOPID = false;
    errorSumYaw= 0;
    previousErrorYaw=0;
    setAngle = 0;
  }
}

void orient_PID(){
  if (indexIMU > 0){
    int speed = calculatePID(Yaw[indexIMU-1], setAngle, errorSumYaw, 
                              previousErrorYaw, Kp_yaw, Ki_yaw, Kd_yaw, 
                              minPWMYaw, maxPWMYaw, maxESumYaw);
    setYaw[indexIMU-1] = setAngle;
    outputPID1[indexIMU-1] = speed;
    speedL=speed;
    speedR=-(int)speed/1.12;
    isSpeedChanged = true;
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
  float yaw = Yaw[indexIMU-1] + gz * dtIMU;
  Yaw[indexIMU] = yaw;
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