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
SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN1);

float tx_float_value = 0.0;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

volatile bool isCollectingData = false; //Read and store IMU data if true
volatile bool isSendingData = false; ////Send stored IMU data if true
unsigned long startTime = 0; 
int dataIndex = 0; 
const int maxData = 30; //max data size for storing IMU data
const int maxTime = 10000; //max time duration in ms to read and store IMU data
float timeIMU[maxData], timeToF[maxData];
float xAcc[maxData], yAcc[maxData], zAcc[maxData];
float xGyr[maxData], yGyr[maxData], zGyr[maxData];
int distance[maxData];
//double rangingTime[maxData];

enum CommandTypes {
    ECHO,
    GET_IMU_DATA,
    GET_DISTANCE,
};

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SHUTDOWN_PIN1, OUTPUT);
  digitalWrite(SHUTDOWN_PIN1, HIGH);
  Wire.begin();
  Wire.setClock(400000);
  SERIAL_PORT.begin(115200);

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

  if (distanceSensor.begin() != 0) {//Begin returns 0 on a good init
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensor online!");
  distanceSensor.setDistanceModeShort();

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

  blink(2);
}

void loop(){
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    while (central.connected()) {
      write_data(); // Send data
      read_data(); // Read data
      //save_IMU(); //store IMU data in array if GET_IMU_DATA command is receieved
      //send_IMU();
      save_DIS();
      send_DIS();
    }
    Serial.println("Disconnected");
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
          isCollectingData = false;
          isSendingData = true;
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
          break;

      case GET_IMU_DATA:
        Serial.println("Recording IMU data with time stemps");
        startTime = millis();
        dataIndex = 0; 
        isCollectingData = true;
        tx_estring_value.clear();
        break;
      
      case GET_DISTANCE:
        Serial.println("Recording distance data with time stemps");
        startTime = millis();
        dataIndex = 0; 
        isCollectingData = true;
        tx_estring_value.clear();
        break;
  }
}

void save_IMU() {
  if (isCollectingData) {
    if (myICM.dataReady()) {
      myICM.getAGMT(); 
      // Store the data with timestamp
      timeIMU[dataIndex] = (float)millis();
      xAcc[dataIndex] = myICM.accX();
      yAcc[dataIndex] = myICM.accY();
      zAcc[dataIndex] = myICM.accZ();
      xGyr[dataIndex] = myICM.gyrX();
      yGyr[dataIndex] = myICM.gyrY();
      zGyr[dataIndex] = myICM.gyrZ();
      dataIndex++;
    }
    if (dataIndex > maxData || (millis() - startTime) > maxTime) {
      isCollectingData = false;
      isSendingData = true;
    }
  }
}

void send_IMU() {
  if (isSendingData) {
    for(int i = 0; i<dataIndex; i++) {
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
    isSendingData = false;
    isCollectingData = false;
    dataIndex=0;
  }
}

void save_DIS() {
  if (isCollectingData) {
    //double rangeStart = micros();
    distanceSensor.startRanging();
    if (distanceSensor.checkForDataReady()) {
      timeToF[dataIndex] = (float)millis();
      distance[dataIndex] = (int)distanceSensor.getDistance();
      //rangingTime[dataIndex] = (double)micros()-rangeStart;
      dataIndex++;
    }
    distanceSensor.stopRanging();
    if (dataIndex > maxData) {
      isCollectingData = false;
      isSendingData = true;
    }
  }
}

void send_DIS() {
  if (isSendingData) {
    for(int i = 0; i<dataIndex; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("Time:");
      tx_estring_value.append(timeToF[i]);
      tx_estring_value.append("|Distance(mm):");
      tx_estring_value.append(distance[i]);
      //tx_estring_value.append("|RangingTime:");
      //tx_estring_value.append(rangingTime[i]);

      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.println(tx_estring_value.c_str());
    }
    isSendingData = false;
    isCollectingData = false;
    dataIndex=0;
  }
}

void resetArray(float arr[]) {
  for (int i = 0; i < maxData; i++) {arr[i] = 0;}
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