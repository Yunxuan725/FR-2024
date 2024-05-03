#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

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

float tx_float_value = 0.0;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

volatile bool isCollectingIMU = false; //Read and store IMU data if true
volatile bool isSendingIMU = false; ////Send stored IMU data if true
unsigned long IMUTime = 0; 
int indexIMU = 0; 
const int maxData = 5000; //max data size for storing IMU data
const int maxTime = 5000; //max time duration in ms to read and store IMU data
float timeIMU[maxData];
float ax, ay, az, gx, gy, gz;
//float Pitch[maxData], Roll[maxData];
float Yaw[maxData];
int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;
const float alpha = 0.98;

enum CommandTypes {
    ECHO,
    GET_ANGLE,
};

void setup(){
  SERIAL_PORT.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(motorR1, OUTPUT);
  analogWrite(motorR1, 0); 
  pinMode(motorR2, OUTPUT);
  analogWrite(motorR2, 0);
  pinMode(motorL1, OUTPUT);
  analogWrite(motorL1, 0);
  pinMode(motorL2, OUTPUT);
  analogWrite(motorL2, 0);

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
      save_IMU();
      send_IMU();
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

      case GET_ANGLE:
        Serial.println("Recording IMU data with time stemps");
        IMUTime = millis();
        indexIMU = 0;
        isCollectingIMU = true;
        isSendingIMU = false;
        tx_estring_value.clear();
        break;
  }
}

void save_IMU() {
  if (isCollectingIMU) {
    if (myICM.dataReady()) {
      myICM.getAGMT(); 
      // Store the data with timestamp
      timeIMU[indexIMU] = (float)micros();
      //ax = -myICM.accX();
      //ay = -myICM.accY();
      //az = -myICM.accZ();
      //gx = myICM.gyrX();
      //gy = myICM.gyrY();
      gz = myICM.gyrZ();
      processIMUData();
      indexIMU++;
    }
    if ((millis() - IMUTime) > maxTime) {
      isCollectingIMU = false;
      isSendingIMU = true;
    }
  }
}

void send_IMU() {
  if (isSendingIMU) {
    for(int i = 0; i<indexIMU; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("Time:");
      tx_estring_value.append(timeIMU[i]);

      //tx_estring_value.append("|Pitch:");
      //tx_estring_value.append(Pitch[i]);
      //tx_estring_value.append("|Roll:");
      //tx_estring_value.append(Roll[i]);
      tx_estring_value.append("|Yaw:");
      tx_estring_value.append(Yaw[i]);

      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.println(tx_estring_value.c_str());
    }
    tx_estring_value.clear();
    tx_estring_value.append("IMU data received.");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.println("IMU data sent.");
    isSendingIMU = false;
    isCollectingIMU = false;
    indexIMU=0;
    IMUTime = 0;
  }
}

void processIMUData() {
  //float pitch= atan2(ay, az) * 180/M_PI;
  //float roll = atan2(ax, az) * 180/M_PI;
  if (indexIMU == 0){
    //Pitch[indexIMU] = pitch;
    //Roll[indexIMU] = roll;
    Yaw[indexIMU] = 0;
    return;
  }
  //if (gx>-1.2 && gx<0) {gx=0;}
  //if (gy>0 && gy<1.3) {gy=0;}
  if (gz>-0.2 && gz<0.8) {gz=0;}

  float dt = (timeIMU[indexIMU] - timeIMU[indexIMU-1]) / 1000000.0; 
  //Pitch[indexIMU] = alpha*(Pitch[indexIMU-1] + gx * dt) + (1-alpha)*pitch;
  //Roll[indexIMU] = alpha*(Roll[indexIMU-1] + gy * dt) + (1-alpha)*roll;
  Yaw[indexIMU] = Yaw[indexIMU-1] + gz * dt;
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