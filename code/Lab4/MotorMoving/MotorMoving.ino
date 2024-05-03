#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#define BLE_UUID_TEST_SERVICE "f8651f9b-7212-45db-87e7-6b2bfcf2a743"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);
RobotCommand robot_cmd(":|");
EString tx_estring_value;

float tx_float_value = 0.0;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;
int speedL = 60;
int speedR = 60;


enum CommandTypes {
    ECHO,
    STOP,
    FORWARD,
    BACKWARD,
    CW,
    CCW,
};

void setup(){
  Serial.begin(115200);
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

  pinMode(LED_BUILTIN, OUTPUT);
  blink(3);
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
      
      case FORWARD:
        success = robot_cmd.get_next_value(speedL);
        if (!success)
          return;
        success = robot_cmd.get_next_value(speedR);
        if (!success)
          return;

        tx_estring_value.clear();
        tx_estring_value.append("Forward ");
        tx_estring_value.append(speedL);
        tx_estring_value.append(", ");
        tx_estring_value.append(speedR);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        analogWrite(motorL1, speedL); 
        analogWrite(motorL2, 0);
        analogWrite(motorR1, 0); 
        analogWrite(motorR2, speedR);
        speedL = 60;
        speedR = 60;
        break;

      case BACKWARD:
        success = robot_cmd.get_next_value(speedL);
        if (!success)
          return;
        success = robot_cmd.get_next_value(speedR);
        if (!success)
          return;

        tx_estring_value.clear();
        tx_estring_value.append("Backward ");
        tx_estring_value.append(speedL);
        tx_estring_value.append(", ");
        tx_estring_value.append(speedR);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        analogWrite(motorL1, 0); 
        analogWrite(motorL2, speedL);
        analogWrite(motorR1, speedR); 
        analogWrite(motorR2, 0);
        speedL = 60;
        speedR = 60;
        break;

      case CW:
        success = robot_cmd.get_next_value(speedL);
        if (!success)
          return;
        success = robot_cmd.get_next_value(speedR);
            if (!success)
                return;

        tx_estring_value.clear();
        tx_estring_value.append("Turn Right ");
        tx_estring_value.append(speedL);
        tx_estring_value.append(", ");
        tx_estring_value.append(speedR);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        analogWrite(motorL1, speedL); 
        analogWrite(motorL2, 0);
        analogWrite(motorR1, speedR); 
        analogWrite(motorR2, 0);
        speedL = 60;
        speedR = 60;
        break;

      case CCW:
        success = robot_cmd.get_next_value(speedL);
        if (!success)
          return;
        success = robot_cmd.get_next_value(speedR);
            if (!success)
                return;

        tx_estring_value.clear();
        tx_estring_value.append("Turn Left ");
        tx_estring_value.append(speedL);
        tx_estring_value.append(", ");
        tx_estring_value.append(speedR);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        analogWrite(motorL1, 0); 
        analogWrite(motorL2, speedL);
        analogWrite(motorR1, 0); 
        analogWrite(motorR2, speedR);
        speedL = 60;
        speedR = 60;
        break;
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