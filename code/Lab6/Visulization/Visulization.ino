#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>

#define SERIAL_PORT Serial
// The value of the last bit of the I2C address.
#define AD0_VAL 0
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
float ax, ay, az, gx, gy, gz;
const float alpha = 0.98;
int motorL1 = 7;
int motorL2 = 9;
int motorR1 = 11;
int motorR2 = 12;
float pitch = 0;
float roll = 0;
float yaw = 0;
unsigned long lastUpdateTime = 0;
void setup(){
  SERIAL_PORT.begin(115200);
  pinMode(motorR1, OUTPUT);
  analogWrite(motorR1, 0); 
  pinMode(motorR2, OUTPUT);
  analogWrite(motorR2, 0);
  pinMode(motorL1, OUTPUT);
  analogWrite(motorL1, 0);
  pinMode(motorL2, OUTPUT);
  analogWrite(motorL2, 0);
  while (!SERIAL_PORT){};
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);

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
  blink(2);
  lastUpdateTime = micros();
}

void loop(){
  if (myICM.dataReady()){
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    ax = -myICM.accX();
    ay = -myICM.accY();
    az = -myICM.accZ();
    gx = myICM.gyrX();
    gy = myICM.gyrY();
    gz = myICM.gyrZ();

    unsigned long currentUpdateTime = micros();
    float dt = (currentUpdateTime-lastUpdateTime)/1000000.;
    lastUpdateTime = currentUpdateTime;
    // Calculate roll (phi) and pitch (theta) angles
    pitch = atan2(ay, az)* 180.0 / M_PI;
    roll = atan2(ax, az)* 180.0 / M_PI;

    if (gx>-1.2 && gx<0) {gx=0;}
    if (gy>0 && gy<1.3) {gy=0;}
    if (gz>-0.2 && gz<0.8) {gz=0;}
    pitch = alpha * (pitch+gx*dt) + (1 - alpha) * pitch;
    roll = alpha * (roll+gy*dt) + (1 - alpha) * roll;
    yaw += gz*dt;

    // SERIAL_PORT.print(gx);
    // SERIAL_PORT.print("\t");
    // SERIAL_PORT.print(gy);
    // SERIAL_PORT.print("\t");
    // SERIAL_PORT.print(gz);
    // SERIAL_PORT.print("\t");
    SERIAL_PORT.print(pitch);
    SERIAL_PORT.print("\t");
    SERIAL_PORT.print(roll);
    SERIAL_PORT.print("\t");
    SERIAL_PORT.println(yaw);
  }else{
    SERIAL_PORT.println("Waiting for data");
    delay(500);
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