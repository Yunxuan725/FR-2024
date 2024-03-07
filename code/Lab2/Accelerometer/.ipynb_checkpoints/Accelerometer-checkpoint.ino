#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>

#define SERIAL_PORT Serial
// The value of the last bit of the I2C address.
#define AD0_VAL 0
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

float pitch_a = 0, roll_a = 0, pitch_g = 0, roll_g = 0, yaw_g = 0, dt =0, pitch = 0, roll = 0, yaw = 0;
void setup(){
  SERIAL_PORT.begin(115200);
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
  blink(3);
}

void loop(){

  
  unsigned long last_time = micros();
  if (myICM.dataReady()){
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(myICM.agmt); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    //plotScaledAGMT(myICM.agmt); //serial plotter
    
    pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; //phi
    roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI;  //theta
    //Serial.print("Time:");
    Serial.print(millis());
    Serial.print(", ");
    //Serial.print("Pitch(degree):");
    Serial.print(pitch_a);
    Serial.print(", ");
    // Serial.print("Roll(degree):");
    Serial.print(roll_a);
    Serial.print(", ");
    
    dt = (micros()-last_time)/1000000.;
    last_time = micros();
    pitch_g = pitch_g + myICM.gyrX()*dt;
    roll_g = roll_g + myICM.gyrY()*dt;
    yaw_g = yaw_g + myICM.gyrZ()*dt;
    Serial.print(pitch_g);
    Serial.print(", ");
    Serial.print(roll_g);
    Serial.print(", ");
    Serial.print(yaw_g);
    Serial.print(", ");

    pitch = (pitch + myICM.gyrX() * dt) * 0.9 + pitch_a * 0.1;
    roll = (roll + myICM.gyrY() * dt) * 0.9 + roll_a * 0.1;
    Serial.print(pitch);
    Serial.print(", ");
    Serial.print(roll);
    Serial.println(" ");

    delay(30);
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




// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val){
  if (val > 0){
    SERIAL_PORT.print(" "); // If the value is positive, print a leading space.
    if (val < 10000) { SERIAL_PORT.print("0"); } 
    if (val < 1000) { SERIAL_PORT.print("0"); } 
    if (val < 100) { SERIAL_PORT.print("0"); } 
    if (val < 10) { SERIAL_PORT.print("0"); } 
  }else{
    SERIAL_PORT.print("-"); // If the value is negative, print a leading minus.
    if (abs(val) < 10000) { SERIAL_PORT.print("0"); } 
    if (abs(val) < 1000) { SERIAL_PORT.print("0"); }
    if (abs(val) < 100) { SERIAL_PORT.print("0"); }
    if (abs(val) < 10) { SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val)); // print the absolute value of the integer.
}

void printRawAGMT(ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if (val < 0){SERIAL_PORT.print("-");}
  else{SERIAL_PORT.print(" ");}

  for (uint8_t indi = 0; indi < leading; indi++){
    uint32_t tenpow = 0;
    if (indi < (leading - 1)){
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++){
      tenpow *= 10;
    }
    if (aval < tenpow){
      SERIAL_PORT.print("0");
    }
    else{
      break;
    }
  }
  if (val < 0){SERIAL_PORT.print(-val, decimals);}
  else{SERIAL_PORT.print(val, decimals);}
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( myICM.accX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accZ(), 5, 2 );
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( myICM.gyrX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrZ(), 5, 2 );
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat( myICM.magX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magZ(), 5, 2 );
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat( myICM.temp(), 5, 2 );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void plotScaledAGMT( ICM_20948_AGMT_t agmt){
  Serial.print("Acc-X:");
  Serial.print(myICM.accX());
  Serial.print(", ");
  Serial.print("Acc-Y:");
  Serial.print(myICM.accY());
  Serial.print(", ");
  Serial.print("Acc-Z:");
  Serial.print(myICM.accZ());
  //
  Serial.print(", ");
  Serial.print("Gyr-X:");
  Serial.print(myICM.gyrX());
  Serial.print(", ");
  Serial.print("Gyr-Y:");
  Serial.print(myICM.gyrY());
  Serial.print(", ");
  Serial.print("Gyr-Z:");
  Serial.println(myICM.gyrZ());
  /*uncommen to plot magnetic field and temperature
  //
  Serial.print(", ");
  Serial.print("Mag-X:");
  Serial.print(myICM.magX());
  Serial.print(", ");
  Serial.print("Mag-Y:");
  Serial.print(myICM.magY());
  Serial.print(", ");
  Serial.print("Mag-Z:");
  Serial.print(myICM.magZ());
  //
  Serial.print(", ");
  Serial.print("Tmp:");
  Serial.print(myICM.temp());
  Serial.println(" ");
  */
}