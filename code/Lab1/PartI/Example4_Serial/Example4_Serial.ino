#define BAUD 115200       
#define CONFIG SERIAL_8N1 // a config value from HardwareSerial.h (defaults to SERIAL_8N1)

void setup() {
  Serial.begin(BAUD); // set the baud rate with the begin() method
  Serial.println("\n\nApollo3 - Serial");

  // the Apollo3 core supports printf on Serial
  for (size_t idx = 0; idx < 10; idx++){
    Serial.printf("printf supports printing formatted strings! count: %d\n", idx);
  }

  Serial.println("\nEcho... (type characters into the Serial Monitor to see them echo back)\n");
}

void loop() {
  while(Serial.available()){
    Serial.write(Serial.read());
  }
}