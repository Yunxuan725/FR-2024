#define RESOLUTION_BITS (16)      // choose resolution (explained in depth below)

#ifdef ADCPIN
#define EXTERNAL_ADC_PIN ADCPIN   // ADCPIN is the lowest analog capable pin exposed on the variant
#endif                            // - if no exposed pins are analog capable this will be undefined
                                  // - to use another pin provide an analog capable pin number such as:
                                  //   - A0 -> A9 (when analog pins are named sequentially from 0)
                                  //   - A11 -> A13, A16, A29, A31 -> A35 (when pins are named after Apollo3 pads)
                                  //   - A variant-specific pin number (when none of the above apply)

void setup() {
  Serial.begin(115200);
  Serial.println("Apollo3 - analogRead");

  analogReadResolution(RESOLUTION_BITS);    // set the resolution of analogRead results
  analogWriteResolution(RESOLUTION_BITS);   // match resolution for analogWrite
}

void loop() {
#ifdef ADCPIN
  int external = analogRead(EXTERNAL_ADC_PIN); // reads the analog voltage on the selected analog pin
  Serial.printf("external (counts): %d, ", external);
  analogWrite(LED_BUILTIN, external);
#endif

  int vcc_3 = analogReadVCCDiv3();    // reads VCC across a 1/3 voltage divider
  int vss = analogReadVSS();          // ideally 0
  int temp_raw = analogReadTemp();    // raw ADC counts from die temperature sensor
  
  float temp_f = getTempDegF();       // computed die temperature in deg F
  float vcc_v = getVCCV();            // computed supply voltage in V

  Serial.printf("temp (counts): %d, vcc/3 (counts): %d, vss (counts): %d, time (ms) %d\n", temp_raw, vcc_3, vss, millis());
}