#include <PDM.h> 
#include <arm_math.h>

#define pdmDataBufferSize 4096 //Default is array of 4096 * 32bit

uint16_t pdmDataBuffer[pdmDataBufferSize];
float g_fPDMTimeDomain[pdmDataBufferSize * 2];
float g_fPDMMagnitudes[pdmDataBufferSize * 2];
uint32_t sampleFreq;

AP3_PDM myPDM;   //Create instance of PDM class

void setup(){
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  if (myPDM.begin() == false){
    Serial.println("PDM Init failed. Are you sure these pins are PDM capable?");
    while (1)
      ;
  }

  Serial.println("PDM Initialized");
  printPDMConfig();
}

void loop(){
  if (myPDM.available()){
    myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
    printLoudest();
  }
}


// Analyze and print frequency data.
void printLoudest(void){
  float fMaxValue;
  uint32_t ui32MaxIndex;
  int16_t *pi16PDMData = (int16_t *)pdmDataBuffer;
  uint32_t ui32LoudestFrequency;

  for (uint32_t i = 0; i < pdmDataBufferSize; i++){
    g_fPDMTimeDomain[2 * i] = pi16PDMData[i] / 1.0;
    g_fPDMTimeDomain[2 * i + 1] = 0.0;
  }

  // Perform the FFT.
  arm_cfft_radix4_instance_f32 S;
  arm_cfft_radix4_init_f32(&S, pdmDataBufferSize, 0, 1);
  arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
  arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, pdmDataBufferSize);

  // Find the loudest frequency
  arm_max_f32(g_fPDMMagnitudes, pdmDataBufferSize / 2, &fMaxValue, &ui32MaxIndex);
  ui32LoudestFrequency = (sampleFreq * ui32MaxIndex) / pdmDataBufferSize;

  //Check if the loudest frequency is playing "A" note
  if(ui32LoudestFrequency >= 430 && ui32LoudestFrequency <= 450){
    Serial.printf("Loudest Frequency: %d Hz - Yes\n", ui32LoudestFrequency);
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on
  } else {
    Serial.printf("Loudest Frequency: %d Hz - No\n", ui32LoudestFrequency);
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off
  }
}

void printPDMConfig(void){
  uint32_t PDMClk;
  uint32_t MClkDiv;
  float frequencyUnits;

  switch (myPDM.getClockDivider()){
  case AM_HAL_PDM_MCLKDIV_4:
    MClkDiv = 4;
    break;
  case AM_HAL_PDM_MCLKDIV_3:
    MClkDiv = 3;
    break;
  case AM_HAL_PDM_MCLKDIV_2:
    MClkDiv = 2;
    break;
  case AM_HAL_PDM_MCLKDIV_1:
    MClkDiv = 1;
    break;

  default:
    MClkDiv = 0;
  }

  switch (myPDM.getClockSpeed()){
  case AM_HAL_PDM_CLK_12MHZ:
    PDMClk = 12000000;
    break;
  case AM_HAL_PDM_CLK_6MHZ:
    PDMClk = 6000000;
    break;
  case AM_HAL_PDM_CLK_3MHZ:
    PDMClk = 3000000;
    break;
  case AM_HAL_PDM_CLK_1_5MHZ:
    PDMClk = 1500000;
    break;
  case AM_HAL_PDM_CLK_750KHZ:
    PDMClk = 750000;
    break;
  case AM_HAL_PDM_CLK_375KHZ:
    PDMClk = 375000;
    break;
  case AM_HAL_PDM_CLK_187KHZ:
    PDMClk = 187000;
    break;

  default:
    PDMClk = 0;
  }

  // Record the effective sample frequency. 
  sampleFreq = (PDMClk / (MClkDiv * 2 * myPDM.getDecimationRate()));
  frequencyUnits = (float)sampleFreq / (float)pdmDataBufferSize;

  Serial.printf("Settings:\n");
  Serial.printf("PDM Clock (Hz):         %12d\n", PDMClk);
  Serial.printf("Decimation Rate:        %12d\n", myPDM.getDecimationRate());
  Serial.printf("Effective Sample Freq.: %12d\n", sampleFreq);
  Serial.printf("FFT Length:             %12d\n\n", pdmDataBufferSize);
  Serial.printf("FFT Resolution: %15.3f Hz\n", frequencyUnits);
}