#include <ADS1256.h>
#include <SPI.h>

float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.5; // voltage reference
// Initialize ADS1256 object
ADS1256 adc(clockMHZ,vRef,false); // RESETPIN is permanently tied to 3.3v

float data1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial.println("Starting ADC");

  adc.begin(ADS1256_DRATE_15SPS,ADS1256_GAIN_1,false);

  Serial.println("ADC Started");

  adc.setChannel(0,1);
}

void loop() {
  // put your main code here, to run repeatedly:
  adc.waitDRDY();
  data1 = adc.readCurrentChannel();
  Serial.println(data1, HEX);
}
