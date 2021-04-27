

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
//#include <ADS1256.h>



#define LED_Pin           13    //Pin for signal LED
#define servPotWip        A0   //Pin for actuator potentiometer wiper
#define recordPin         2  //Pin to start recordingdata or automatic test
#define testSwitch        4  //Pin toggled low/high to determine whether manual or automatic test is executed
#define armPin            3  //Pin to arm test rig
#define LC_P              A2 //Load cell positive input
#define LC_N              A1 //Load cell negative input
#define automatic         1
#define manual            0

#define motorFreq   1600
#define motorPWM    20   //0 to 255
#define potWipMax   3550
#define potWipMin   420  //pot wiper reading to indicate minimum position (actually 398, but needs a buffer)

#define ADCDRDYpin  5
#define ADCCSpin    6
#define ADCRSTpin   7

#define cmdRDATA    0x01
#define cmdRESET    0xFE

//Pin Definitions
#define pinDRDY 9
#define pinCS   10
#define pinRST  8

#define clockMHZ 7.68

// ADS1256 Register
#define STATUS 0x00
#define MUX 0x01
#define ADCON 0x02
#define DRATE 0x03
#define IO 0x04
#define OFC0 0x05
#define OFC1 0x06
#define OFC2 0x07
#define FSC0 0x08
#define FSC1 0x09
#define FSC2 0x0A

// ADS1256 Command
#define WAKEUP 0x00
#define RDATA 0x01
#define RDATAC 0x03
#define SDATAC 0x0f
#define RREG 0x10
#define WREG 0x50
#define SELFCAL 0xF0
#define SELFOCAL 0xF1
#define SELFGCAL 0xF2
#define SYSOCAL 0xF3
#define SYSGCAL 0xF4
#define SYNC 0xFC
#define STANDBY 0xFD
#define RESET 0xFE

// define multiplexer codes
#define ADS1256_MUXP_AIN0 0x00
#define ADS1256_MUXP_AIN1 0x10
#define ADS1256_MUXP_AIN2 0x20
#define ADS1256_MUXP_AIN3 0x30
#define ADS1256_MUXP_AIN4 0x40
#define ADS1256_MUXP_AIN5 0x50
#define ADS1256_MUXP_AIN6 0x60
#define ADS1256_MUXP_AIN7 0x70
#define ADS1256_MUXP_AINCOM 0x80

#define ADS1256_MUXN_AIN0 0x00
#define ADS1256_MUXN_AIN1 0x01
#define ADS1256_MUXN_AIN2 0x02
#define ADS1256_MUXN_AIN3 0x03
#define ADS1256_MUXN_AIN4 0x04
#define ADS1256_MUXN_AIN5 0x05
#define ADS1256_MUXN_AIN6 0x06
#define ADS1256_MUXN_AIN7 0x07
#define ADS1256_MUXN_AINCOM 0x08

// define gain codes
#define ADS1256_GAIN_1 0x00
#define ADS1256_GAIN_2 0x01
#define ADS1256_GAIN_4 0x02
#define ADS1256_GAIN_8 0x03
#define ADS1256_GAIN_16 0x04
#define ADS1256_GAIN_32 0x05
#define ADS1256_GAIN_64 0x06

// define drate codes
/*
        NOTE :   Data Rate vary depending on crystal frequency. Data rates
   listed below assumes the crystal frequency is 7.68Mhz
                for other frequency consult the datasheet.
*/

#define ADS1256_DRATE_30000SPS 0xF0
#define ADS1256_DRATE_15000SPS 0xE0
#define ADS1256_DRATE_7500SPS 0xD0
#define ADS1256_DRATE_3750SPS 0xC0
#define ADS1256_DRATE_2000SPS 0xB0
#define ADS1256_DRATE_1000SPS 0xA1
#define ADS1256_DRATE_500SPS 0x92
#define ADS1256_DRATE_100SPS 0x82
#define ADS1256_DRATE_60SPS 0x72
#define ADS1256_DRATE_50SPS 0x63
#define ADS1256_DRATE_30SPS 0x53
#define ADS1256_DRATE_25SPS 0x43
#define ADS1256_DRATE_15SPS 0x33
#define ADS1256_DRATE_10SPS 0x23
#define ADS1256_DRATE_5SPS 0x13
#define ADS1256_DRATE_2_5SPS 0x03

//**********************************************************************************
//ADS1256
//**********************************************************************************

class ADS1256 {
 public:
  ADS1256(float clockspdMhz, float vref, bool useresetpin);
  void writeRegister(unsigned char reg, unsigned char wdata);
  unsigned char readRegister(unsigned char reg);
  void sendCommand(unsigned char cmd);
  float readCurrentChannel();
  float readCurrentChannelRaw();
  void setConversionFactor(float val);
  void setChannel(byte channel);
  void setChannel(byte AIP, byte AIN);
  void begin(unsigned char drate, unsigned char gain, bool bufferenable);
  void waitDRDY();
  boolean isDRDY();
  void setGain(uint8_t gain);
  void readTest();

 private:
  void CSON();
  void CSOFF();
  unsigned long read_uint24();
  long read_int32();
  float read_float32();
  byte _pga;
  float _VREF;
  float _conversionFactor;
};

ADS1256::ADS1256(float clockspdMhz, float vref, bool useResetPin) {
  // Set DRDY as input
      //DDR_DRDY &= ~(1 << PINDEX_DRDY);
  pinMode(pinDRDY, INPUT);
  // Set CS as output
      //DDR_CS |= (1 << PINDEX_CS);
  pinMode(pinCS, OUTPUT);
  
  if (useResetPin) {
    // set RESETPIN as output
            //DDR_RESET |= (1 << PINDEX_RESET);
    pinMode(pinRST, OUTPUT);
    // pull RESETPIN high
          //PORT_RESET |= (1 << PINDEX_RESET);
    digitalWrite(pinRST, HIGH);
  }

  // Voltage Reference
  _VREF = vref;

  // Default conversion factor
  _conversionFactor = 1.0;

  // Start SPI on a quarter of ADC clock speed
  /*SPI.begin();
  SPI.beginTransaction(
      SPISettings(clockspdMhz * 1000000 / 4, MSBFIRST, SPI_MODE1));*/
}

void ADS1256::writeRegister(unsigned char reg, unsigned char wdata) {
  CSON();
  SPI.transfer(WREG | reg);
  SPI.transfer(0);
  SPI.transfer(wdata);
  //__builtin_avr_delay_cycles(8);  // t11 delay (4*tCLKIN) after WREG command,
                                  // 16Mhz avr clock is approximately twice
                                  // faster that 7.68 Mhz ADS1256 master clock
                                  //Due's internal clock is 84 MHz, so about 12ns/cycle
                                  //AVR's 16MHz clock is about 62.5ns/cycle
                                  //8*62.5ns = 500 ns
                                  //500 ns / 11.9 ns ~= 42 cycles
  delayMicroseconds(1); //DEBUG check this is equivalent to above
                        //twice the delay of the AVR version, should be fine?
  CSOFF();
}

unsigned char ADS1256::readRegister(unsigned char reg) {
  unsigned char readValue;
  
  CSON();
  SPI.transfer(RREG | reg);
  SPI.transfer(0);
  //__builtin_avr_delay_cycles(200);  // t6 delay (50*tCLKIN), 16Mhz avr clock is
                                    // approximately twice faster that 7.68 Mhz
                                    // ADS1256 master clock
                                    // 200*62.5ns = 12.5 us                          
  delayMicroseconds(13); //DEBUG check equivalent
             //Close to the AVR delay
  
  readValue = SPI.transfer(0);
  //__builtin_avr_delay_cycles(8);  // t11 delay
  delayMicroseconds(1); //DEBUG delay
  CSOFF();

  return readValue;
}

void ADS1256::sendCommand(unsigned char reg) {
  CSON();
  waitDRDY();
  SPI.transfer(reg);
  //__builtin_avr_delay_cycles(8);  // t11
  delayMicroseconds(1); //DEBUG delay
  CSOFF();
}

void ADS1256::setConversionFactor(float val) { _conversionFactor = val; }

/*void ADS1256::readTest() {
  unsigned char _highByte, _midByte, _lowByte;
  CSON();
  SPI.transfer(RDATA);
  __builtin_avr_delay_cycles(200);  // t6 delay

  _highByte = SPI.transfer(WAKEUP);
  _midByte = SPI.transfer(WAKEUP);
  _lowByte = SPI.transfer(WAKEUP);

  CSOFF();
}*/


float ADS1256::readCurrentChannel() {
  CSON();
  SPI.transfer(RDATA);
  //__builtin_avr_delay_cycles(200);  // t6 delay
  delayMicroseconds(13); //DEBUG delay
  float adsCode = read_float32();
  CSOFF();
  return ((adsCode / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) *
         _conversionFactor;
}

float ADS1256::readCurrentChannelRaw() {
  CSON();
  SPI.transfer(RDATA);
  //__builtin_avr_delay_cycles(200);  // t6 delay
  delayMicroseconds(13); //DEBUG delay
  float adsCode = read_float32();
  CSOFF();
  return ((adsCode / 0x7FFFFF) * _conversionFactor);
}

// Call this ONLY after RDATA command
unsigned long ADS1256::read_uint24() {
  unsigned char _highByte, _midByte, _lowByte;
  unsigned long value;

  _highByte = SPI.transfer(WAKEUP);
  _midByte = SPI.transfer(WAKEUP);
  _lowByte = SPI.transfer(WAKEUP);

  // Combine all 3-bytes to 24-bit data using byte shifting.
  value = ((long)_highByte << 16) + ((long)_midByte << 8) + ((long)_lowByte);
  return value;
}

// Call this ONLY after RDATA command
long ADS1256::read_int32() {
  long value = read_uint24();

  if (value & 0x00800000) {
    value |= 0xff000000;
  }

  return value;
}

// Call this ONLY after RDATA command
float ADS1256::read_float32() {
  long value = read_int32();
  return (float)value;
}

// Channel switching for single ended mode. Negative input channel are
// automatically set to AINCOM
void ADS1256::setChannel(byte channel) { setChannel(channel, -1); }

// Channel Switching for differential mode. Use -1 to set input channel to
// AINCOM
void ADS1256::setChannel(byte AIN_P, byte AIN_N) {
  unsigned char MUX_CHANNEL;
  unsigned char MUXP;
  unsigned char MUXN;

  switch (AIN_P) {
    case 0:
      MUXP = ADS1256_MUXP_AIN0;
      break;
    case 1:
      MUXP = ADS1256_MUXP_AIN1;
      break;
    case 2:
      MUXP = ADS1256_MUXP_AIN2;
      break;
    case 3:
      MUXP = ADS1256_MUXP_AIN3;
      break;
    case 4:
      MUXP = ADS1256_MUXP_AIN4;
      break;
    case 5:
      MUXP = ADS1256_MUXP_AIN5;
      break;
    case 6:
      MUXP = ADS1256_MUXP_AIN6;
      break;
    case 7:
      MUXP = ADS1256_MUXP_AIN7;
      break;
    default:
      MUXP = ADS1256_MUXP_AINCOM;
  }

  switch (AIN_N) {
    case 0:
      MUXN = ADS1256_MUXN_AIN0;
      break;
    case 1:
      MUXN = ADS1256_MUXN_AIN1;
      break;
    case 2:
      MUXN = ADS1256_MUXN_AIN2;
      break;
    case 3:
      MUXN = ADS1256_MUXN_AIN3;
      break;
    case 4:
      MUXN = ADS1256_MUXN_AIN4;
      break;
    case 5:
      MUXN = ADS1256_MUXN_AIN5;
      break;
    case 6:
      MUXN = ADS1256_MUXN_AIN6;
      break;
    case 7:
      MUXN = ADS1256_MUXN_AIN7;
      break;
    default:
      MUXN = ADS1256_MUXN_AINCOM;
  }

  MUX_CHANNEL = MUXP | MUXN;

  CSON();
  writeRegister(MUX, MUX_CHANNEL);
  sendCommand(SYNC);
  sendCommand(WAKEUP);
  CSOFF();
}

void ADS1256::begin(unsigned char drate, unsigned char gain, bool buffenable) {
  _pga = 1 << gain;
  sendCommand(
      SDATAC);  // send out SDATAC command to stop continous reading mode.
  writeRegister(DRATE, drate);  // write data rate register
  uint8_t bytemask = B00000111;
  uint8_t adcon = readRegister(ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  writeRegister(ADCON, byte2send);
  if (buffenable) {
    uint8_t status = readRegister(STATUS);
    bitSet(status, 1);
    writeRegister(STATUS, status);
  }
  sendCommand(SELFCAL);  // perform self calibration
  waitDRDY();
  ;  // wait ADS1256 to settle after self calibration
}

void ADS1256::CSON() {
  //PORT_CS &= ~(1 << PINDEX_CS);
  digitalWrite(pinCS, LOW);
}  // digitalWrite(_CS, LOW); 

void ADS1256::CSOFF() {
  //PORT_CS |= (1 << PINDEX_CS);
  digitalWrite(pinCS, HIGH);
}  // digitalWrite(_CS, HIGH); 

//Wait for DRDY to go low, indicating data ready
void ADS1256::waitDRDY() {
  //while (PIN_DRDY & (1 << PINDEX_DRDY))
  //  ;
  bool DRDYstate;
  while (DRDYstate == HIGH){
    DRDYstate = digitalRead(pinDRDY);
  }
}

boolean ADS1256::isDRDY() {
  //return ~(PIN_DRDY & (1 << PINDEX_DRDY));
  return ~digitalRead(pinDRDY);
  
}  


//*************************************************
//Main
//*************************************************

const int chipSelect = 10;

//float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.5; // voltage reference
// Initialize ADS1256 object
//ADS1256 adc(clockMHZ,vRef,false); // RESETPIN is permanently tied to 3.3v

ADS1256 adc(7.68,2.5,false); // RESETPIN is permanently tied to 3.3v


float data1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //Create motor shield object
Adafruit_DCMotor *linearActuator = AFMS.getMotor(1); //Select motor on M1 port

unsigned short LA_Pos = 0;
bool whichTest = LOW; //If low, manual test. If high, automatic test.
char rx_char = 0;


void setup() {
  // put your setup code here, to run once:

  analogReadResolution(12);
  pinMode(armPin, INPUT);
  pinMode(testSwitch, INPUT);
  pinMode(recordPin, INPUT);
  pinMode(LED_Pin, OUTPUT);

  ADCSetup(); //Set up ADS1256
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  AFMS.begin(motorFreq, NULL);
  linearActuator -> setSpeed(motorPWM);


  //Start ADC
  Serial.println("Starting ADC");

  //SPI.begin(); DEBUG should be initialized by SD lib. 
  
  //Serial.println("Setting ADC");
  SPI.beginTransaction(
      SPISettings(clockMHZ * 1000000 / 4, MSBFIRST, SPI_MODE1));

  adc.begin(ADS1256_DRATE_15SPS,ADS1256_GAIN_1,false);

  Serial.println("ADC Started");

  adc.setChannel(0,1);

}

void loop() {
  
  bool Armed = false;

  bool record = false;


  LA_Pos = analogRead(servPotWip);
  if(LA_Pos > potWipMin){
    linearActuator -> run(FORWARD);
    Serial.println("Rewinding actuator, please wait.");
  
    unsigned long potWipe = analogRead(servPotWip);
    while(potWipe > potWipMin){
        potWipe = analogRead(servPotWip);
        delay(1);
        }
    linearActuator -> run(RELEASE);
  }

  Serial.println("Enter 'A' to arm test rig");
  //Arm Test rig
  while(Armed == false){
    if (Serial.available() > 0){
      rx_char = Serial.read();
      if(rx_char == 'A'){
        Armed = true;
        SD.remove("datalog.csv");
        Serial.println("Armed");
      }
    }
    /*if (digitalRead(armPin) == HIGH){
      Armed = true;
      SD.remove("datalog.csv");
      Serial.println("Armed");
    }
    else{
      delay(1);
    }*/

  }  

  Serial.println("Enter 'R' to start recording data");
  //Wait for recordPin to be high
  while(record == false){
    if (Serial.available() > 0){
      rx_char = Serial.read();
      if(rx_char == 'R'){
        whichTest = digitalRead(testSwitch);
        //Serial.println(whichTest);

        record = true;
        Serial.println("recording");
        
        switch(whichTest){
          case manual: 
            Serial.println("manual");
            manualTest();
          break;
      
          case automatic:
            Serial.println("automatic");
            automaticTest();
          break;
      
          default:
          Serial.println("no case");
          break;
        }
      }
    }
    /*if (digitalRead(recordPin) == HIGH){
        whichTest = digitalRead(testSwitch);
        //Serial.println(whichTest);

        record = true;
        Serial.println("recording");
        
        switch(whichTest){
          case manual: 
            Serial.println("manual");
            manualTest();
          break;
      
          case automatic:
            Serial.println("automatic");
            automaticTest();
          break;
      
          default:
          Serial.println("no case");
          break;
      }
    }
    else{
      delay(1);
    }*/
  }
  
  //LA_Pos = analogRead(servPotWip);
  //Serial.println(LA_Pos);
  /*if(LA_Pos > 4000){
    linearActuator -> run(BACKWARD);
  }
  if(LA_Pos < 1000){
    linearActuator -> run(FORWARD);
  }*/
}

void manualTest(){
  

  //flash LED when starting to log data
    analogWrite(LED_Pin, HIGH);
    Serial.println("LED High");
  
  //Start logging data
    RADCDATA();

  //finish logging when memory full
    analogWrite(LED_Pin, LOW);
    Serial.println("Finished");
  //Dump to Computer
}

void automaticTest(){

  //flash LED when starting to log data
    analogWrite(LED_Pin, HIGH);
    Serial.println("LED High");

  //start motor to tighten string and start test
  linearActuator -> run(BACKWARD); //Actually forward,assuming red wire 

  //start reading data immediately, stop when it plateues
  RADCDATA();

  //finish logging when memory full
  analogWrite(LED_Pin, LOW);
  Serial.println("Finished");

  //Restore Actuator to starting position
  linearActuator -> run(FORWARD);
  Serial.println("Rewinding actuator, please wait.");

  unsigned long potWipe = analogRead(servPotWip);
  while(potWipe > potWipMin){
      potWipe = analogRead(servPotWip);
      delay(1); 
  }
  
  linearActuator -> run(RELEASE);
  Serial.println("Actuator position reset.");
}

void RADCDATA(){

  String dataString = "#";

  unsigned long timeRef = micros();
  unsigned long timeNow = timeRef;
  //unsigned long timeThen = 0; //DEBUG
  unsigned long deltaTime = 0;
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  
  
  while(timeNow <= timeRef + 10000000){
    //timeThen = timeNow; //DEBUG
    timeNow = micros();
    deltaTime = timeNow - timeRef; //Total time for plotting
    //deltaTime = timeNow - timeThen; //Time between samples to verify speed requirement 
    dataString = analogRead(LC_P) - analogRead(LC_N); //Removed for ADS version

    //ADC read:
    /*adc.waitDRDY(); //Maybe remove to make faster. Might not be necessary
    dataString = adc.readCurrentChannel();    */
 
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(dataString);
      dataFile.print(",");
      dataFile.println(deltaTime);
      //dataFile.println("\n");*/
      
      // print to the serial port too: //DEBUG START
      /*Serial.println(dataString);

      Serial.print("\nDelta Time: ");
      Serial.println(deltaTime);
      Serial.println("\n");// DEBUG END*/
      
      if(whichTest == automatic){
        LA_Pos = analogRead(servPotWip);
        //Serial.print("        ");
        //Serial.println(LA_Pos);
        if(LA_Pos > potWipMax){
          linearActuator -> run(RELEASE);
          Serial.println("Maximum actuator position");
          break;
        }
      }
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.csv");
    }
  }
    dataFile.close();
}

void ADCSetup(){
  pinMode(ADCDRDYpin, INPUT);
  pinMode(ADCCSpin, OUTPUT);
  pinMode(ADCRSTpin, OUTPUT);
  digitalWrite(ADCRSTpin, HIGH);
}

void CSON(){
  digitalWrite(ADCCSpin, LOW);
}

void CSOFF(){
  digitalWrite(ADCCSpin, HIGH);
}
