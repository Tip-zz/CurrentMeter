/*
File:        CurrentMeter.ino
Author:      Tip Partridge
Origin:      02-Jul-2023

Description: Current meter based on ADS1115 16 bit ADC and NCS199A2R (100X) shunt amp with 0.010 Ohm shunt..
  
Target: Arduino Nano

Revisions:  02-Jul-2023 (TEP) v0.0 First pass.
            06-Jul-2023 (TEP) v1.0 Looks pretty good.
            20-Jul-2023 (TEP) v1.1 Add ADCOffset
            30-Jul-2023 (TEP) v1.2 Seems to work pretty well, probably less than 2 bugs.
                                  Add commas to log so spreadsheet can import as .csv.
*/
//
//**************************
// Global constant and variable declarations
//**************************
#include <Wire.h> // For I2C
#include <ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <EEPROM.h>
#include <LibPrintf.h>

#define vers "CurrentMeter v1.2"

// Pin assignments
const int LEDPin = 13;
const int ADCreadyPin = 3;
const int negPin = 2;   // negative sign LED
// SDA = A5
// SCL = A6

// ADC
ADS1115 ads1115(0x48); // ads1115 at default address 0x48
#define GAIN_TWOTHIRDS 0    // ADC gain command arguments
#define GAIN_ONE       1    // << Use this one, 0.125mA/b
#define GAIN_TWO       2
#define GAIN_FOUR      4
#define GAIN_EIGHT     8
#define GAIN_SIXTEEN   16
const float FSR1 = 6.144;   // +/- Full scale voltage (32767b)
const float FSR2 = 4.096;   // << Use this one
const float FSR3 = 2.048;
const float FSR4 = 1.024;
const float FSR5 = 0.512;
const float FSR6 = 0.256;
#define SPS8   0            // sample per second
#define SPS16  1
#define SPS32  2
#define SPS64  3
#define SPS128 4
#define SPS250 5
#define SPS475 6
#define SPS860 7            // << Use this one
float ADCFull = 32767; // 15 bit ADC (16 bits - sign bit)

// 7 Segment Display
Adafruit_7segment AmpDisplay = Adafruit_7segment();
#define DISPLAY_ADDRESS   0x70

// Flags
volatile bool gotADCValue;  // set in ISR when ADC has new value ready

//Defaults
float VFull = FSR2;
int sampleMode = SPS860;
int frequency = 860;
int nSample = 430;

// Working variables
char cmdChar;           // user comand via USB serial
int ADCVal;             // latest ADC reading
long ADCValSum;         // running sum of ADC readings
long ADCVal0;           // raw sum before offset correction
long ADCHigh;           // Highest ADC reading
long ADCLow;            // Lowest ADC reading
int sample;             // number of samples so far
float Amps;             // conveniently 1V = 1A
char astr[83];          // general purpose string

//Parameters
float VScale;           // calibrated volts per ADC bit
float ADCCal;           // scale factor to calibrate voltage readings
long ADCOffset;         // sum of nsamples ADC readings done with 0 input, used in loop.
float fADCOffset;       // floating point offset for one ADC reading, calc from ADCOffset, used in Serial report.
bool SerialDisplay;     // flag to turn on serial logging

//EEPROM
const int EEPROMFlag = 12345;   // value that lets program know that EEPROM has been programmed.
int EEPROMProgrammedFlagAddr = 0;
int ADCCalAddr = EEPROMProgrammedFlagAddr + sizeof(EEPROMFlag);
int ADCOffsetAddr = ADCCalAddr + sizeof(ADCCal);

// ***************   SSSSS   EEEEEE   TTTTTTTT   UU   UU   PPPPP    *********
// ***************  SS       EE          TT      UU   UU   PP   PP  *********
// Setup *********    SS     EEEEE       TT      UU   UU   PPPPP    *********
// ***************      SS   EE          TT      UU   UU   PP       *********
// ***************  SSSSS    EEEEEE      TT        UUU     PP       *********

void setup()
{
  Serial.begin(115200);       // USB virtual serial port
  Serial.println(vers);
  ads1115.begin();            // Initialize ads1115
  pinMode(LEDPin, OUTPUT);    // LED that blinks to indicate ADC is running
  digitalWrite(LEDPin, LOW);

// Set up ADC
   int EEPROMProgrammedFlag;
  EEPROM.get(EEPROMProgrammedFlagAddr, EEPROMProgrammedFlag);
  if (EEPROMProgrammedFlag != EEPROMFlag)   // EEPROM not initialized, load default values.
    {
    Serial.print("Loading defaults to EEPROM...");
    EEPROM.put(EEPROMProgrammedFlagAddr, EEPROMFlag);
    ADCOffset = 0;
    EEPROM.put(ADCOffsetAddr, ADCOffset);
    ADCCal = 1.0;
    EEPROM.put(ADCCalAddr, ADCCal);
    Serial.println(" Done.");
    }
  // load values from EEPROM
  EEPROM.get(ADCOffsetAddr, ADCOffset);
  fADCOffset = float(ADCOffset)/float(nSample);
  EEPROM.get(ADCCalAddr, ADCCal);
  VScale = ADCCal * VFull / ADCFull;  // ADC scale factor V/bit
  ads1115.setMode(0);                 // 0 = continuous mode
  ads1115.setDataRate(sampleMode);    // set samples per second
  attachInterrupt(digitalPinToInterrupt(ADCreadyPin), ADCread, RISING); // ADC ready interrupt
  ADCHigh = -32768;
  ADCLow = 32767;

  ads1115.setComparatorThresholdLow(0x0000);      // set to 0 for ALERT/RDY
  ads1115.setComparatorThresholdHigh(0x8000);     // set MSB to 1 for ALERT/RDY
  ads1115.setComparatorQueConvert(0);             // enable ALERT/RDY pin
  ads1115.setComparatorLatch(0);                  // whatever
  ads1115.setGain(GAIN_ONE);                      // 1x gain   +/- 4.096V  1 bit = 0.125mV
  ads1115.requestADC_Differential_0_1();          // kick ADC

// Set up the display.
  AmpDisplay.begin(DISPLAY_ADDRESS);
  pinMode( negPin, OUTPUT);

  Serial.println("(Press ? for menu)");
  prompt();
}

//*******************  LL       OOOOO     OOOOO    PPPPPP   **************************
//*******************  LL      00   00   00   00   PP   PP  **************************
// Loop *************  LL      00   O0   00   00   PPPPPP   **************************
//*******************  LL      00   OO   00   00   PP       **************************
//*******************  LLLLLL   00O00     OOOOO    PP       **************************

void loop()
{
// Check for conversion complete
  if (gotADCValue)
    {
    gotADCValue = false;                    // have to clear flag
    ADCVal = ads1115.getValue();
    if (ADCVal > ADCHigh) ADCHigh = ADCVal; // get highest value
    if (ADCVal < ADCLow) ADCLow = ADCVal;   // and lowest vslue
    ADCValSum += ADCVal;                    // add reading to running sum
    sample+=1;
    if (sample >= nSample)                  // have enough?
      {
      ADCVal0 = ADCValSum;
      ADCValSum += ADCOffset;
      Amps = VScale*(ADCValSum/sample);     // calculate mean
      if (Amps < 0.0)
        {
        AmpDisplay.print(-Amps,3);          // send to numeric display
        digitalWrite( negPin, HIGH);        // deal with sign LED
        }
      else
        {
        AmpDisplay.print(Amps,3);           // send to numeric display
        digitalWrite( negPin, LOW);         // deal with sign LED
        }
      AmpDisplay.writeDisplay();
      digitalWrite(LEDPin, !digitalRead(LEDPin)); // toggle activity LED
      if (SerialDisplay) 
        {
        Print_Data();
        }
      ADCValSum=0;                          // get ready for next data set
      ADCHigh = -32768;
      ADCLow = 32767;
      sample=0;
      }
    }

// User interface via USB Serial
  if (Serial.available()) 
    {
    cmdChar = Serial.read();      // Get char
    while (Serial.available()) Serial.read();  // flush
    switch(cmdChar)               // Parse command
      {
      case 13:                    // Display version and prompt, and why not?
      case 10:
        Serial.println();
        Serial.println(vers);
        Serial.println("(Press ? for menu)");
        prompt();
        break;
      case '?':                   // Help - display menu of commands
      case '/':
      case 'H':
      case 'h':
        echo();
        Serial.println("H - Display this help menu");
        Serial.println("G - Set gain (scale factor)");
        Serial.println("! (shift 1) - Set gain to 1.0");
        Serial.println("O - Capture offset");
        Serial.println(") (shift 0) - Set offset to 0");
        Serial.println("P - Display Parameters");
        Serial.println("S - Toggle serial logging");
        Serial.println("K - Kick the ADC if it's not running");
        prompt();
        break;
      case 'G':                   // Set gain (scale factor) and save to EEPROM
      case 'g':
        echo();
        delay(100);   // Be sure IDE Serial monitor is done sending newline
        while (Serial.available()) Serial.read();  // flush
        Set_Scale();
        prompt();
        break;
      case 'K':                   // restart ADC continuous reading, probably not needed.
      case 'k':
        echo();
        ads1115.requestADC_Differential_0_1();
        Serial.println("Kicked ADC.");
        prompt();
        break;
      case ')': // ) = <Shilt> 0  // Set offset to zero and save to EEPROM
        echo();
        ADCOffset = 0;
        EEPROM.put(ADCOffsetAddr, ADCOffset);
        fADCOffset = float(ADCOffset)/float(nSample);
        Serial.println("Offset set to zero.");
        prompt();
        break;
      case '!': // ! = <Shilt> 1  // Set gain to 1.0 and save to EEPROM
        echo();
        ADCCal = 1.0;
        EEPROM.put(ADCCalAddr, ADCCal);
        VScale = ADCCal * VFull / ADCFull;
        Serial.println("Gain set to one.");
        prompt();
        break;
      case 'O':                   // Capture offset and save to EEPROM
      case 'o':
        echo();
        ADCOffset = -ADCVal0;
        EEPROM.put(ADCOffsetAddr, ADCOffset);
        fADCOffset = float(ADCOffset)/float(nSample);
        Serial.println("Offset captured and saved.");
        prompt();
        break;
      case 'P':                   // Display parameters
      case 'p':
        echo();
        Display_parameters();
        prompt();
        break;
      case 'S':                   // Toggle Serial logging
      case 's':
        SerialDisplay = !SerialDisplay;
        Display_Header();
        break;
      default:
        if (cmdChar > 32 && cmdChar < 127)   // printable char?
          Serial.print(char(cmdChar));
        else
          Serial.print(byte(cmdChar));       // otherwise
        Serial.print(" ??");
        prompt();
        break;
      }
// Flush serial
      while (Serial.available()) Serial.read();  // flush
    }  //Serial.available
}

// ****************** End loop() *****************************************************

//**************************
// echo command
//**************************
void echo()
  {
  Serial.print(cmdChar);
  Serial.println();
  }

//**************************
// Print prompt
//**************************
void prompt()
  {
  Serial.println();
  Serial.print("> ");
  }

//**************************
// Print log header
//**************************
void Display_Header()
  {
  if (!SerialDisplay)
  Serial.println("=======,   ====   ,   ====   ,   ===    ,   =====");
  else Serial.println();
  Serial.println("ADC nom,   Amps   ,   High   ,   Low    ,   Delta");
  if (SerialDisplay)
  Serial.println("=======,   ====   ,   ====   ,   ===    ,   =====");
  }

//**************************
// Print log Data
//**************************
void Print_Data()
  {
 // Need to split up some sprintf or it prints nonsence??
  sprintf(astr, "%6d , ", ADCValSum/sample);
  Serial.print(astr);
  sprintf(astr, "%8.4f , %8.4f , ", 
      Amps, VScale * (float(ADCHigh)+fADCOffset));
  Serial.print(astr);
  sprintf(astr, "%8.4f , %8.4f", 
      VScale * (float(ADCLow)+fADCOffset), VScale * (float(ADCHigh)+fADCOffset) - VScale * (float(ADCLow)+fADCOffset));
  Serial.println(astr);
  }

//**************************
// Print parameters
//**************************
void Display_parameters()
  {
  Serial.println();
  Serial.println(vers);
  Serial.print("Sample rate (Samp/s):   "); Serial.println(frequency);
  Serial.print("Number of samples:      "); Serial.println(nSample);
  Serial.print("Calibrated bits/mA:     "); Serial.println(0.001/VScale,3);
  Serial.print("ADC offset:             "); Serial.print(ADCOffset);
  Serial.print("  (");  Serial.print(fADCOffset,3); Serial.println(")");
  Serial.print("ADC scale factor:       "); Serial.println(ADCCal,3);
  Serial.print("USB serial output:      ");
  if (SerialDisplay) Serial.println("On");
  else Serial.println("Off");
  delay(200);
  }

//**************************
// Set scale factor
//**************************
void Set_Scale()
  {
  float aflt;
  int ier;
  Serial.print("Enter scale factor"); Default(ADCCal);
  ier = getFloat( &aflt, 10000);
  Serial.println();
  if (ier > 0)
    {
    ADCCal = aflt;
    EEPROM.put(ADCCalAddr, ADCCal);
    VScale = ADCCal * VFull / ADCFull;  // ADC scale factor V/bit
    Serial.print("Scale factor (ADCCal) = "); Serial.println(ADCCal,4);
    }
  else
    {
    Serial.print("Scale factor not updated = "); Serial.println(ADCCal,4);
    }
  }

//**************************
// ADC Ready interrupt
//**************************
void ADCread()
  {
  gotADCValue = true;
///Serial.print('+');
  }
