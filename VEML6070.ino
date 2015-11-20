/* VEML6070_t3 Basic Example Code
 by: Kris Winer
 date: May 23, 2015
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic VEML6070 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled UV intensity and index data out. Sketch runs on the 3.3 V Teensy 3.1.
 
 VEML6070 is an advanced ultraviolet (UV) light sensor with I2C protocol interface
 and designed by the CMOS process. It is easily operated via a simple I2C command. The active 
 acknowledge (ACK) feature with threshold windows setting allows the UV sensor to send out a UVI alert message. 
 Under a strong solar UVI condition, the smart ACK signal can be easily implemented by the software programming.
 VEML6070 incorporates a photodiode, amplifiers, and analog / digital circuits into a single chip. VEML6070’s adoption of FiltronTM
 UV technology provides the best spectral sensitivity to cover UV spectrum sensing. 
 It has an excellent temperature compensation and a robust refresh rate setting that does not use an external RC low pass filter. 
 VEML6070 has linear sensitivity to solar UV light and is easily adjusted by an external resistor. Software shutdown 
 mode is provided, which reduces power consumption to be less than 1 μA. VEML6070’s operating voltage ranges from 
 2.7 V to 5.5 V.

 SDA and SCL  have external pull-up resistors (to 3.3V).
 2K2 resistors are on the VEML6070 breakout board.
 
 Hardware setup:
 VEML6070 Breakout ------ Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- pin 17 or 18
 SCL ----------------------- pin 16 or 19
 GND ---------------------- GND
 
  */
//#include "Wire.h"   
#include <i2c_t3.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

// See also VEML6070 data sheet:http://www.vishay.com/docs/84277/veml6070.pdf
//
////////////////////////////
// VEML6070 Registers/addresses //
////////////////////////////
#define  VEML6070_ADDR_ARA	      0x19 >> 1 // shift by one to get correct register addresses
#define  VEML6070_ADDR_CMD		    0x70 >> 1
#define  VEML6070_ADDR_DATA_LSB		0x71 >> 1
#define  VEML6070_ADDR_DATA_MSB		0x73 >> 1

enum IT {  // set of allowable sample rates
  IT_0_5 = 0,
  IT_1,
  IT_2,
  IT_4
};

// VEML6070 command register bits
#define VEML6070_CMD_DISABLE             0x01
#define VEML6070_CMD_WDM                 0x02

#define SerialDebug true  // set to true to get Serial output for debugging

// Pin definitions
int intPin = 15;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;                      
uint16_t count = 0;

//Integration Time with 270 kOhm set resistor
#define IT_1_2 0x00 //1/2T  90 ms
#define IT_1   0x01 //1T   180 ms
#define IT_2   0x02 //2T   360 ms
#define IT_4   0x03 //4T   720 ms

// Specify VEML6070 Integration time
uint8_t IT = IT_1;
uint8_t ITime = 180;

void setup()
{
 // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  I2Cscan();
  
  enableVEML6070(); // initalize sensor
 
  delay(150);
  
  digitalWrite(myLed, LOW);
}

void loop()
{  
  uint16_t steps = getUVdata();
  Serial.print("UV raw counts = "); Serial.println(steps);
  Serial.print("UV radiant power = "); Serial.print((float)steps*5.0); Serial.println(" microWatts/cm*cm");

  uint8_t risk_level = convert_to_risk_level(steps);
  if(risk_level == 0) Serial.println("UV risk level is low"); 
  if(risk_level == 1) Serial.println("UV risk level is moderate"); 
  if(risk_level == 2) Serial.println("UV risk level is high"); 
  if(risk_level == 3) Serial.println("UV risk level is very high"); 
  if(risk_level == 4) Serial.println("UV risk level is extreme"); 

  digitalWrite(myLed, !digitalRead(myLed));
  delay(ITime+20);
}

//===================================================================================================================
//====== Set of useful function to access UV data
//===================================================================================================================

uint16_t getUVdata()
{
  uint8_t msb=0, lsb=0;

  Wire.requestFrom(VEML6070_ADDR_DATA_MSB, 1); //MSB
  delay(100);
  if(Wire.available())
    msb = Wire.read();

  Wire.requestFrom(VEML6070_ADDR_DATA_LSB, 1); //LSB
  delay(100);
  if(Wire.available())
    lsb = Wire.read();

  return ((uint16_t) msb<<8) | lsb;
}

void enableVEML6070()
{
  Wire.beginTransmission(VEML6070_ADDR_CMD);
  Wire.write((IT<<2) | VEML6070_CMD_WDM); // Bit 1 must be 1, bit 0 is 0 for run and 1 for shutdown
  Wire.endTransmission();
}

uint16_t  convert_to_risk_level(uint16_t uvs_step)
{
uint16_t risk_level_mapping_table[4] = {2241, 4482, 5976, 8217};
uint16_t i;
for (i = 0; i < 4; i++)
{
if (uvs_step <= risk_level_mapping_table[i])
{
break;
}
}
return i;
}

// I2C scan function

void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}
