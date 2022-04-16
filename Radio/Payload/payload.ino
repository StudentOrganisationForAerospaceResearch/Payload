//Payload Model Ouroboros / Osprey Level 2
//Objective: Transmitting and storing sensor data

//Sensors:
// Magsensor (Adafruit_LIS2MDL)
// IMU (Adafruit ISM330DHCX)
// BME (Adafruit_BME680)
// GPS (GNSS ZOE-Click)

//Radio:
// Lora SX1280 radios (DLP-RFS1280)

// Libraries:
// Radio: //github.com/StuartsProjects/SX12XX-LoRa
// Magsensor: search for Adafruit_LIS2MDL in arduino library manager,
// IMU: search for ISM330DHCX in arduino library manager,
// BME: search for Adafruit_BME680 in arduino library manager,
// GPS: //librarymanager/All#SparkFun_u-blox_GNSS,

//libraries\\---------------------------------------------------------------------------------------------------
#include <SPI.h> //used for radio, GPS, MicroSD and flash
#include <Wire.h> //used for i2c sensors 
#include <Adafruit_Sensor.h> //For all Adafruit sensors
#include <Adafruit_LIS2MDL.h> //libraries needed for IMU (Adafruit ISM330DHCX)
#include "Adafruit_BME680.h" //libraries needed for BME (Adafruit_BME680)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //library needed for GPS(GNSS ZOE-Click)
#include <SX128XLT.h>  //libraries needed for radio (DLP-RFS1280)
#include "Settings_ESP.h" //libraries needed for frequencies, LoRa settings etc for ESP32

SPIClass SPI2(HSPI); //setting up HSPI to work on ESP32 in conjuction with VSPI

//Pin Set up\\---------------------------------------------------------------------------------------------------
//most Pins are already defined in the library
uint8_t csPin = 15; //GPIO pin 15 has been selected for chip select the GPS

//will be using GPIO 32 for LED

//Defining Library functions to be called\\---------------------------------------------------------------------------------------------------
SFE_UBLOX_GNSS myGNSS; //Create the library class instance called myGNSS
SX128XLT LT; //Create the library class instance called LT

//Needed for radio to work\\---------------------------------------------------------------------------------------------------
uint8_t TXPacketL; //successful transmission
uint8_t start_buff[] = "a"; //low memory transmit mode makes the smallest buffer

void setup()
{
    Serial.begin(112500);
}
