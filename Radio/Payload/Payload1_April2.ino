/*
   Transmitting template for SOAR payload.Adafruit_ISM330DHCX.h
*/

#include <SPI.h>                                               //the lora device is SPI based so load the SPI library                                         
#include <SX128XLT.h>                                          //Library for sx1280 
//#include "Settings.h"                                          //include the settings file, frequencies, LoRa settings etc for ardunio pinout
#include "Settings_ESP.h"                                       //include the settings file, frequencies, LoRa settings etc for ardunio pinout
#include <Wire.h>                                                // I2C library
//Include sensor libraries here

//libraries needed for magsensor (Adafruit_LIS2MDL)
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>

//libraries needed for IMU (Adafruit ISM330DHCX)
#include <Adafruit_ISM330DHCX.h>


#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"


Adafruit_BME680 bme; // I2C

Adafruit_ISM330DHCX ism330dhcx;
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define SEALEVELPRESSURE_HPA (1013.25)

// GNSS gps
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

SPIClass SPI2(HSPI);

uint8_t csPin = 15;

SX128XLT LT;                                                   //create a library class instance called LT


uint8_t TXPacketL;                                             // successful transmission
uint8_t start_buff[] = "a";                                     // low memory transmit mode makes the smallest buffer

void setup()
{
  Serial.begin(112500);
  Wire.begin();        // Join i2c bus
  
  SPI.begin();
  SPI2.begin();
  
  Serial.println("Connecting to ISM330DHCX ...");
  if (!ism330dhcx.begin_I2C()) {
    Serial.println("Failed to connect to ISM330DHCX");
    while (1) {
      delay(10);
    }
  }

  //Connecting to Adafruit_LIS2MDL Magnetometer
  /* Initialise the sensor */
  Serial.println("Connecting to LIS2MDL Magnetometer ...");
  if (!lis2mdl.begin()) {  // I2C mode

    Serial.println("Failed to connect to LIS2MDL Magnetometer");
    while (1) delay(10);
  }

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  if (myGNSS.begin(SPI2, csPin, 5500000) == false)
  {
    Serial.println(F("u-blox GNSS not detected on SPI bus. Please check wiring. Freezing."));
    while (1);
  }


  myGNSS.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR


  // connecting to LORASX_1280


  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
    LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
    Serial.print(F("Transmitter ready"));
  }
  else
  {
    Serial.println("No device responding");
  }


}

void loop()
{

  // define variables to be sent as float or uint8_t and then use functions from the sensor libraries to assign values
  // eg. float altitude = myPressure.readAltitudeFt();


  float temperature = bme.temperature;
  float pressure = bme.pressure / 100.0;
  float humidity = bme.humidity;
  float gas = bme.gas_resistance / 1000.0;
  float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);

  sensors_event_t event;
  lis2mdl.getEvent(&event);
  float x_magnetic = event.magnetic.x;
  float y_magnetic = event.magnetic.y;
  float z_magnetic = event.magnetic.z;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  float temper = temp.temperature;
  float x_accel = accel.acceleration.x;
  float y_accel = accel.acceleration.y;
  float z_accel = accel.acceleration.z;
  float x_gyro = gyro.gyro.x;
  float y_gyro = gyro.gyro.y;
  float z_gyro = gyro.gyro.z;

  float latitude = myGNSS.getLatitude();
  float longitude = myGNSS.getLongitude();
  float altitude = myGNSS.getAltitude();
  uint8_t SIV = myGNSS.getSIV();



  Serial.flush();                           // waits until the transmission is finished

  uint8_t len;                              // length of packet sent

  LT.startWriteSXBuffer(0);                 //start the write packet to buffer process
  LT.writeBuffer(start_buff, sizeof(start_buff));
  // add to the buffer by LT.write(type of packet (float/uint_8))

  // BME data. Bytes = 20
  LT.writeFloat(temperature);
  LT.writeFloat(pressure);
  LT.writeFloat(humidity);
  LT.writeFloat(gas);
  LT.writeFloat(alt);

  // magnometer data. Bytes = 12
  LT.writeFloat(x_magnetic);
  LT.writeFloat(y_magnetic);
  LT.writeFloat(z_magnetic);

  //IMU data. Bytes =
  LT.writeFloat(temper);
  LT.writeFloat(x_accel);
  LT.writeFloat(y_accel);
  LT.writeFloat(z_accel);
  LT.writeFloat(x_gyro);
  LT.writeFloat(y_gyro);
  LT.writeFloat(z_gyro);

  //GPS data. Bytes = 13
  LT.writeFloat(latitude);
  LT.writeFloat(longitude);
  LT.writeFloat(altitude);
  LT.writeUint8(SIV);


  len = LT.endWriteSXBuffer();

  if (TXPacketL = LT.transmitSXBuffer(0, len, 10000, TXpower, 0)) {  // The transmitting magic, a little underwhelming. will return packet length sent if OK, otherwise 0 if transmit error
    Serial.println(len);
  }
  else {
    Serial.println(F("Transmission failed"));
  }

  delay(100);

}
