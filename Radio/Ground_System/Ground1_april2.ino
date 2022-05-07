/*
Objective: Receive GPS and Sensor data from a Lora SX1280 radio (DLP-RFS1280)

Libraries: Radio (http://github.com/StuartsProjects/SX12XX-LoRa)

Receive instructions

  1. define variable
  2. read the incoming buffer in the order that the variables are being sent from the transmitter

Sumair Shergill - April 29
 */

//Libraries\\-------------------------------------------
#include <SPI.h>    //Radio uses SPI
#include <SX128XLT.h> //libraries needed for radio (DLP-RFS1280)
#include "Settings.h" //libraries needed for frequencies, LoRa settings etc for Ardunio

//LoRa\\-------------------------------------------------------
SX128XLT LT;    //Create the library class instance called LT
uint8_t RXPacketL;    // receive fuction return assigned to this. 1 if sucessful transmission and 0 if unsucessful
int16_t Packet_RSSI;    // strength of signal
int16_t PacketSNR;      // Signal to noise ratio

uint8_t len;                // length of received packet. Needed for recieve
char receivebuffer[1];      // the transmitted buffer has 1 byte in front that is not sensor data

// define transmitted variables (uint_8, float)  

float temperature;
float pressure;
float humidity;
float gas;
float alt;


float x_magnetic;
float y_magnetic;
float z_magnetic;

float temper;
float x_accel;
float y_accel;
float z_accel;
float x_gyro;
float y_gyro;
float z_gyro;   

float latitude;
float longitude;
float altitude;
uint8_t SIV;



void setup()

{

  Serial.begin(112500);
  SPI.begin();

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE)) // Checking if pins are correctly connected
  {
    Serial.println(F("LoRa OK"));
  }
  else
  {
    Serial.println(F("Device error"));
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);  //Setup radio based on settings specified on Settings.h
  Serial.println(F("Receiver ready"));
  Serial.println();
  delay(1000);
}

void loop(){

  RXPacketL = LT.receiveSXBuffer(0,0, WAIT_RX);  // Packet received
  Packet_RSSI = LT.readPacketRSSI();      // read signal stregth
  PacketSNR = LT.readPacketSNR();         // read signal to noise ratio
  LT.startReadSXBuffer(0);               //start buffer read at location 0
  LT.readBufferChar(receivebuffer);      //The first bite transmitted is a single character buffer

  //READ DATA\\----------------------------------------------------------------------

  // BME Data
  temperature = LT.readFloat(); 
  pressure = LT.readFloat();
  humidity = LT.readFloat();
  gas = LT.readFloat();
  alt= LT.readFloat();

  //Magnometer
  //x_magnetic = LT.readFloat();
  //y_magnetic = LT.readFloat();
  //z_magnetic = LT.readFloat();
  
  // IMU
  temper = LT.readFloat();
  x_accel = LT.readFloat();
  y_accel = LT.readFloat();
  z_accel = LT.readFloat();
  x_gyro = LT.readFloat();
  y_gyro = LT.readFloat();
  z_gyro = LT.readFloat();

  //GPS
  latitude = LT.readFloat();
  longitude = LT.readFloat();
  altitude = LT.readFloat();
  SIV = LT.readUint8();

  len = LT.endReadSXBuffer();   // end reading 

   //PRINT DATA\\----------------------------------------------------------------------
  
  Serial.print("PacketRSSI: ");
  Serial.print(Packet_RSSI);
  Serial.print(",");
  Serial.print("PacketSNR: ");
  Serial.print(PacketSNR);
  Serial.println();

  Serial.print("BME: ");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(gas);
  Serial.print(",");
  Serial.print(altitude);
  Serial.print(",");
  Serial.println();
  /*
  
  Serial.print("Magnometer: ");
  Serial.print(x_magnetic);
  Serial.print(",");
  Serial.print(y_magnetic);
  Serial.print(",");
  Serial.print(z_magnetic);
  
  Serial.println(",");

  */
  Serial.print("IMU: ");
  Serial.print(temper);
  Serial.print(",");
  Serial.print(x_accel);
  Serial.print(",");
  Serial.print(y_accel);
  Serial.print(",");
  Serial.print(z_accel);
  Serial.print(",");
  Serial.print(x_gyro);
  Serial.print(",");
  Serial.print(y_gyro);
  Serial.print(",");
  Serial.print(z_gyro);
  Serial.print(",");
  Serial.println();

  Serial.print("GPS: ");
  Serial.print(latitude);
  Serial.print(",");
  Serial.print(longitude);
  Serial.print(",");
  Serial.print(altitude);
  Serial.print(",");
  Serial.print(SIV);
  Serial.print(",");
  Serial.println();
 
  delay(500);    // delay between reading
  
}
