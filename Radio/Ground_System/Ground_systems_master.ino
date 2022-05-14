/*
Objective: Receive GPS and Sensor data from a Lora SX1280 radio (DLP-RFS1280) and print it as a CSV to the serial port

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

float recieved[30];   // array to recieve data

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
  for(int i = 0; i<30; i++){
    recieved[i] = LT.readFloat();
  }
  
  len = LT.endReadSXBuffer();   // end reading 

  //Print Data\\-----------------------------------------------------------------------------------------------
  for(int i = 0; i<30; i++){
    Serial.print(recieved[i]);
    Serial.print(",");  
  } 
  Serial.print(Packet_RSSI);
  Serial.print(","); 
  Serial.print(PacketSNR); 
  Serial.print(","); 
  Serial.println();
  delay(50);    // delay between reading only ends up printing at 10 times a second despite being set for 20
  
}
