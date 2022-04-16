/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 02/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//unused pins are set to -1

#define NSS 5                                   //select pin on LoRa device
#define SCK 18                                  //SCK on VSPi
#define VSPI_MISO 19                            //MISO on VSPi 
#define VSPI_MOSI 23                            //MOSI on VSPi

#define HSPI_MISO 12                            //MISO on HSPi      
#define HSPI_MOSI 13                            //MOSI on HSPi

 
#define NRESET 27                               //reset pin on LoRa device
#define RFBUSY 25                               //busy line for LoRa device

#define LED1 -1                                  
#define DIO1 35                                 //DIO1 pin on LoRa device, used for RX and TX done 
#define DIO2 -1                                 //DIO2 pin on LoRa device, normally not used so set to -1 
#define DIO3 -1                                 //DIO3 pin on LoRa device, normally not used so set to -1
#define RX_EN -1                                //pin for RX enable, used on some SX128X devices, set to -1 if not used
#define TX_EN -1                                //pin for TX enable, used on some SX128X devices, set to -1 if not used 
#define BUZZER -1                               //pin for buzzer, set to -1 if not used 
#define VCCPOWER 14                             //pin controls power to external devices
#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;          //frequency of transmissions. Do not change, the LoRa SX1280 runs on 2.4 GHz 
const int32_t Offset = 0;                       //offset frequency for calibration purposes  
const uint8_t Bandwidth = LORA_BW_0800;         //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor 
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate

const int8_t TXpower = 10;                      //LoRa transmit power in dBm

const uint16_t packet_delay = 100;             //mS delay between packets. literally no idea what this does but i dont think we need it
