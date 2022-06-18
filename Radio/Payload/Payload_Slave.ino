//Payload Model Tsukuyomi / Osprey 1
//Slave ESP32 Software
//Objective: Storing sensor data in a CSV

// Memory peripherals:
// W25Q128JV-SIM (flash memory)
// SparkFun SD Card Breakout (SD card)

// Libraries:
// W25Q128JV-SIM: SPIMemory V3.4.0 by Prajwal Bhattaram
// SparkFun SD Card Breakout: Arduino-ESP32 SD library by Espressif

// Configuring the data written to flash chip and SD card:
// 1. Change the sizeOfData variable to the total number of bytes being tranferred over. Note that this program currently only saves floats.
// 2. Change the fileHeader variable to the desired title for each column in the CSV file (make sure it is in CSV format).

//Libraries\\-----------------------------------------------------------------------------------------------------------------------------
#include "Wire.h" //needed for communication between slave and master ESP32
#include "SPIMemory.h" //needed for flash chip
#include "SPI.h" //needed for SD card and flash chip
#include "SD.h" //needed for SD card
#include "FS.h" //needed for SD card

//Defining library functions\\------------------------------------------------------------------------------------------------------------
File file; //FS.h library instance file
SPIFlash flash(5); //SPIMemory.h library instance flash, uses pin 5 as chip select pin

//Configurable variables\\----------------------------------------------------------------------------------------------------------------
const uint8_t sizeOfData = 48; //the number of bytes this ESP32 will recieve from the master
const uint8_t sizeOfDataInFloat = sizeOfData / 4; //the number of floats this ESP32 will recieve from the master
//const char fileHeader[264] = {"time,temperature,accel-x,accel-y,accel-z,gyro-x,gyro-y,gyro-z,temperature,pressure,humidity,gas-resistance,mag-x,mag-y,mag-z,compass,temperature,accel-x,accel-y,accel-z,gyro-x,gyro-y,gyro-z,temperature,velocity,temperature,differential pressure,lat,long,altitude\n"};
const char fileHeader[] = {"Time,IMU1-ax,IMU1-ay,IMU1-az,IMU1-gx,IMU1-gy,IMU1-gz,IMU1-temp,BME-temp,BME-hum,BME-p,BME-gasr,MS-p,MS-temp,GPS-lat,GPS-lon,GPS-alt,GPS-SIV,BMM-mx,BMM-my,BMM-mz,TMP-temp,FS-vel,LWLP-dp,LWLP-temp,SCD-temp,SCD-hum,SCD-CO2\n\0"};


//Setting up this ESP32 as a slave\\------------------------------------------------------------------------------------------------------
#define I2C_ESP32_ADDR (uint8_t) 0x11 //slave address

//Setting up HSPI for SD card\\-----------------------------------------------------------------------------------------------------------
#define SD_MISO     25
#define SD_MOSI     13
#define SD_SCLK     14
#define SD_CS       15

SPIClass SD_SPI(HSPI); //setting up HSPI to work in conjunction with VSPI 

//Initializing variables\\----------------------------------------------------------------------------------------------------------------
union float_and_char { //union used to recieve floats as bytes (chars) and then read as float
  float flt; // 4 bytes
  char c[4]; // 1 byte * 4
};

/* NEW STUFF 6-9 */

typedef struct {
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float temperature;
} __attribute__((packed)) IMU_6DOF_t;


typedef struct {
  float temperature;
  float humidity;
  float pressure;
  float gas_resistance;
} BME680_t;

typedef struct {
  float temperature;
} __attribute__((packed)) TMP75_t;

typedef struct {
  float air_velocity;
} __attribute__((packed)) FS3000_t;

typedef struct {
  float differential_pressure;
  float temperature;
} __attribute__((packed)) LWLP5000_t;

typedef struct {
  float latitude;
  float longitude;
  float altitude;
  float satellites_in_view;
} __attribute__((packed)) ZOE_M8Q_t;

typedef struct {
  float pressure;
  float temperature;
} __attribute__((packed)) MS5611_t;
/*
typedef struct {
  float geomagnetic_x;
  float geomagnetic_y;
  float geomagnetic_z;
} __attribute__((packed)) BMM150_t;
*/
typedef struct {
  float temperature;
  float humidity;
  float CO2_concentration;
} __attribute__((packed)) SCD30_t;

typedef struct {
  float time_stamp; // 4 bytes
  IMU_6DOF_t ISM330DHCX_data; // 28 bytes
  //IMU_6DOF_t MPU6050_data;
  BME680_t BME680_data; // 16 bytes
  MS5611_t MS5611_data; // 8 bytes
  ZOE_M8Q_t ZOE_M8Q_data; // 16 bytes
  //BMM150_t BMM150_data;
  TMP75_t TMP75_data; // 4 bytes
  FS3000_t FS3000_data; // 4 bytes
  LWLP5000_t LWLP5000_data; // 8 bytes
  SCD30_t SCD30_data; // 12 bytes
} __attribute__((packed)) PayloadData;

/* END OF NEW STUFF 6-9 */



PayloadData* payload_data = (PayloadData*) malloc(sizeof(PayloadData));

//Address variables:
uint32_t addr = 0; //current address of the flash chip
uint32_t starting_address; //the next available address if there is memory on the flash chip

//char buffers:
char str_to_write[50]; //used for storing floats converted to strings
char file_name1[18]; //used for the first file name
char file_name2[23]; //used for the second file name (to write data that was already on the flash chip)
char c[sizeOfData]; //stores the chars recieved from the master ESP32
float_and_char flash_data; //will store a char array in the same memory location so it can be read as a float for writing floats to the SD card

//Flags:
uint8_t write_data = 0; //writing data to flash chip
uint8_t dump_to_SD_card = 0; //copy the flash chip data to SD card
uint8_t erase_flash_chip = 0; //erase the flash chip
uint8_t flash_has_prev_data = 0; //will cause the flash LED to continually blink if it has previous data on it
uint8_t flash_is_being_written_to = 0; //will cause the flash LED to blink in a slow, rhythmic pattern to indicate that it is being written to
uint8_t wait_five_seconds_to_blink = 0; //used for the flash LED's rhythmic blinking

//Time updating variables:
uint32_t blink_time_prev_data = 0; //the next time to blink the flash LED if flash chip has previous data on it (every 500ms)
uint32_t blink_time_writing_data = 0; //the next time to blink the flash LED while writing to the flash chip (200ms to 5000ms)

#define LED_SD                    (uint8_t) 33
#define LED_FLASH                 (uint8_t) 12

void setup() { //Runs once

  Serial.begin(115200);
  delay(50);

  Serial.println("Hello");

  

  //Setting up LED pins\\-----------------------------------------------------------------------------------------------------------------
  pinMode(LED_SD, OUTPUT); //SD card LED
  pinMode(LED_FLASH, OUTPUT); //flash chip LED

  //Setting up flash chip\\---------------------------------------------------------------------------------------------------------------
  while (!flash.begin()); //Both LEDs stay off and program halts if flash does not begin

  digitalWrite(LED_FLASH, HIGH); //flash chip LED turns on
  
  starting_address = flash.getAddress(sizeof(PayloadData)); //starting address is set to the next available address with the current data size
  if (starting_address) { //if the starting address is not 0:
    flash_has_prev_data = 1; //write a 1 to the previous data flag, this will make the flash chip LED blink every 500ms to indicate it has data on it
    addr = starting_address; //set the current address to the starting address
  }

  Serial.println("Starting address: ");
  Serial.println(starting_address);
  //Setting up SD card\\-------------------------------------------------------------------------------------------------------------------
  SD_SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS); //Set up SD_SPI (HSPI) with the correct pins

  while(!SD.begin(SD_CS, SD_SPI, 10000000)){ //halt the program until the SD card begins and:
    digitalWrite(LED_SD, !digitalRead(LED_SD)); //blink the SD card LED every 100ms
    delay(100);
  }
  
  digitalWrite(LED_SD, HIGH); //turn on the SD card LED if the SD card initializes

  //Setting up this ESP32 as a slave\\------------------------------------------------------------------------------------------------------
  Wire.onReceive(onReceive); //sets up the receiving function
  Wire.onRequest(onRequest); //sets up the requesting function
  Wire.begin(I2C_ESP32_ADDR); //begins I2C as a slave with the given address
  
}

void loop() { //To be run repeatedly.Different flags will be triggered depending on the data recieved and the setup function

  
  //Writing data\\---------------------------------------------------------------------------------------------------------------------------
  if (write_data) { 
    Serial.printf("\nStart writing data at %d\n", millis());
    char* payload_data_8bit = (char*) payload_data;
    if (flash.writeCharArray(addr, payload_data_8bit, sizeof(PayloadData))) {
      Serial.println("Bytes written!");
      Serial.println(addr);
      Serial.println("What was written: ");
      for (int j = 0; j < sizeof(PayloadData); j++) {
        Serial.printf("%X", payload_data_8bit[j]);
      }
      Serial.println(" ");
    } else {
      Serial.println("Bytes not written!");
      Serial.println(addr);
    }
    addr += sizeof(PayloadData);
    write_data = 0;
    Serial.printf("\nEnd writing data at %d\n", millis());
  }

  //Dumping data to SD\\-----------------------------------------------------------------------------------------------------------------------
  if (dump_to_SD_card) {

    int fileCounter = 0;
    
    file = SD.open("/filesystem.txt");
    if (file) {
      Serial.println("filesystem opened");
      while (file.available()) {
        Serial.println("Reading filesystem");
        Serial.println(file.read());
        Serial.println(fileCounter);
        fileCounter++;
      }
      file.close();
    }
    
    file = SD.open("/filesystem.txt", FILE_APPEND);
    if (file) {
      sprintf(file_name1, "/data%d.txt", fileCounter);
      sprintf(file_name2, "/dataextra%d.txt", fileCounter);
      file.write(48);
      file.close();
    }

    digitalWrite(LED_FLASH, HIGH);
    file = SD.open(file_name1, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(fileHeader)) {
      Serial.println("First line written to SD!");
    } else {
      Serial.println("First line didn't write to SD.");
    }

    for (int i = starting_address; i < addr; i += sizeof(PayloadData)) {
      for (int j = 0; j < sizeof(PayloadData) / 4; j++) {
        flash_data.c[0] = flash.readChar(4*j+i);
        flash_data.c[1] = flash.readChar(4*j+1+i);
        flash_data.c[2] = flash.readChar(4*j+2+i);
        flash_data.c[3] = flash.readChar(4*j+3+i);
        dtostrf(flash_data.flt,2,6,str_to_write);
        file.print(str_to_write);
        file.print(",");
      }
      if (file.print("\n")) {
        Serial.println("Packet written!");
        digitalWrite(LED_SD, !digitalRead(LED_SD));
      } else {
        Serial.println("Packet not written!");
      }
    }

    dump_to_SD_card = 0;
    file.close();

    if (starting_address) {
      file = SD.open(file_name2, FILE_WRITE);
      if (file) {
        file.print(fileHeader);
        for (int i = 0; i < starting_address; i += sizeof(PayloadData)) {
          Serial.printf("i: %d\n", i);
          Serial.printf("i < starting_address: %d\n", i < starting_address);
          Serial.printf("starting_address: %d\n", starting_address);
          for (int j = 0; j < sizeof(PayloadData) / 4; j++) {
            Serial.printf("j: %d\n", j);
            flash_data.c[0] = flash.readChar(4*j+i);
            flash_data.c[1] = flash.readChar(4*j+1+i);
            flash_data.c[2] = flash.readChar(4*j+2+i);
            flash_data.c[3] = flash.readChar(4*j+3+i);
            double temp = flash_data.flt;
            file.print(dtostrf(temp,2,6,str_to_write));
            file.print(",");
          }
          if (file.print("\n")) {
          Serial.println("2nd File Packet written!");
          digitalWrite(LED_SD, !digitalRead(LED_SD));
          } else {
          Serial.println("2nd File Packet not written!");
          }
        }
        file.close();
      } else {
        Serial.println("2nd File Didn't Open");
      }
    }
    
    while(1) {
      digitalWrite(LED_SD, LOW);
      delay(5000);
      digitalWrite(LED_SD, HIGH);
      delay(200);
      digitalWrite(LED_SD, LOW);
      delay(200);
      digitalWrite(LED_SD, HIGH);
      delay(200);
    }
  }

  //Erasing the flash chip\\-------------------------------------------------------------------------------------------------------------------
  if (erase_flash_chip) {
    Serial.println("Erasing flash chip...");
    digitalWrite(LED_FLASH, LOW);
    flash_has_prev_data = 0;
    erase_flash_chip = 0;
    if (flash.eraseChip()) {
      digitalWrite(LED_FLASH, HIGH);
      Serial.println("Flash chip erased");
      addr = 0;
      starting_address = 0;
    } else {
      Serial.println("Failure");
    }
  }

  //Blinking the flash LED to indicate writing\\------------------------------------------------------------------------------------------------
  if (flash_is_being_written_to) {
    if ((millis() - 200) > blink_time_writing_data) {
      blink_time_writing_data = millis();
      switch (wait_five_seconds_to_blink) {
        case 1:
          digitalWrite(LED_FLASH, LOW);
          break;
        case 2:
          digitalWrite(LED_FLASH, HIGH);
          break;
        case 3:
          digitalWrite(LED_FLASH, LOW);
          break;
        case 4:
          digitalWrite(LED_FLASH, HIGH);
          break;
        case 5:
          digitalWrite(LED_FLASH, LOW);
          blink_time_writing_data += 5200;
          wait_five_seconds_to_blink = -1;
          break;
      }
      wait_five_seconds_to_blink++;
    }
  }


  //Blinking the flash LED to indicate that there is old data\\---------------------------------------------------------------------------------
  if (flash_has_prev_data && !flash_is_being_written_to) {
    if ((millis() - 500) > blink_time_prev_data) {
      digitalWrite(LED_FLASH, !digitalRead(LED_FLASH));
      blink_time_prev_data = millis();
    }

  }
}


void onRequest() { 
}


void onReceive(int len){
  Serial.printf("\nRecieving data starts %d, ", millis());
  Serial.printf("onReceive[%d]: ", len);
  uint8_t* payload_data_8bit = (uint8_t*) payload_data;
  int i = 0;

  
  if (len == 1) {
    Serial.printf("\nBeginning to dump to SD Card\n");
    dump_to_SD_card = 1;
    flash_is_being_written_to = 0;
    while(Wire.available()) {
      Wire.read();
    }
  } else if (len == 2) {
    erase_flash_chip = 1;
    while(Wire.available()) {
      Wire.read();
    }
  } else {
    write_data = 1;
    flash_is_being_written_to = 1;
    while(Wire.available()){
      payload_data_8bit[i] = Wire.read();
      Serial.printf("%X", (int)payload_data_8bit[i]);
      i++;
    }
    Serial.printf("recieving data ends %d\n", millis());
  }
  Serial.println();
}
