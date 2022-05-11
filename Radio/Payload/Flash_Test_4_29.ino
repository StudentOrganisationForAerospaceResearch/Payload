#include<SPIMemory.h>
#include "SPI.h"
#include "SD.h"
#include "Wire.h"
#include "FS.h"

#define I2C_ESP32_ADDR 0x11
#define SD_MISO     25
#define SD_MOSI     13
#define SD_SCLK     14
#define SD_CS       15

SPIClass SPI2(HSPI);

union float_and_char {
  float flt;
  char c[4];
};

uint32_t addr = 0;
int startingAddress = 0;
char strToWrite[12];
char fileName1[18];
char fileName2[23];
float_and_char flashData;

File file;
SPIFlash flash(5);

int writeData = 0;
int dumpToSDCard = 0;
int eraseFlashChip = 0;
int flashGreenLED = 0;
uint32_t flashTime = 0;

const int sizeOfData = 120;
const int sizeOfDataInFloat = 30;

char c[sizeOfData] = {' '};


void setup() {
  Serial.begin(115200);
  delay(50); //Time for terminal to get connected

  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);

  if (flash.begin()) {
    digitalWrite(32, HIGH);
  }

  if (flash.getAddress(1)) {
    flashGreenLED = 1;
    startingAddress = flash.getAddress(1);
    Serial.print("Starting address: ");
    Serial.print(itoa(startingAddress,strToWrite,10));
    Serial.print("\n");
    addr = startingAddress;
  }
  
  SPI2.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  while(!SD.begin(SD_CS, SPI2, 10000000)){
    Serial.println("Card Mount Failed");
    delay(100);
    digitalWrite(33, !digitalRead(33));
  }
  
  Serial.println("Card Mount Success!");
  digitalWrite(33, HIGH);

  Wire.onReceive(onMyReceive);
  Wire.onRequest(onMyRequest);
  Wire.begin((uint8_t)I2C_ESP32_ADDR);
  
}

void loop() {
   if (writeData) {
    if (flash.writeCharArray(addr, c, sizeOfData)) {
      Serial.println("Bytes written!");
      Serial.println(addr);
      Serial.println("What was written: ");
      for (int j = 0; j < sizeOfData; j++) {
        Serial.printf("%X", c[j]);
      }
      Serial.println(" ");
    } else {
      Serial.println("Bytes not written!");
      Serial.println(addr);
    }
    addr += sizeOfData;
    writeData = 0;
  }

  if (dumpToSDCard) {

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
      sprintf(fileName1, "/data%d.txt", fileCounter);
      sprintf(fileName2, "/dataextra%d.txt", fileCounter);
      file.write(48);
      file.close();
    }

    digitalWrite(32, HIGH);
    file = SD.open(fileName1, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print("time,temperature,accel-x,accel-y,accel-z,gyro-x,gyro-y,gyro-z,temperature,pressure,humidity,gas-resistance,mag-x,mag-y,mag-z,compass,temperature,accel-x,accel-y,accel-z,gyro-x,gyro-y,gyro-z,temperature,velocity,temperature,differential pressure,lat,long,altitude\n")) {
      Serial.println("First line written to SD!");
    } else {
      Serial.println("First line didn't write to SD.");
    }

    for (int i = startingAddress; i < addr; i += sizeOfData) {
      for (int j = 0; j < sizeOfDataInFloat; j++) {
        flashData.c[3] = flash.readChar(4*j+i);
        flashData.c[2] = flash.readChar(4*j+1+i);
        flashData.c[1] = flash.readChar(4*j+2+i);
        flashData.c[0] = flash.readChar(4*j+3+i);
        dtostrf(flashData.flt,2,6,strToWrite);
        file.print(strToWrite);
        file.print(",");
      }
      if (file.print("\n")) {
        Serial.println("Packet written!");
      } else {
        Serial.println("Packet not written!");
      }
    }

    dumpToSDCard = 0;
    file.close();

    if (startingAddress) {
      file = SD.open(fileName2, FILE_WRITE);
      if (file) {
        file.print("time,temperature,accel-x,accel-y,accel-z,gyro-x,gyro-y,gyro-z,temperature,pressure,humidity,gas-resistance,mag-x,mag-y,mag-z,compass,temperature,accel-x,accel-y,accel-z,gyro-x,gyro-y,gyro-z,temperature,velocity,temperature,differential pressure,lat,long,altitude\n");
        for (int i = 0; i < startingAddress - 1; i += sizeOfData) {
          for (int j = 0; j < sizeOfDataInFloat; j++) {
            flashData.c[3] = flash.readChar(4*j);
            flashData.c[2] = flash.readChar(4*j+1);
            flashData.c[1] = flash.readChar(4*j+2);
            flashData.c[0] = flash.readChar(4*j+3);
            file.print(dtostrf(flashData.flt,2,6,strToWrite));
            file.print(",");
          }
          if (file.print("\n")) {
          Serial.println("2nd File Packet written!");
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
      digitalWrite(33, LOW);
      delay(200);
      digitalWrite(33, HIGH);
      delay(200);
      digitalWrite(33, LOW);
      delay(200);
      digitalWrite(33, HIGH);
      delay(200);
      digitalWrite(33, LOW);
      delay(5000);
    }
  }

  if (eraseFlashChip) {
    Serial.println("Erasing flash chip...");
    digitalWrite(32, LOW);
    flashGreenLED = 0;
    eraseFlashChip = 0;
    if (flash.eraseChip()) {
      digitalWrite(32, HIGH);
      Serial.println("Flash chip erased");
      addr = 0;
      startingAddress = 0;
    } else {
      Serial.println("Failure");
    }
  }

  if (flashGreenLED) {
    if ((millis() - 500) > flashTime) {
      digitalWrite(32, !digitalRead(32));
      flashTime = millis();
    }
  }
 
}

void onMyRequest() {
  
}


void onMyReceive(int len){
  Serial.printf("onReceive[%d]: ", len);
  int i = 0;
  while(Wire.available()){
    c[i] = Wire.read();
    Serial.write((int)c[i]);
    i++;
  }
  
  if (len == 1) {
    Serial.printf("\nBeginning to dump to SD Card\n");
    dumpToSDCard = 1;
  } else if (len == 2) {
    eraseFlashChip = 1;
  } else {
    writeData = 1;
  }
  Serial.println();
}

void openFile (fs::FS &fs, const char * path) {
  file = fs.open(path, FILE_WRITE);
}
