//Payload Model Ouroboros / Osprey Level 2
//Objective: Transmitting and storing sensor data

//Sensors:
// Magsensor (BMM150)
// IMU (Adafruit ISM330DHCX)
// Baro/Humidity/Temp/Gas (Adafruit_BME680)
// GPS (GNSS ZOE-Click)
// IMU 2 (MPU6050)
// 

//Radio:
// Lora SX1280 radios (DLP-RFS1280)

// Libraries:
// Radio: //github.com/StuartsProjects/SX12XX-LoRa
// Magsensor: search for Adafruit_LIS2MDL in arduino library manager,
// IMU: search for ISM330DHCX in arduino library manager,
// BME: search for Adafruit_BME680 in arduino library manager,
// GPS: //librarymanager/All#SparkFun_u-blox_GNSS,

// Changing Transmission settings 
// 1. using the settings.h file, radio transmission elements such as bandwidth, coding rate and spreading factor can be manipulated to change 
//    signal range and packet transmission speed. Accpeted values for these parameters can be found here: github.com/StuartsProjects/SX12XX-LoRa/blob/master/src/SX128XLT_Definitions.h
//    For more information on the effects of each of these settings: medium.com/home-wireless/testing-lora-radios-with-the-limesdr-mini-part-2-37fa481217ff                                
// 2. Changing the delay in the settings file does not work. Instead, change the delay at the bottom of the main code that breaks up the transmissions. This value cannot be set to least than 10ms. Ensure that the settings in step 1 allow for the transmission speed you coded for. 
  
 
// Adding additional sensors          
// 1. Install libraries and set up device         
// 2. Define variables that need to be sent             
// 3. Assign values to these variables               
// 4. Send values by adding them to the buffer (Using LT.writeFloat or LT.writeUint8)

//libraries\\---------------------------------------------------------------------------------------------------
#include <SPI.h> //used for radio, GPS, MicroSD, GPS and flash
#include <Wire.h> //used for i2c sensors 
#include <Adafruit_Sensor.h> //For all Adafruit sensors
#include "Adafruit_BME680.h" //libraries needed for BME (Adafruit_BME680)
#include <Adafruit_ISM330DHCX.h> //libraries needed for IMU (ISM330DHCX)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //library needed for GPS(GNSS ZOE-Click)
#include <SX128XLT.h>  //libraries needed for radio (DLP-RFS1280)
#include "Settings_ESP.h" //libraries needed for frequencies, LoRa settings etc for ESP32
#include "DFRobot_BMM150.h" //libraries needed for BMM150 Breakout (DFRobot SEN0419)
#include <Adafruit_TMP117.h> //libraries needed for TMP117 Breakout (Adafruit TMP117)
#include <Adafruit_MPU6050.h> //libraries needed for the 2nd IMU (MPU6050)
#include <SparkFun_FS3000_Arduino_Library.h> //libraries needed for the velocity sensor (FS3000)
#include <DFRobot_LWLP.h> //libraries needed for the differential pressure sensor (DF Robot SEN0343)

//Slave ESP32 Set up\\-------------------------------------------------------------------------------
#define I2C_ESP32_ADDR 0x11 //0x11 is the defined slave address for the ESP32 slave, can be changed
int dump_to_SD_card = 0; //flag to send the command to dump the flash data to the SD card
int data_log = 0; //flag to start logging data
int erase_flash_chip = 0;
union float_and_char { //stores a float and char array in the same memory location, used to send 32-bit float values as 4 8-bit char values over I2C
  float flt; // 4 bytes
  char c[4]; // 1 byte * 4
};
float_and_char sensor_data[30]; //array that stores all of our sensor data before sending

//SPI Set up\\---------------------------------------------------------------------------------------
SPIClass SPI2(HSPI); //setting up HSPI to work on ESP32 in conjuction with VSPI

//Pin Set up\\---------------------------------------------------------------------------------------------------
//most Pins are already defined in the library
uint8_t cs_pin = 15; //GPIO pin 15 has been selected for chip select the GPS
const int led_pin = 33; //GPIO pin 33 connected to LED 
const int button_pin = 32; //GPIO pin 32 connected to button
const int dump_to_SD_button = 35; //GPIO pin 35 connected to SD dump button, for now it also starts data logging
const int erase_flash_chip_button = 34; //GPIO pin 34 connected to erase flash chip button

//Defining Library functions to be called\\---------------------------------------------------------------------------------------------------
SFE_UBLOX_GNSS myGNSS; //Create the library class instance called myGNSS
SX128XLT LT; //Create the library class instance called LT for radio
Adafruit_ISM330DHCX ism330dhcx; //Create the library class instance called ism330dhcx
Adafruit_BME680 bme; //Create the library class instance called bme
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4); //Create the library class instance bmm150 for the magnometer
Adafruit_TMP117  tmp117; //Create the library class instance tmp117 for the high accuracy temperature sensor
Adafruit_MPU6050 mpu; //Create the library class instance mpu for the MPU6050 IMU
FS3000 fs; //Create the library class instance fs for the FS3000 velocity sensor
DFRobot_LWLP lwlp; //Create the library class instance lwlp for the differential pressure sensor

//Needed for radio to work\\---------------------------------------------------------------------------------------------------
uint8_t TXPacketL; //successful transmission
uint8_t start_buff[] = "a"; //low memory transmit mode makes the smallest buffer

//BME set up\\-------------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
uint32_t time_BME_data_is_ready = 0;

void setup() 
{
    Serial.begin(115200);
    Wire.begin(); //For i2c 
    SPI.begin(); //For VSPI (Used for Radio)*/
    SPI2.begin(); //FOR HSPI (Used for GPS)

    //Setting pins\\-------------------------------------------------------------------------------------------------------------
    pinMode(led_pin, OUTPUT);
    pinMode(button_pin, INPUT);
    pinMode(dump_to_SD_button, INPUT);
    pinMode(erase_flash_chip_button, INPUT);

    //connecting to IMU
    while (!ism330dhcx.begin_I2C())  { 
        Serial.println("Failed to communicate with IMU.");
        flash(50);
    }
    flash(500);

    //connecting to BME 
    while (!bme.begin())  {
        Serial.println("Failed to communicate with Barometer.");
        flash(50);
    }
    //BME settings (think so)
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
    time_BME_data_is_ready = bme.beginReading(); // Start the reading so it's ready when data logging begins
    flash(500);

    //connecting to magnetometer 
    while(bmm150.begin()) { //returns true if it does not initialize
      Serial.println("Failed to communicate with BMM150 (magnometer).");
      flash(50);
    }
    flash(500);
    //BMM150 settings
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY); //can change from high accuracy to enhanced or regular, probably would decrease power consumption
    bmm150.setMeasurementXYZ();

    //connecting to high accuracy temp sensor
    while (!tmp117.begin()) {
      Serial.println("Failed to communicate with TMP117 (temperature sensor)");
      flash(50);
    }
    flash(500);

    //connecting to MPU6050
    while (!mpu.begin()) {
      Serial.println("Failed to communicate with MPU6050.");
      flash(50);
    }
    flash(500);
    //MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    //connecting to FS3000
    while (!fs.begin()) {
      Serial.println("Failed to communicate with FS3000");
      flash(50);
    }
    flash(500);

    //connecting to differential pressure sensor
    while (lwlp.begin() != 0) {
      Serial.println("Failed to communicate with differential pressure sensor/");
      flash(50);
    }
    flash(500);

    //connecting to GPS
    while (myGNSS.begin(SPI2, cs_pin, 5500000) == false)  {
        Serial.println("Failed to communicate with GPS.");
        flash(50);
    }
    myGNSS.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    myGNSS.setNavigationFrequency(5);
    flash(500);

    //conneting to radio
    while (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))  {
        Serial.println("Failed to communicate with Radio.");
        flash(50);
    }
    
    LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate); //sets up necessary settings for the radio
    flash(500); 
    digitalWrite(led_pin, HIGH); //Turns on LED if all connections are working!
    Serial.println("All Sensors Initialized ");
}


void loop() {
  uint8_t len;                                         // length of packet sent
  LT.startWriteSXBuffer(0);                 //start the write packet to buffer process
  LT.writeBuffer(start_buff, sizeof(start_buff));
  /////Building the array for data logging and the buffer for radio transmission
  sensor_data[0].flt = (float) millis() / 1000.0; //convert ms to s
  //LT.writeFloat(sensor_data[0]);
  ///Getting data from sensors///////
  imu_data();
  //baro_data();
  magData();
  TMPData();
  MPUData();
  FSData();
  diff_pressureData();
  gpsData();
  len = LT.endWriteSXBuffer();
  ////sending and saving data////////
  send_radio(len);
  //sendEsp();
  Serial.println("sent");
  delay(500);
}

void send_radio(uint8_t len){
  Serial.flush();                           // waits until the transmission is finished

  if (TXPacketL = LT.transmitSXBuffer(0, len, 10000, TXpower, 0)) {  // The transmitting magic, a little underwhelming. will return packet length sent if OK, otherwise 0 if transmit error
      Serial.println(len);
  }
  else {
      Serial.println(F("Transmission failed"));
  }
}

void sendEsp(){
          /*buttonInput();
        if (erase_flash_chip) {
          eraseFlashChip();
          erase_flash_chip = 0;
        }
        if (dump_to_SD_card) {
          dumpToSDCard();
          while(1); //for now, the program freezes after dumping to SD... will be changed later
        }
        if (data_log) {
          dataLog();
        }*/

     /* SENDING DATA TO SLAVE ESP32 */
  /*Wire.beginTransmission(I2C_ESP32_ADDR);
  for (int i = 0; i < 30; i++) {
    Serial.printf("sensor_data[%d]: %f\n", i, sensor_data[i].flt);
    for (int j = 3; j >= 0; j--) {
      Wire.write(sensor_data[i].c[j]);
    }
  }
  Serial.println("");
  Wire.endTransmission(true);*/
  
}


//Function to read the button input
void buttonInput() {
  if (digitalRead(dump_to_SD_button)) { //if the SD button is pressed, either start data logging or dump the data to the SD by setting the appropriate flag
    if (!(data_log) && !(dump_to_SD_card)) {
      data_log = 1;
    } else {
      dump_to_SD_card = 1;
      data_log = 0;
    }
    while(digitalRead(dump_to_SD_button)); //wait for button to be released
  }
  
  if (digitalRead(erase_flash_chip_button)) { //if the erase button is pressed, set the erase_flash_chip flag to 1
    erase_flash_chip = 1;
    while(digitalRead(erase_flash_chip_button)); //wait for button to be released
  }
}

//Function to send the command to the slave ESP32 to erase the flash chip
void eraseFlashChip() {
  Wire.beginTransmission(I2C_ESP32_ADDR);
  Wire.write(0);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("Erasing chip...");
  delay(30000);
  Serial.println("All done.");
}

//Function to send the command to the slave ESP32 to dump the flash chip data to the SD card
void dumpToSDCard() {
  Wire.beginTransmission(I2C_ESP32_ADDR);
  Wire.write(0);
  Wire.endTransmission(true);
}


//Function to flash LED 
//Pre: int millis for blink
//Post: None
//Blinks the LED on and off for the wait time
void flash(int wait){
  digitalWrite(led_pin, HIGH);
  delay(wait);
  digitalWrite(led_pin, LOW);
  delay(wait);
}

//Function to print IMU Data to the serial port 
void imu_data(){
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  sensor_data[1].flt = (float) temp.temperature;
  sensor_data[2].flt = (float) accel.acceleration.x;
  sensor_data[3].flt = (float) accel.acceleration.y;
  sensor_data[4].flt = (float) accel.acceleration.z;
  sensor_data[5].flt = (float) gyro.gyro.x;
  sensor_data[6].flt = (float) gyro.gyro.y;
  sensor_data[7].flt = (float) gyro.gyro.z;

  
  LT.writeFloat(sensor_data[1].flt);
  LT.writeFloat(sensor_data[2].flt);
  LT.writeFloat(sensor_data[3].flt);
  LT.writeFloat(sensor_data[4].flt);
  LT.writeFloat(sensor_data[5].flt);
  LT.writeFloat(sensor_data[6].flt);
  LT.writeFloat(sensor_data[7].flt);

//  Serial.print("IMU: ");
//  Serial.print(sensor_data[1].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[2].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[3].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[4].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[5].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[6].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[7].flt);
//  Serial.print(",");
//  Serial.println();
  
} 


//Function to print Barometer Data to the serial port 
void baro_data(){
    if (! bme.performReading()) { //needed for BME not sure why?
    return;
  }
  
  if (millis() > time_BME_data_is_ready) { //checks if data is ready, otherwise does not change BME's sensor_data values
    bme.endReading();

    sensor_data[8].flt = (float) bme.temperature;
    sensor_data[9].flt = (float) bme.pressure / 100.0;
    sensor_data[10].flt = (float) bme.humidity;
    sensor_data[11].flt = (float) bme.gas_resistance / 1000.0;
    sensor_data[12].flt = (float) bme.readAltitude(SEALEVELPRESSURE_HPA);
  
    LT.writeFloat(sensor_data[8].flt);
    LT.writeFloat(sensor_data[9].flt);
    LT.writeFloat(sensor_data[10].flt);
    LT.writeFloat(sensor_data[11].flt);
    LT.writeFloat(sensor_data[12].flt);
//    
//    Serial.print("BME: ");
//    Serial.print(sensor_data[8].flt); //BME Temp
//    Serial.print(",");
//    Serial.print(sensor_data[9].flt); //BMe Pressure 
//    Serial.print(",");
//    Serial.print(sensor_data[10].flt); //BME Humidity
//    Serial.print(",");
//    Serial.print(sensor_data[11].flt); //BME Gas resistance 
//    Serial.print(",");
//    Serial.print(sensor_data[12].flt); //BME alt
//    Serial.println(",");
  
    time_BME_data_is_ready = bme.beginReading(); //begins the next reading and returns the next time the BME data will be ready
  }

  
}

//Function to print Magnetometer Data to the serial port *** NEEDS TO BE CHANGED ***
void magData(){
  sBmm150MagData_t magData = bmm150.getGeomagneticData();

  sensor_data[12].flt = (float) magData.x;
  sensor_data[13].flt = (float) magData.y;
  sensor_data[14].flt = (float) magData.z;
  sensor_data[15].flt = (float) bmm150.getCompassDegree();

  LT.writeFloat(sensor_data[13].flt);
  LT.writeFloat(sensor_data[14].flt);
  LT.writeFloat(sensor_data[15].flt);
  LT.writeFloat(sensor_data[16].flt);
  
//  Serial.print("Magnometer: ");
//  Serial.print(sensor_data[13].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[14].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[15].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[16].flt);
//  Serial.println(",");

}

void TMPData(){
  /* TMP117 DATA */
  sensors_event_t tempTMP117; // create an empty event to be filled
  tmp117.getEvent(&tempTMP117); //fill the empty event object with the current measurements

  sensor_data[16].flt = (float) tempTMP117.temperature;
  LT.writeFloat(sensor_data[17].flt);
//  Serial.print("TMP117: ");
//  Serial.print(sensor_data[17].flt);
//  Serial.println(",");
}

void MPUData(){
   /* MPU6050 DATA */
  sensors_event_t a, g, tempMPU;
  mpu.getEvent(&a, &g, &tempMPU);

  sensor_data[18].flt = (float) a.acceleration.x;
  sensor_data[19].flt = (float) a.acceleration.y;
  sensor_data[20].flt = (float) a.acceleration.z;
  sensor_data[21].flt = (float) g.gyro.x;
  sensor_data[22].flt = (float) g.gyro.y;
  sensor_data[23].flt = (float) g.gyro.z;
  sensor_data[24].flt = (float) tempMPU.temperature;
  
  LT.writeFloat(sensor_data[18].flt);
  LT.writeFloat(sensor_data[19].flt);
  LT.writeFloat(sensor_data[20].flt);
  LT.writeFloat(sensor_data[21].flt);
  LT.writeFloat(sensor_data[22].flt);
  LT.writeFloat(sensor_data[23].flt);
  LT.writeFloat(sensor_data[24].flt);

//  Serial.print("IMU: ");
//  Serial.print(sensor_data[18].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[19].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[20].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[21].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[22].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[23].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[24].flt);
//  Serial.println(",");
}

void FSData(){
    /* FS3000 DATA */
  sensor_data[24].flt = fs.readMetersPerSecond();; //already a float
  LT.writeFloat(sensor_data[25].flt);
//  Serial.print("Velocity: ");
//  Serial.print(sensor_data[25].flt);
//  Serial.println(",");
}

void diff_pressureData(){
  /* DIFFERENTIAL PRESSURE SENSOR DATA */
  DFRobot_LWLP::sLwlp_t data_1;
  data_1 = lwlp.getData();

  sensor_data[25].flt = (float) data_1.temperature;
  sensor_data[26].flt = (float) data_1.presure; //typo was in the example code, works
  LT.writeFloat(sensor_data[25].flt);
  LT.writeFloat(sensor_data[26].flt);
//  Serial.print("differential pressure: ");
//  Serial.print(sensor_data[25].flt);
//  Serial.print(",");
//  Serial.print(sensor_data[26].flt);
//  Serial.println(",");
}

//Function to print GPS Data to the serial port 
void gpsData(){
  
  sensor_data[27].flt = (float) myGNSS.getLatitude()/ 10000000.0;
  sensor_data[28].flt = (float) myGNSS.getLongitude() / 10000000.0;
  sensor_data[29].flt = (float) myGNSS.getAltitude() / 10000000.0;
  //sensor_data[30].flt = (uint_8) myGNSS.getSIV();

  
  LT.writeFloat(sensor_data[27].flt);
  LT.writeFloat(sensor_data[28].flt);
  LT.writeFloat(sensor_data[29].flt);
  //LT.writeUint8(sensor_data[30]);
//
//  Serial.print("GPS: ");
//  Serial.print(sensor_data[27].flt); //GPS Data Latitude
//  Serial.print(",");
//  Serial.print(sensor_data[28].flt); //GPS Data Longitude
//  Serial.print(",");
//  Serial.print(sensor_data[29].flt); //GPS SIV
//  //Serial.print(",");
//  //Serial.print(SIV); //GPS SIV
//  Serial.println(",");
}
