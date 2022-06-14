//Payload Model Tsukuyomi / Osprey 1
//Master ESP32 Software
//Objective: Polling and transmitting sensor data

// Sensors:
// ISM330DHCX (IMU1)
// BME680 (Barometer/humidity/temperature/gas)
// ZOE-M8Q (GPS)
// MPU6050 (IMU2)
// BMM150 (Magnometer)
// TMP117 (High accuracy temperature)
// FS3000 (Velocity)
// LWLP5000 (Differential pressure)

// Radio:
// DLP-RFS1280 (Lora SX1280 radios)

// Libraries:
// Radio: github.com/StuartsProjects/SX12XX-LoRa
// ISM330DHCX (IMU1): search for Adafruit_LSM6DS library in Arduino library manager.
// BME680 (Barometer/humidity/temperature/gas): search for Adafruit_BME680 in Arduino library manager.
// ZOE-M8Q (GPS): search for Sparkfun u-blox GNSS Arduino Library in Arduino library manager.
// MPU6050 (IMU2): search for Adafruit MPU6050 in Arduino library manager.
// BMM150 (Magnometer): search for DFRobot_BMM150 in Arduino library manager.
// TMP117 (Temperature): search for Adafruit TMP117 in Arduino library manager.
// FS3000 (Velocity): search for Sparkfun_FS3000_Arduino_Library in Arduino library manager.
// LWLP5000 (Differential pressure): search for DFRobot_LWLP in Arduino library manager.

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
//#include "Adafruit_BME680.h" //libraries needed for BME (Adafruit_BME680)
#include <Adafruit_ISM330DHCX.h> //libraries needed for IMU (ISM330DHCX)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //library needed for GPS(GNSS ZOE-Click)
#include <SX128XLT.h>  //libraries needed for radio (DLP-RFS1280)
#include "Settings_ESP.h" //libraries needed for frequencies, LoRa settings etc for ESP32
//#include "DFRobot_BMM150.h" //libraries needed for BMM150 Breakout (DFRobot SEN0419)
#include <Temperature_LM75_Derived.h> //libraries needed for TMP75
#include <Adafruit_MPU6050.h> //libraries needed for the 2nd IMU (MPU6050)
#include <SparkFun_FS3000_Arduino_Library.h> //libraries needed for the velocity sensor (FS3000)
#include <DFRobot_LWLP.h> //libraries needed for the differential pressure sensor (DF Robot SEN0343)
#include <MS5611.h> //libraries needed for the MS5611 barometer

//Slave ESP32 Set up\\-------------------------------------------------------------------------------
#define I2C_ESP32_ADDR 0x11 //0x11 is the defined slave address for the ESP32 slave, can be changed
uint8_t dump_to_SD_card = 0; //flag to send the command to dump the flash data to the SD card
uint8_t data_log = 0; //flag to start logging data
uint8_t erase_flash_chip = 0; //flag to erase the flash chip
union float_and_char { //stores a float and char array in the same memory location, used to send 32-bit float values as 4 8-bit char values over I2C
  float flt; // 4 bytes
  char c[4]; // 1 byte * 4
};
float_and_char sensor_data[12]; //array that stores all of our sensor data before sending



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


/*typedef struct {
  float temperature;
  float humidity;
  float pressure;
  float gas_resistance;
} BME680_t;*/

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


typedef struct {
  float time_stamp;
  IMU_6DOF_t ISM330DHCX_data;
  //IMU_6DOF_t MPU6050_data;
  TMP75_t TMP75_data;
  FS3000_t FS3000_data;
  //LWLP5000_t LWLP5000_data;
  //ZOE_M8Q_t ZOE_M8Q_data;
  MS5611_t MS5611_data;
} __attribute__((packed)) PayloadData;

PayloadData* payload_data = (PayloadData*) malloc(sizeof(PayloadData));

/* END OF NEW STUFF 6-9 */


//SPI Set up\\---------------------------------------------------------------------------------------
SPIClass SPI2(HSPI); //setting up HSPI to work on ESP32 in conjuction with VSPI

//Pin Set up\\---------------------------------------------------------------------------------------------------
//most Pins are already defined in the library
uint8_t cs_pin = 15; //GPIO pin 15 has been selected for chip select the GPS
const uint8_t LED_PIN = 33; //GPIO pin 33 connected to LED 
const uint8_t MOSFET_PIN = 4; //GPIO pin 4 connected to the fan's MOSFET
//const uint8_t erase_flash_chip_button = 32; //GPIO pin 32 connected to erase flash chip button

//Defining Library functions to be called\\---------------------------------------------------------------------------------------------------
//SFE_UBLOX_GNSS ZOE_M8Q_sensor; //Create the library class instance called myGNSS
SX128XLT LT; //Create the library class instance called LT for radio
Adafruit_ISM330DHCX ISM330DHCX_sensor; //Create the library class instance called ism330dhcx
//Adafruit_BME680 bme; //Create the library class instance called bme
//DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4); //Create the library class instance bmm150 for the magnometer
//Adafruit_MPU6050 MPU6050_sensor; //Create the library class instance mpu for the MPU6050 IMU
FS3000 FS3000_sensor; //Create the library class instance fs for the FS3000 velocity sensor
DFRobot_LWLP LWLP5000_sensor; //Create the library class instance lwlp for the differential pressure sensor
Generic_LM75_12Bit TMP75_sensor; //Creat the library class instance TMP75_sensor for the temperature sensor
MS5611 MS5611_sensor(0x77);


//Needed for radio to work\\---------------------------------------------------------------------------------------------------
uint8_t TXPacketL; //successful transmission
uint8_t start_buff[] = "a"; //low memory transmit mode makes the smallest buffer*/

//BME set up\\-------------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)
uint32_t time_BME_data_is_ready = 0;

void setup() 
{
    Serial.begin(115200);
    delay(3000);
    Wire.begin(); //For i2c 
    SPI.begin(); //For VSPI (Used for Radio)
    SPI2.begin(); //FOR HSPI (Used for GPS)

    //Setting pins\\-------------------------------------------------------------------------------------------------------------
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOSFET_PIN, OUTPUT);
    //pinMode(data_log_button, INPUT);
    //pinMode(erase_flash_chip_button, INPUT);

    //connecting to IMU
    while (!ISM330DHCX_sensor.begin_I2C())  { 
        Serial.println("Failed to communicate with IMU.");
        flash(50);
    }
    flash(500);

    //connecting to BME 
    /*while (!bme.begin())  {
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
    */
    /*
    //connecting to magnetometer 
    while(bmm150.begin()) { //returns true if it does not initialize
      Serial.println("Failed to communicate with BMM150 (magnometer).");
      flash(50);
    }
    flash(500);
    //BMM150 settings
    /*bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY); //can change from high accuracy to enhanced or regular, probably would decrease power consumption
    bmm150.setMeasurementXYZ();*/
    

    //connecting to MPU6050
    //while (!MPU6050_sensor.begin()) {
    //  Serial.println("Failed to communicate with MPU6050.");
    //  flash(50);
    //}
    flash(500);
    //MPU6050 settings
    //MPU6050_sensor.setAccelerometerRange(MPU6050_RANGE_8_G);
    //MPU6050_sensor.setGyroRange(MPU6050_RANGE_500_DEG);
   // MPU6050_sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);

    //connecting to FS3000
    /*
    while (!FS3000_sensor.begin()) {
      Serial.println("Failed to communicate with FS3000");
      flash(50);
    }
    flash(500); */

    //connecting to differential pressure sensor
    /*while (LWLP5000_sensor.begin() != 0) {
      Serial.println("Failed to communicate with differential pressure sensor/");
      flash(50);
    }
    flash(500);*/

    //connecting to GPS
    /*while (ZOE_M8Q_sensor.begin(SPI2, cs_pin, 5500000) == false)  {
        Serial.println("Failed to communicate with GPS.");
        flash(50);
    }
    ZOE_M8Q_sensor.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
    ZOE_M8Q_sensor.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    ZOE_M8Q_sensor.setAutoPVT(true); //Automatically updates PVT
    flash(500);
*/
    //connecting to MS5611
    while (!MS5611_sensor.begin()) {
      Serial.println("MS5611 failed to communicate.");
      flash(50);
    }
    flash(500);

    MS5611_sensor.setOversampling(OSR_ULTRA_LOW);

    //conneting to radio
    
   while (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))  {
        Serial.println("Failed to communicate with Radio.");
        flash(50);
    }
    
    LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate); //sets up necessary settings for the radio
    flash(500); 
    digitalWrite(LED_PIN, HIGH); //Turns on LED if all connections are work ing!
    Serial.println("All Sensors Initialized ");

    delay(30000);
}

// TESTING TESTING TESTING TESTING TESTING TESTING TESTING
uint32_t last_time_data_was_logged = 0;


void loop() {
  if (last_time_data_was_logged <= millis() - 200) { // Currently sends data through radio 5 times/second.
    last_time_data_was_logged = millis();
    dataLog();
  }

}


//Function to read the button input
/*
void buttonInput() {
  if (digitalRead(data_log_button)) { //if the data log button is pressed, either start data logging or dump the data to the SD by setting the appropriate flag
    if (!(data_log) && !(dump_to_SD_card)) {
      data_log = 1;
    } else {
      dump_to_SD_card = 1;
      data_log = 0;
    }
    while(digitalRead(data_log_button)); //wait for button to be released
    delay(500); //Just in case the button vibrates and gets pressed multiple times
  }
  
  if (!digitalRead(erase_flash_chip_button)) { //if the erase button is pressed, set the erase_flash_chip flag to 1
    erase_flash_chip = 1;
    while(!digitalRead(erase_flash_chip_button)); //wait for button to be released
    delay(500); //Just in case the button vibrates and gets pressed multiple times
  }
}
*/

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

//Function to collect sensor data and send it to the slave ESP32 to write to the flash chip
void dataLog() {

  //Serial.println("");
  /* TIME DATA */
  sensor_data[0].flt = (float) millis() / 1000.0; //convert ms to s
  /* END OF TIME DATA */
  
  /* ISM330DHCX DATA */ //Serial.printf("ISM330 starts: %d, ", millis());
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t tempISM330;
  ISM330DHCX_sensor.getEvent(&accel, &gyro, &tempISM330);
  
  sensor_data[1].flt = (float) tempISM330.temperature;
  sensor_data[2].flt = (float) accel.acceleration.x;
  sensor_data[3].flt = (float) accel.acceleration.y;
  sensor_data[4].flt = (float) accel.acceleration.z;
  sensor_data[5].flt = (float) gyro.gyro.x;
  sensor_data[6].flt = (float) gyro.gyro.y;
  sensor_data[7].flt = (float) gyro.gyro.z;
  /* END OF ISM330DHCX DATA */ //Serial.printf("ISM330 ends: %d\n", millis());

  /* BME680 DATA */ //Serial.printf("BME starts: %d, ", millis());
 /* if (millis() > time_BME_data_is_ready) { //checks if data is ready, otherwise does not change BME's sensor_data values
    bme.endReading();
    
    sensor_data[8].flt = (float) bme.temperature;
    sensor_data[9].flt = (float) bme.pressure / 100.0;
    sensor_data[10].flt = (float) bme.humidity;
    sensor_data[11].flt = (float) bme.gas_resistance;

    time_BME_data_is_ready = bme.beginReading(); //begins the next reading and returns the next time the BME data will be ready
  }
  /* END OF BME680 DATA */ //Serial.printf("BME ends: %d\n", millis());

  /* BMM150 DATA */ //Serial.printf("BMM starts: %d, ", millis());
  /*
  sBmm150MagData_t magData = bmm150.getGeomagneticData();

  sensor_data[12].flt = (float) magData.x;
  sensor_data[13].flt = (float) magData.y;
  sensor_data[14].flt = (float) magData.z;
  sensor_data[15].flt = (float) bmm150.getCompassDegree(); //already a float 
  /* END OF BMM150 DATA */ //Serial.printf("BMM ends: %d\n", millis());

  /* TMP75 DATA */ //Serial.printf("TMP starts: %d, ", millis());
  sensor_data[8].flt = (float) TMP75_sensor.readTemperatureC();

  /* END OF TMP75 DATA */ //Serial.printf("TMP ends: %d\n", millis());

  /* MPU6050 DATA */ //Serial.printf("MPU starts: %d, ", millis());
  /*sensors_event_t a, g, tempMPU;
  MPU6050_sensor.getEvent(&a, &g, &tempMPU);

  payload_data->MPU6050_data->acceleration_x = (float) a.acceleration.x;
  payload_data->MPU6050_data->acceleration_y = (float) a.acceleration.y;
  payload_data->MPU6050_data->acceleration_z = (float) a.acceleration.z;
  payload_data->MPU6050_data->gyro_x = (float) g.gyro.x;
  payload_data->MPU6050_data->gyro_y = (float) g.gyro.y;
  payload_data->MPU6050_data->gyro_z = (float) g.gyro.z;
  payload_data->MPU6050_data->temperature = (float) tempMPU.temperature;
  /* END OF MPU6050 DATA */ //Serial.printf("MPU ends: %d\n", millis());

  /* FS3000 DATA */ //Serial.printf("FS3000 starts: %d, ", millis());
  sensor_data[9].flt = 1.0; //already a float
  /* END OF FS3000 DATA */ //Serial.printf("FS3000 ends: %d\n", millis());

  /* DIFFERENTIAL PRESSURE SENSOR DATA */ //Serial.printf("LWLP starts: %d, ", millis());
  //DFRobot_LWLP::sLwlp_t data;
  //data = LWLP5000_sensor.getData();

  //payload_data->LWLP5000_data.temperature = (float) data.temperature;
  //payload_data->LWLP5000_data.differential_pressure = (float) data.presure; //typo was in the example code, works
  /* END OF DIFFERENTIAL PRESSURE SENSOR DATA */ //Serial.printf("LWLP ends: %d\n", millis());

  /* GPS DATA */ //Serial.printf("GPS starts: %d, ", millis());
  /*
  payload_data->ZOE_M8Q_data.latitude = (float) ZOE_M8Q_sensor.getLatitude() / 10000000.0;
  payload_data->ZOE_M8Q_data.longitude = (float) ZOE_M8Q_sensor.getLongitude() / 10000000.0;
  payload_data->ZOE_M8Q_data.altitude = (float) ZOE_M8Q_sensor.getAltitude() / 10000000.0;
  payload_data->ZOE_M8Q_data.satellites_in_view = (float) ZOE_M8Q_sensor.getSIV();
  /* END OF GPS DATA */ //Serial.printf("GPS ends: %d\n", millis());


  /* MS5611 DATA */
  MS5611_sensor.read();
  sensor_data[10].flt = (float) MS5611_sensor.getTemperature();
  sensor_data[11].flt = (float) MS5611_sensor.getPressure();
  /*END OF MS5611 DATA */


  /*for (size_t i = 0; i < sizeof(PayloadData); i++) {
    Serial.println(byte_data[i]);
    Serial.printf("byte: %d\n", i);
  }*/
  /*Wire.beginTransmission(I2C_ESP32_ADDR);
  for (size_t i = 0; i < 12; i++) {
    for (int j = 0; j < 4; j++) {
          Wire.write(sensor_data[i].c[j]);
    }
  }
  //Serial.println("");
  Wire.endTransmission(true); */
  /* END OF SENDING DATA TO SLAVE ESP32 */ //Serial.printf("Data logging ends: %d\n", millis());

  Serial.flush();                           // waits until the transmission is finished
  uint8_t len;                              // length of packet sent
  LT.startWriteSXBuffer(0);                 //start the write packet to buffer process
  LT.writeBuffer(start_buff, sizeof(start_buff));

  for (size_t i = 0; i < sizeof(PayloadData) / 4; i++) {
    LT.writeFloat(sensor_data[i].flt);
    Serial.println(i);
  }

  len = LT.endWriteSXBuffer();
  if (TXPacketL = LT.transmitSXBuffer(0, len, 10000, TXpower, 0)) {  // The transmitting magic, a little underwhelming. will return packet length sent if OK, otherwise 0 if transmit error
    Serial.println(len);
  }
  else {
    Serial.println(F("Transmission failed"));
  }
}
//Function to flash LED 
//Pre: int millis for blink
//Post: None
//Blinks the LED on and off for the wait time
void flash(int wait){
  digitalWrite(LED_PIN, HIGH);
  delay(wait);
  digitalWrite(LED_PIN, LOW);
  delay(wait);
}
