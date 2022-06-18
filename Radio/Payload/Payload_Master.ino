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
#include "Adafruit_BME680.h" //libraries needed for BME (Adafruit_BME680)
#include <Adafruit_ISM330DHCX.h> //libraries needed for IMU (ISM330DHCX)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //library needed for GPS(GNSS ZOE-Click)
#include <SX128XLT.h>  //libraries needed for radio (DLP-RFS1280)
#include "Settings_ESP.h" //libraries needed for frequencies, LoRa settings etc for ESP32
//#include "DFRobot_BMM150.h" //libraries needed for BMM150 Breakout (DFRobot SEN0419)
#include <Temperature_LM75_Derived.h> //libraries needed for TMP75
//#include <Adafruit_MPU6050.h> //libraries needed for the 2nd IMU (MPU6050)
#include <SparkFun_FS3000_Arduino_Library.h> //libraries needed for the velocity sensor (FS3000)
#include <DFRobot_LWLP.h> //libraries needed for the differential pressure sensor (DF Robot SEN0343)
#include <MS5611.h> //libraries needed for the MS5611 barometer
#include <Adafruit_SCD30.h> //libraries needed for the SCD30


//Slave ESP32 Set up\\-------------------------------------------------------------------------------
#define I2C_ESP32_ADDR 0x11 //0x11 is the defined slave address for the ESP32 slave, can be changed
uint8_t dump_to_SD_card = 0; //flag to send the command to dump the flash data to the SD card
uint8_t data_log = 0; //flag to start logging data
uint8_t erase_flash_chip = 0; //flag to erase the flash chip
union float_and_char { //stores a float and char array in the same memory location, used to send 32-bit float values as 4 8-bit char values over I2C
  float flt; // 4 bytes
  char c[4]; // 1 byte * 4
};



/* NEW STUFF 6-9 */

#define PRELAUNCH_STATE       (uint8_t) 0
#define ARM_STATE             (uint8_t) 1
#define LAUNCH_STATE          (uint8_t) 2
#define APOGEE_STATE          (uint8_t) 3
#define DESCENT_STATE          (uint8_t) 4
#define GROUND_STATE          (uint8_t) 5

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

PayloadData* payload_data = (PayloadData*) malloc(sizeof(PayloadData));

/* END OF NEW STUFF 6-9 */


//SPI Set up\\---------------------------------------------------------------------------------------
SPIClass SPI2(HSPI); //setting up HSPI to work on ESP32 in conjuction with VSPI

//Pin Set up\\---------------------------------------------------------------------------------------------------
//most Pins are already defined in the library
uint8_t cs_pin = 15; //GPIO pin 15 has been selected for chip select the GPS
const uint8_t LED_PIN = 33; //GPIO pin 33 connected to LED 
const uint8_t MOSFET_PIN = 32; //GPIO pin 4 connected to the fan's MOSFET
//const uint8_t erase_flash_chip_button = 32; //GPIO pin 32 connected to erase flash chip button

//Defining Library functions to be called\\---------------------------------------------------------------------------------------------------
SFE_UBLOX_GNSS ZOE_M8Q_sensor; //Create the library class instance called myGNSS
SX128XLT LT; //Create the library class instance called LT for radio
Adafruit_ISM330DHCX ISM330DHCX_sensor; //Create the library class instance called ism330dhcx
Adafruit_BME680 BME680_sensor; //Create the library class instance called bme
//DFRobot_BMM150_I2C BMM150_sensor(&Wire, I2C_ADDRESS_4); //Create the library class instance bmm150 for the magnometer
//Adafruit_MPU6050 MPU6050_sensor; //Create the library class instance mpu for the MPU6050 IMU
FS3000 FS3000_sensor; //Create the library class instance fs for the FS3000 velocity sensor
DFRobot_LWLP LWLP5000_sensor; //Create the library class instance lwlp for the differential pressure sensor
Generic_LM75_12Bit TMP75_sensor; //Creat the library class instance TMP75_sensor for the temperature sensor
MS5611 MS5611_sensor(0x77);
Adafruit_SCD30  SCD30_sensor;


//Needed for radio to work\\----------------------------------------------------------------------------------------------------------------
uint8_t TXPacketL; //successful transmission
uint8_t start_buff[] = "a"; //low memory transmit mode makes the smallest buffer*/

//BME set up\\------------------------------------------------------------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25)

//Logging periods\\-------------------------------------------------------------------------------------------------------------------------
#define LOG_HALF_HZ                 (uint32_t) 2000 //ms
#define LOG_FIVE_HZ                 (uint32_t) 200 //ms

//Timers\\-----------------------------------------------------------------------------------------------------------------------------------
uint32_t time_BME_data_is_ready = 0;
uint32_t last_time_LWLP_was_polled = 0;
uint32_t last_time_data_was_logged = 0;
uint32_t log_period = LOG_HALF_HZ;
uint32_t time_to_enter_arm_state;

//ISM330DHCX Variables for continuous updating\\---------------------------------------------------------------------------------------------
sensors_event_t acceleration_global;
sensors_event_t gyro_global;
sensors_event_t temperature_global;

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
    digitalWrite(MOSFET_PIN, LOW);
    //pinMode(data_log_button, INPUT);
    //pinMode(erase_flash_chip_button, INPUT);

    //connecting to IMU
    while (!ISM330DHCX_sensor.begin_I2C())  { 
        Serial.println("Failed to communicate with IMU.");
        flash(50);
    }
    flash(500);

    //connecting to BME 
    while (!BME680_sensor.begin((uint8_t)0x76))  {
        Serial.println("Failed to communicate with BME.");
        flash(50);
    }
    //BME settings (think so)
    BME680_sensor.setTemperatureOversampling(BME680_OS_8X);
    BME680_sensor.setHumidityOversampling(BME680_OS_2X);
    BME680_sensor.setPressureOversampling(BME680_OS_4X);
    BME680_sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
    BME680_sensor.setGasHeater(320, 150); // 320*C for 150 ms
    time_BME_data_is_ready = BME680_sensor.beginReading(); // Start the reading so it's ready when data logging begins
    flash(500);
    
    
    //connecting to magnetometer 
    /*
    while(BMM150_sensor.begin()) { //returns true if it does not initialize
      Serial.println("Failed to communicate with BMM150 (magnometer).");
      flash(50);
    }
    flash(500); */
    
    //BMM150 settings
    /*
    BMM150_sensor.setOperationMode(BMM150_POWERMODE_NORMAL);
    BMM150_sensor.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY); //can change from high accuracy to enhanced or regular, probably would decrease power consumption
    BMM150_sensor.setMeasurementXYZ();
    */
    

    //connecting to MPU6050
    /*
    while (!MPU6050_sensor.begin()) {
      Serial.println("Failed to communicate with MPU6050.");
      flash(50);
    }
    flash(500);
    */
    //MPU6050 settings
    /*
    MPU6050_sensor.setAccelerometerRange(MPU6050_RANGE_8_G);
    MPU6050_sensor.setGyroRange(MPU6050_RANGE_500_DEG);
    MPU6050_sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
    */
    //connecting to FS3000
    
    while (!FS3000_sensor.begin()) {
      Serial.println("Failed to communicate with FS3000");
      flash(50);
    }
    flash(500); 

    //connecting to differential pressure sensor
    while (LWLP5000_sensor.begin() != 0) {
      Serial.println("Failed to communicate with differential pressure sensor/");
      flash(50);
    }
    flash(500);
    // starting the first reading
    LWLP5000_sensor.configChip();

    //connecting to GPS
    while (ZOE_M8Q_sensor.begin(SPI2, cs_pin, 5500000) == false)  {
        Serial.println("Failed to communicate with GPS.");
        flash(50);
    }
    ZOE_M8Q_sensor.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
    ZOE_M8Q_sensor.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    ZOE_M8Q_sensor.setAutoPVT(true); //Automatically updates PVT
    flash(500);

    //connecting to MS5611
    while (!MS5611_sensor.begin()) {
      Serial.println("MS5611 failed to communicate.");
      flash(50);
    }
    flash(500);

    MS5611_sensor.setOversampling(OSR_ULTRA_LOW);


  // Connecting to SCD30
    while (!SCD30_sensor.begin()) {
      Serial.println("Failed to find SCD30 chip");
      flash(50);
    }
    Serial.println("SCD30 Found!");
    flash(500);

    //conneting to radio
   
   while (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))  {
        Serial.println("Failed to communicate with Radio.");
        flash(50);
    }
    
    LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate); //sets up necessary settings for the radio
    flash(500); 
    digitalWrite(LED_PIN, HIGH); //Turns on LED if all connections are work ing!
    Serial.println("All Sensors Initialized ");
    
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(MOSFET_PIN, HIGH);
    delay(20000);
    //eraseFlashChip();
    //delay(15000);
    digitalWrite(LED_PIN, LOW);

//Setting state timers\\-----------------------------------------------------------------------------------------------------------
    time_to_enter_arm_state = millis() + 1200000;
    log_period = LOG_FIVE_HZ;
}

/* TESINNG */
uint32_t time_to_log = 0;
uint8_t current_state = PRELAUNCH_STATE;


void loop() {
  
  if (last_time_data_was_logged <= millis() - log_period) { // Currently sends data through radio 5 times/second.
    last_time_data_was_logged = millis();
    dataLog();
    time_to_log++;
    Serial.printf("time_to_log: %d\n", time_to_log);
  }
  if (time_to_log > 2000) {
    dumpToSDCard();
    while(1);
  }

  if (digitalRead(MOSFET_PIN)) {
    if (time_to_log > 200) {
      digitalWrite(MOSFET_PIN, LOW);
    }
  }


//STATE MACHINE\\------------------------------------------------------------------------------------------------------------------
  /*
  switch (current_state) {
    case PRELAUNCH_STATE:
        
        //Log data
        if (last_time_data_was_logged <= millis() - LOG_HALF_HZ) { // Currently sends data through radio 0.5 times/second.
          last_time_data_was_logged = millis();
          dataLog();
        }

        //Read the acceleration
        ISM330DHCX_sensor.getEvent(&acceleration_global, &gyro_global, &temperature_global);
        payload_data->ISM330DHCX_data.acceleration_z = acceleration_global.acceleration.z; //we only care about the z-axis

      //once 20 minutes have passed (time_to_enter_arm_state), check for 0.8G's in the selected axis
      if (millis() >= time_to_enter_arm_state) { 
        if (compareAcceleration(payload_data->ISM330DHCX_data.acceleration_z, -7.85)) { // if the z-axis experiences 0.8G's or greater (negative or positive), set current_state to ARM_STATE
          current_state = ARM_STATE;
        }
      }
      
      break;
      
    case ARM_STATE:
      break;
      
    case LAUNCH_STATE:
      break;
      
    case APOGEE_STATE:
      break;
      
    case DESCENT_STATE:
      break;
      
    case GROUND_STATE:
      break;
      
  }
  */
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
  payload_data->time_stamp = (float) millis() / 1000.0; //convert ms to s
  /* END OF TIME DATA */
  
  /* ISM330DHCX DATA */ //Serial.printf("ISM330 starts: %d, ", millis());
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t tempISM330;
  ISM330DHCX_sensor.getEvent(&accel, &gyro, &tempISM330);
  
  payload_data->ISM330DHCX_data.temperature = (float) tempISM330.temperature;
  payload_data->ISM330DHCX_data.acceleration_x = (float) accel.acceleration.x;
  payload_data->ISM330DHCX_data.acceleration_y = (float) accel.acceleration.y;
  payload_data->ISM330DHCX_data.acceleration_z = (float) accel.acceleration.z;
  payload_data->ISM330DHCX_data.gyro_x = (float) gyro.gyro.x;
  payload_data->ISM330DHCX_data.gyro_y = (float) gyro.gyro.y;
  payload_data->ISM330DHCX_data.gyro_z = (float) gyro.gyro.z;
  /* END OF ISM330DHCX DATA */ //Serial.printf("ISM330 ends: %d\n", millis());

  /* BME680 DATA */ //Serial.printf("BME starts: %d, ", millis());
  if (millis() > time_BME_data_is_ready) { //checks if data is ready, otherwise does not change BME's sensor_data values
    BME680_sensor.endReading();
    
    payload_data->BME680_data.temperature = (float) BME680_sensor.temperature;
    payload_data->BME680_data.pressure = (float) BME680_sensor.pressure / 100.0;
    payload_data->BME680_data.humidity = (float) BME680_sensor.humidity;
    payload_data->BME680_data.gas_resistance = (float) BME680_sensor.gas_resistance;

    time_BME_data_is_ready = BME680_sensor.beginReading(); //begins the next reading and returns the next time the BME data will be ready
  }
  /* END OF BME680 DATA */ //Serial.printf("BME ends: %d\n", millis());

  /* BMM150 DATA */ //Serial.printf("BMM starts: %d, ", millis());
  /*
  sBmm150MagData_t magData = BMM150_sensor.getGeomagneticData();

  payload_data->BMM150_data.geomagnetic_x = (float) magData.x;
  payload_data->BMM150_data.geomagnetic_y = (float) magData.y;
  payload_data->BMM150_data.geomagnetic_z = (float) magData.z; 
  */
  /* END OF BMM150 DATA */ //Serial.printf("BMM ends: %d\n", millis());

  /* TMP75 DATA */ //Serial.printf("TMP starts: %d, ", millis());
  payload_data->TMP75_data.temperature = (float) TMP75_sensor.readTemperatureC();

  /* END OF TMP75 DATA */ //Serial.printf("TMP ends: %d\n", millis());

  /* MPU6050 DATA */ //Serial.printf("MPU starts: %d, ", millis());
  /*
  sensors_event_t a, g, tempMPU;
  MPU6050_sensor.getEvent(&a, &g, &tempMPU);

  payload_data->MPU6050_data.acceleration_x = (float) a.acceleration.x;
  payload_data->MPU6050_data.acceleration_y = (float) a.acceleration.y;
  payload_data->MPU6050_data.acceleration_z = (float) a.acceleration.z;
  payload_data->MPU6050_data.gyro_x = (float) g.gyro.x;
  payload_data->MPU6050_data.gyro_y = (float) g.gyro.y;
  payload_data->MPU6050_data.gyro_z = (float) g.gyro.z;
  payload_data->MPU6050_data.temperature = (float) tempMPU.temperature;

  */
  /* END OF MPU6050 DATA */ //Serial.printf("MPU ends: %d\n", millis());

  /* FS3000 DATA */ //Serial.printf("FS3000 starts: %d, ", millis());
  payload_data->FS3000_data.air_velocity = (float) FS3000_sensor.readMetersPerSecond();
  /* END OF FS3000 DATA */ //Serial.printf("FS3000 ends: %d\n", millis());

  /* DIFFERENTIAL PRESSURE SENSOR DATA */ //Serial.printf("LWLP starts: %d, ", millis());
  if (last_time_LWLP_was_polled <= millis() - 30) { // can only be read a maximum of every 30ms or else it freezes
      DFRobot_LWLP::sLwlp_t data;
      data = LWLP5000_sensor.getData();
      LWLP5000_sensor.configChip(); //starting the next read
      last_time_LWLP_was_polled = millis(); //to check when it's ready to be read next

      payload_data->LWLP5000_data.temperature = (float) data.temperature;
      payload_data->LWLP5000_data.differential_pressure = (float) data.presure; //pressure typo was in the example code, works
  }

  /* END OF DIFFERENTIAL PRESSURE SENSOR DATA */ //Serial.printf("LWLP ends: %d\n", millis());

  /* GPS DATA */ //Serial.printf("GPS starts: %d, ", millis());
  
  payload_data->ZOE_M8Q_data.latitude = (float) ZOE_M8Q_sensor.getLatitude() / 10000000.0;
  payload_data->ZOE_M8Q_data.longitude = (float) ZOE_M8Q_sensor.getLongitude() / 10000000.0;
  payload_data->ZOE_M8Q_data.altitude = (float) ZOE_M8Q_sensor.getAltitude() / 1000.0; //in meters
  payload_data->ZOE_M8Q_data.satellites_in_view = (float) ZOE_M8Q_sensor.getSIV();
  /* END OF GPS DATA */ //Serial.printf("GPS ends: %d\n", millis());

  /* SCD30 DATA */
  if (SCD30_sensor.dataReady()){
    SCD30_sensor.read();

    payload_data->SCD30_data.temperature = (float) SCD30_sensor.temperature;
    payload_data->SCD30_data.humidity = (float) SCD30_sensor.relative_humidity;
    payload_data->SCD30_data.CO2_concentration = (float) SCD30_sensor.CO2;
  }
  /* END OF SCD30 DATA */


  /* MS5611 DATA */
  MS5611_sensor.read();
  payload_data->MS5611_data.temperature = (float) MS5611_sensor.getTemperature();
  payload_data->MS5611_data.pressure = (float) MS5611_sensor.getPressure();
  /*END OF MS5611 DATA */


  /*for (size_t i = 0; i < sizeof(PayloadData); i++) {
    Serial.println(byte_data[i]);
    Serial.printf("byte: %d\n", i);
  }*/

  
  /* SENDING DATA TO SLAVE ESP32 */
  Wire.beginTransmission(I2C_ESP32_ADDR);
  uint8_t* payload_data_8bit = (uint8_t*) payload_data;
  for (size_t i = 0; i < sizeof(PayloadData); i++) {
    Wire.write(payload_data_8bit[i]);
  }
  //Serial.println("");
  Wire.endTransmission(true);
  /* END OF SENDING DATA TO SLAVE ESP32 */ //Serial.printf("Data logging ends: %d\n", millis());

  Serial.flush();                           // waits until the transmission is finished
  uint8_t len;                              // length of packet sent
  LT.startWriteSXBuffer(0);                 //start the write packet to buffer process
  LT.writeBuffer(start_buff, sizeof(start_buff));

  float* payload_data_float = (float*) payload_data;
  for (size_t i = 0; i < sizeof(PayloadData) / 4; i++) {
    LT.writeFloat(payload_data_float[i]);
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

uint8_t compareAcceleration (float a1, float a2) {
  if (a2 < 0) { // if a2 is negative, assume we are comparing two negative values and change them to positive
    a2 *= -1;
    a1 *= -1;
  }
  return (a1 > a2) ? 1 : 0; //a1 > a2 returns 1, a1 < a2 rea\turns 0.
}
