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
#include <Adafruit_LIS2MDL.h> //libraries needed for Magnetometer (Adafruit ISM330DHCX)
#include "Adafruit_BME680.h" //libraries needed for BME (Adafruit_BME680)
#include <Adafruit_ISM330DHCX.h> //libraries needed for IMU (ISM330DHCX)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //library needed for GPS(GNSS ZOE-Click)
#include <SX128XLT.h>  //libraries needed for radio (DLP-RFS1280)
#include "Settings_ESP.h" //libraries needed for frequencies, LoRa settings etc for ESP32

SPIClass SPI2(HSPI); //setting up HSPI to work on ESP32 in conjuction with VSPI

//Pin Set up\\---------------------------------------------------------------------------------------------------
//most Pins are already defined in the library
uint8_t csPin = 15; //GPIO pin 15 has been selected for chip select the GPS
const int led_pin = 33; //GPIO pin 33 connected to LED 
const int button_pin = 32; //GPIO pin 32 connected to button

//Defining Library functions to be called\\---------------------------------------------------------------------------------------------------
SFE_UBLOX_GNSS myGNSS; //Create the library class instance called myGNSS
SX128XLT LT; //Create the library class instance called LT for radio
Adafruit_ISM330DHCX ism330dhcx; //Create the library class instance called ism330dhcx
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
Adafruit_BME680 bme; //Create the library class instance called bme 

//Needed for radio to work\\---------------------------------------------------------------------------------------------------
uint8_t TXPacketL; //successful transmission
uint8_t start_buff[] = "a"; //low memory transmit mode makes the smallest buffer

//for BME 
#define SEALEVELPRESSURE_HPA (1013.25)

void setup() 
{
    Serial.begin(115200);
    Wire.begin(); //For i2c 
    SPI.begin(); //For VSPI (Used for Radio)
    SPI2.begin(); //FOR HSPI (Used for GPS)

    //Connecting\\-------------------------------------------------------------------------------------------------------------
    pinMode(led_pin, OUTPUT);
    pinMode(button_pin, INPUT);

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
    flash(500);

    //connecting to magnetometer 
    while (!lis2mdl.begin())  {
        Serial.println("Failed to communicate with Magnetometer.");
        flash(50);
    }
    flash(500);

    //connecting to GPS
    while (myGNSS.begin(SPI2, csPin, 5500000) == false)  {
        Serial.println("Failed to communicate with GPS.");
        flash(50);
    }
    myGNSS.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
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
        control();
        delay(10);
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

//main control
//has functions required to run all sensors 
void control(){
    baro_data();
    imu_data();
    mag_data();
    gps_data();
    radio_data();
    esp_now();
}

//Function to print Barometer Data to the serial port 
void baro_data(){
    if (! bme.performReading()) { //needed for BME not sure why?
    return;
  }
  Serial.print(float (bme.temperature)); //BME Temp
  Serial.print(",");
  Serial.print(float (bme.pressure / 100.0)); //BMe Pressure 
  Serial.print(",");
  Serial.print(float (bme.humidity)); //BME Humidity
  Serial.print(",");
  Serial.print(float (bme.gas_resistance / 1000.0)); //BME Gas resistance 
  Serial.print(",");
}

//Function to print IMU Data to the serial port 
void imu_data(){
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  Serial.print(float (temp.temperature)); //IMU Temp
  Serial.print(",");
  Serial.print(float (accel.acceleration.x)); //IMU accel x
  Serial.print(",");
  Serial.print(float (accel.acceleration.y)); //IMU accel y
  Serial.print(",");
  Serial.print(float (accel.acceleration.z)); //IMU accel z
  Serial.print(",");
  Serial.print(float (gyro.gyro.x)); //IMU Gyro x
  Serial.print(",");
  Serial.print(float (gyro.gyro.y)); //IMU Gyro y
  Serial.print(",");
  Serial.print(float (gyro.gyro.z)); //IMU Gyro z
  Serial.print(",");
} 

//Function to print Magnetometer Data to the serial port 
void mag_data(){
  /* Get a normalized sensor event */
  sensors_event_t event;
  lis2mdl.getEvent(&event);
  Serial.print(float (event.magnetic.x)); //Magnetic x
  Serial.print(",");
  Serial.print(float (event.magnetic.y)); //Magnetic y
  Serial.print(",");
  Serial.print(float (event.magnetic.z)); //Magnetic z
}

//Function to print GPS Data to the serial port 
void gps_data(){
  Serial.print(float (myGNSS.getLatitude())); //GPS Data Latitude
  Serial.print(",");
  Serial.print(float (myGNSS.getLongitude())); //GPS Data Longitude
  Serial.print(",");
  Serial.print(uint8_t (myGNSS.getSIV())); //GPS SIV
  Serial.print(",");
}

void radio_data(){
    if (! bme.performReading()) { //needed for BME not sure why?
    return;
    }
    //converting all BME Data into floats before sending
    float temperature = bme.temperature;
    float pressure = bme.pressure / 100.0;
    float humidity = bme.humidity;
    float gas = bme.gas_resistance / 1000.0;
    float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);

    //converting all magnetometer data into floats before sending 
    sensors_event_t event;
    lis2mdl.getEvent(&event);
    float x_magnetic = event.magnetic.x;
    float y_magnetic = event.magnetic.y;
    float z_magnetic = event.magnetic.z;

    //converting all accelerometer data into floats before sending 
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

    //converting all GPS data into floats before sending
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

void esp_now(){

}