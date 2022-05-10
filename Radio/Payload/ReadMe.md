### Obejective
Transmit Sensor and GPS data between 2 Lora SX1280 radios (DLP-RFS1280) and log the data to a flash chip

### Sensors           
- Magsensor (BMM150)           
- IMU (Adafruit ISM330DHCX and MPU6050)         
- Baro/Humidity/Temp/Gas (Adafruit_BME680)                
- GPS (GNSS ZOE-Click)
- Temperature sensor (TMP117)
- Velocity sensor (FS3000)
- Differential Pressure sensor (DF Robot SEN0343)

### Libraries:               
- Radio: //github.com/StuartsProjects/SX12XX-LoRa             
- Magsensor: search for Adafruit_LIS2MDL in arduino library manager            
- Adafruit ISM330DHCX: search for ISM330DHCX in arduino library manager              
- Baro/Humidity/Temp/Gas: search for Adafruit_BME680 in arduino library manager            
- GPS: //librarymanager/All#SparkFun_u-blox_GNSS
- BMM150: DFRobot SEN0419
- TMP117: Adafruit TMP117
- MPU6050: Search MPU6050 in library manager
- FS3000: SparkFun_FS3000_Arduino_Library.h
- Differential Pressure sensor: DFRobot_LWLP.h

### Changing Transmission settings 
1. using the settings.h file, radio transmission elements such as bandwidth, coding rate and spreading factor can be manipulated to change 
    signal range and packet transmission speed. Accpeted values for these parameters can be found here: github.com/StuartsProjects/SX12XX-LoRa/blob/master/src/SX128XLT_Definitions.h
    For more information on the effects of each of these settings: medium.com/home-wireless/testing-lora-radios-with-the-limesdr-mini-part-2-37fa481217ff                                
2. Changing the delay in the settings file does not work. Instead, change the delay at the bottom of the main code that breaks up the transmissions. This value cannot be set to least than 10ms. Ensure that the settings in step 1 allow for the transmission speed you coded for. 
  
 
 ### Adding additional sensors          
1. Install libraries and set up device       
2. Add the sensor measurements to the sensor_data array 

    
Sumair Shergill - April 9, 2022
  
