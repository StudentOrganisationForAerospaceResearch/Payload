Obejective: Transmit Sensor and GPS data between 2 Lora SX1280 radios (DLP-RFS1280)

Sensors:           
  Magsensor (Adafruit_LIS2MDL)           
  IMU (Adafruit ISM330DHCX)          
  BME (Adafruit_BME680)                
  GPS (GNSS ZOE-Click)

Libraries:               
  Radio: //github.com/StuartsProjects/SX12XX-LoRa             
  Magsensor: search for Adafruit_LIS2MDL in arduino library manager,            
  IMU: search for ISM330DHCX in arduino library manager,              
  BME: search for Adafruit_BME680 in arduino library manager,            
  GPS: //librarymanager/All#SparkFun_u-blox_GNSS,         

Changing Transmission settings: 
1. using the settings.h file, radio transmission elements such as bandwidth, coding rate and spreading factor can be manipulated to change 
    signal range and packet transmission speed. Accpeted values for these parameters can be found here: github.com/StuartsProjects/SX12XX-LoRa/blob/master/src/SX128XLT_Definitions.h
    For more information on the effects of each of these settings: medium.com/home-wireless/testing-lora-radios-with-the-limesdr-mini-part-2-37fa481217ff                                
2. Changing the delay in the settings file does not work. Instead, change the delay at the bottom of the main code that breaks up the transmissions. This value cannot be set to least than 10ms. Ensure that the settings in step 1 allow for the transmission speed you coded for. 
  
 
  Adding additional sensors:           
    1. Install libraries and set up device         
    2. Define variables that need to be sent             
    3. Assign values to these variables               
    4. Send values by adding them to the buffer (Using LT.writeFloat or LT.writeUint8)
    
Sumair Shergill - April 9, 2022
  
