# Payload Model - Tsukuyomi




## Radio and Ground Systems 

### Ground Systems 

LoRaWAN is used to relay information from the rocket down to ground systems radio. The DLP-RFS1280 uses a modified Semtech SX1280 for long range communication at 2.4 GHz. The team chose to use a spreading factor of 7, a coding rate of 4:5, and a bandwidth of 812500hz. Packets are sent at 100 ms intervals. These settings are designed to maximize data rate while maintaining radio range and are replicated on an identical receiving radio module. Both radios have a small directional antenna attached (ANT-2.4-PML-UFL).

The ground systems radio is connected to an Arduino Nano which connects, via serial port, to a computer. Incoming data is graphed using an open-source Javascript software developed by Farrel Farahbod. The program enables the graphing of real time data from the serial port of the Arduino Nano. Additionally, the data can be exported as a CSV for further analysis after flight. 

The following data is received by the radio (* denotes data from multiple sensors):
*Time, temperature*, acceleration-x*, acceleration-y*, acceleration-z*, rotation-x*, rotation-y*, rotation-z*, pressure*, humidity*, gas resistance, geomagnetic-x, geomagnetic-y, geomagnetic-z, air velocity, differential pressure, latitude, longitude, altitude*

Additional wireless communication is done through a WiFi mesh with the avionics bay, where payload transmits data using the ESP-NOW protocol. This transmission is done through short range WiFi where data can be sent to avionics ground system for redundancy of logging experimental data in the case of a faulty recovery or LoRaWAN issue.

#### Instructions on use of the Ground Systems

Before use of the ground systems software, ensure you have the latest version of [Java](https://www.java.com/en/) installed.
Ensure that the Software location is within your documents or downloads folder as a cache file is created which is required to export a final CSV Log.

Plug in the Ground Systems Board to an available COM port and `connect` to the microcontroller being used from within the software.
Load the Ground systems layout txt file, this file can be edited to change the colours as well as the information.
Once flight is completed click on the `export CSV log button` and select a location to save the file to.




