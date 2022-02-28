/* SOAR Payload Model Ra - Teensy 3.6 code

  Objective:
  -Record the following data 
    * 4 thermocouples on the nose cone
    * 4 photosensors radial around nose cone 
    * IMU -> 9DOF Acceleration, Magnetic field, Angular acceleration
    * Barometer -> Pressure
  -Calculate 
    * Roll rate -> Photosensors & IMU
    * Position -> Altitude from Baro & Altitude / Position from IMU
    * Temperature profile of nose cone from thermocouples

  -Recording Data 
    * Flash on Teensy 3.6 
    * Try to save to micro sd card -> Vibrations can interupt
    * Once vibrations are gone offload data fully to file on micro sd card 
    * Still try to log data to sd card throughout flight until an interupt occurs 
  

   Sensors
     - IMU '1*' (sen 13284 - 9 DoF)  https://www.sparkfun.com/products/13284
     - Pressure Sensor '1*'(MS5611) hhttps://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036 
     - Thermister '4*' 'profile' https://www.digikey.ca/en/products/detail/amphenol-advanced-sensors/NHQM103B375T10/374822
     - Photoresitor '4*' 'radial' https://www.digikey.ca/en/products/detail/adafruit-industries-llc/161/7244927
    
  
  How to use Teensy in Arduino --> https://www.pjrc.com/teensy/tutorial.html
  1. Setting up ide and loader:
     Go to prjc.com in the teesny section check what versions of arduino work
     Install aruino ide from arduino.cc -I use 1.8.11 I am sure the most recent will work
     On prjc.com install teensyduino
     At this point it is good to check if it is working follow youtube video
     Use teensy 3.6 and the cable is a micro usb
     https://www.youtube.com/watch?v=QdAuWOfOTE4&t=19s
  2. Setting Up libraries:
     There four main libraries two come with arduino, you can search for them in the
     sketch > include librarie > manage libraries
     Search 'LSM9DS1' and use the sparkfun one
     https://github.com/jarzebski/Arduino-MS5611 library
  3. Testing:
     Plug in teensy board with micro controller usb, do not press the button
     Click the verify button in the ids (the checkmark)
     This may take some time, it will then launch the teensy program loader
     It will instruct you to hit the button if you want to load your code onto it, if not it will just run the code on it
     To test the blink tutorial code can be used by going to file > examples > 01.basics > blink (optional)
     This will load the code if you modify the code just click the upload button (the right arrow) in the arduino ide
     To see the console, where the serial connection is to, tools > serial monitor
     https://forum.pjrc.com/threads/639-Teensy-3-0-serial-monitor-not-working

  Metro Anderson
 */

//Libray set up\\ ------------------------------------------------------------------------------------------------------------------------
#include <SD.h>     //For the micro SD card
#include <Wire.h>   //For I2C
#include <SPI.h>    //From IMU sample code pretty sure not needed...
#include <SparkFunLSM9DS1.h>      //IMU
#include <MS5611.h>   //Barometer 


//Pin set up\\ ------------------------------------------------------------------------------------------------------------------------
const int LED = 13;   //For the Teensy 3.6
//Schematic based:
int thermistor_pins[] = {A6, A7, A8, A9};    //lowest is closer to middle on hub board 
int photo_resistor_pins[] = {A0, A1, A2, A3}; 


//Global Vars\\ ------------------------------------------------------------------------------------------------------------------------
#define THERMISTORSCOUNT 4
#define PHOTORESISTORCOUNT 4

#define previous 25 //Set amount of previous data to store... Not used oh well

#define VCC 3.3     //Supply voltage
#define R 10000     //R=10K

//Calibrate Steinhart-Hart Coefficients for Thermistors
//https://www.vishay.com/thermistors/ntc-rt-calculator/
float A = -12.89228328;
float B = 4245.14800000;
float C = -87493.00000000;
//float D = -9588114.00000000  
//Not sure what D is for cannot find an equation using it...

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 13.17 // Declination (degrees) for viking
//13.79 for NE calgary by my house
double referencePressure;


//Holds all current data \\ ------------------------------------------------------------------------------------------------------------------------
/*
   Data is an array of all reading values that will be updated
   0-2 Gyro: alt, pressure, temp
   3-14 IMU: gx, gy, gz, ax, ay, az, mx, my, mz, pitch, roll, heading
   15-18 TR: T1, T2, T3, T4
   19-22 PR: PR1, PR2, PR3, PR4
*/
float data[23];
String titles[23] = {"alt", "pressure", "temp", "gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz", "pitch", "roll", "heading", "T1", "T2", "T3", "T4", "PR1", "PR2", "PR3", "PR4"};
float previous_alt[previous];
float previous_acc[previous]; //Not sure what axis to focus on
int num_recorded = 0;
int state = 0;  //Just will determine how oftern the data is recorded not the rate at which data is aquired

long tick = 0; 
int rate = 100; //Smaller the rate more data recorded => inverse relationship as modulus is used
int flag = 0; //1 for once launched
long started = 0;
int rail_flag = 0; //1 for when upright on launch rail

boolean testing = false; //Prints to SD card, then will go back to terminal
//***In launch mode

//Instances that use I2C\\ ------------------------------------------------------------------------------------------------------------------------
MS5611 ms5611;
LSM9DS1 imu;


//Files on SD card\\ ------------------------------------------------------------------------------------------------------------------------
File myFile;  //for SD card
const int chipSelect = BUILTIN_SDCARD;
int recording_rate = 0; //not needed yet
char filename[24];  //Will be a new file that is n+1 from the last data recording
//******Important Note: SD card health gets affected negatively with how I open and close files all the time.
//Need to find a way to keep file open and if there is an interupt the data is not lost... This couls be done later with altitude as indicators


void setup() {
  Serial.begin(9600);  //115200 could be used if all compentents can handle it
  analogReadResolution(10); //10 bits giving a 0-1023 value limiting accuracy    
  Wire.begin();

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(2000);  //Gives time for 'serial to connect'
  digitalWrite(LED, LOW);

  //Connecting\\ ------------------------------------------------------------------------------------------------------------------------
  wash();
  
  /* Half second Intervals with wash before 
   * IMU -> flash none none
   * Baro -> flash flash none
   * SD -> flash flash flash
   */

  //IMU\\
  Serial.println("Initialize LSM9DS1 Sensor");
  /*
  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }
  */ 
  /*
  while (imu.begin() == false) {
    Serial.println("Failed to communicate with LSM9DS1.");
    delay(500);
  }
  */
  //IMU connected sequence
  Serial.println("LSM9DS1 connected!");
  wash();
  flash(500);
  delay(500);
  delay(500);
  

  //Baro\\
  Serial.println("Initialize MS5611 Sensor");
  while(!ms5611.begin())
  {
    Serial.println("Could not find a valid MS5611 sensor");
    delay(500);
  }

  //Baro connect sequence 
  Serial.println("MS5611 connected!");
  wash();
  flash(500);
  flash(500);
  delay(500);
  
  // Get reference pressure for relative altitude
  referencePressure = ms5611.readPressure();

  // Check settings
  checkSettings();

  digitalWrite(LED, HIGH);

 //SD card\\
 Serial.println("Initialize SD card");
 while (!SD.begin(chipSelect)) {
    Serial.println("initialization failed with SD card!");
    delay(500);
  }

  //SD connect sequence
  Serial.println("SD card connected!");
  wash();
  flash(500);
  flash(500);
  flash(500);

  //https://forum.sparkfun.com/viewtopic.php?t=38500
  //Testing/data'xxx'.txt for each run 
  int n = 0;
  snprintf(filename, sizeof(filename), "Testing/data%03d.txt", n); // includes a three-digit sequence number in the file name
  while(SD.exists(filename)) {
    n++;
    snprintf(filename, sizeof(filename), "Testing/data%03d.txt", n);
  }  

  Serial.println("Data will be saved in: "+ String(filename));
  
  myFile = SD.open(filename, FILE_WRITE);                                                                     
  if (myFile) {
    myFile.println("time,alt,pressure,temp,gx,gy,gz,ax,ay,az,mx,my,mz,pitch,roll,heading,T1,T2,T3,T4,PR1,PR2,PR3,PR4");
    myFile.close();
  }

  //led always on means everything is good for 5 seconds
  digitalWrite(LED, HIGH); //Turns on if all connections are working!
  delay(5000);
  digitalWrite(LED, LOW); 
  
  Serial.println("Begining!");
}

void loop() {
  tick += 1;
  if (tick % rate == 0) {
    SD_print(data);
    num_recorded += 1;
  }
  delay(10); //100 times per second is max recording rate
  if (tick > 100){
    control_data(data);
    launching(data[6]);
  }
}


/**
   Pre: int millis for blink
   Post: None
   Blinks the LED on the teensy on and off for wait time
*/
void flash(int wait) {
  digitalWrite(LED, HIGH);
  delay(wait);
  digitalWrite(LED, LOW);
  delay(wait);
}


/**
   Pre:
   Post: 3 quick flashes with a delay of 2 seconds
   wash for next clear combo
*/
void wash() {
  flash(50);
  flash(50);
  flash(50);
  delay(2000);
}

/**
   Pre:  
   Post: Baro / Nothing
   Baro is a mystery and uses this. Don't think it does much
 */
void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
}

/*
   Pre: Given data array by reference
   Post: Gets all values for the array (23)
   Fills out all predefinded value keys with values
*/
void control_data(float *data) {
  baro_data(data);
  imu_data(data);
  thermo_data(data);
  photo_data(data);
}

//Baro\\ ------------------------------------------------------------------------------------------------------------------------

/*
   Pre: data array
   Post: gives altitude, pressure, temperature
   Index: 0,1,2
*/
void baro_data(float *data) {
  // Read raw values
  uint32_t rawTemp = ms5611.readRawTemperature();
  uint32_t rawPressure = ms5611.readRawPressure();

  // Read true temperature & Pressure
  double realTemperature = ms5611.readTemperature();
  long realPressure = ms5611.readPressure();

  // Calculate altitude
  float absoluteAltitude = ms5611.getAltitude(realPressure);
  float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
  
  data[0] = relativeAltitude;
  data[1] = realPressure;
  data[2] = realTemperature;
}


//IMU\\ ------------------------------------------------------------------------------------------------------------------------

/*
   Pre: data array
   Post: Sets values in main data array
   Reads IMU
*/
void imu_data(float *data) {
  xyz_dps(data);
  xyz_acc(data);
  xyz_mag(data);
  axes(data);
  //latest();
}

/*
   Pre: data array
   Post: gives xyz dps or -1 if not available
   index: 3,4,5
   deg/s
*/
void xyz_dps(float *data) {
  if (imu.gyroAvailable()) {
    imu.readGyro();
    data[3] = imu.calcGyro(imu.gx);
    data[4] = imu.calcGyro(imu.gy);
    data[5] = imu.calcGyro(imu.gz);
  } else {
    data[3] = -1;
    data[4] = -1;
    data[5] = -1;
  }
}

/*
   Pre: data array
   Post:gives xyz acceleration or -1 if not available
   Index: 6,7,8
   g
*/
void xyz_acc(float *data) {
  if (imu.accelAvailable()) {
    imu.readAccel();
    data[6] = imu.calcAccel(imu.ax);
    data[7] = imu.calcAccel(imu.ay);
    data[8] = imu.calcAccel(imu.az);
  } else {
    data[6] = -1;
    data[7] = -1;
    data[8] = -1;
  }
}

/*
   Pre: data array
   Post: gives xyz magnitude for mag feilds or -1 is not available
   Index: 9,10,11
   gauss
*/
void xyz_mag(float *data) {
  if (imu.magAvailable()) {
    imu.readMag();
    data[9] = imu.calcMag(imu.mx);
    data[10] = imu.calcMag(imu.my);
    data[11] = imu.calcMag(imu.mz);
  } else {
    data[9] = -1;
    data[10] = -1;
    data[11] = -1;
  }
}

/*
   Pre: data array
   Post: gives pitch roll heading
   Index: 12,13,14 (pitch, roll, heading)
*/
void axes(float *data) {
  // Calculate pitch, roll, and heading.
  // Pitch/roll calculations take from this app note:
  // http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
  // Heading calculations taken from this app note:
  // http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf

  data[12] = atan2((-1 * imu.mx), sqrt(imu.ay * imu.ay + imu.az * imu.az)) * 180.0 / PI;
  data[13] = atan2(imu.ay, imu.az) * 180.0 / PI;

  float heading;  //Not sure what is happening here
  if (imu.my == 0)
    heading = (imu.mx < 0) ? PI : 0;
  else
    heading = atan2(imu.mx, imu.my);

  heading -= DECLINATION * PI / 180.0;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  data[14] = heading * PI / 180.0;
}

//heading -= DECLINATION * PI / 180.0;


//Thermo\\ ------------------------------------------------------------------------------------------------------------------------

/**
   Pre: data array
   Post: gives the temps down the profule of the nose cone
   Index: 15,16,17,18
   
*/
void thermo_data(float *data) {
  /*
  for (int i = 0; i < THERMISTORSCOUNT; i++) {
    arrVRT[i] = analogRead(thermistor_pins[i]);
    arrVRT[i] = (3.30 / 1023.00) * arrVRT[i];
    arrVR[i] = VCC - arrVRT[i];
    arrRT[i] = arrVRT[i] / (arrVR[i] / R);

    arrln[i] = log(arrRT[i] / RT0);
    arrTX[i] = (1 / ((arrln[i] / B) + (1 / T0)));
    arrTX[i] = arrTX[i] - 273.15;

    data[i + 15] = arrTX[i];
  }
  
  float t_temp = 0;
  float r_temp = 0;
  for (int i = 0; i < THERMISTORSCOUNT; i++){
    t_temp = analogRead(thermistor_pins[i]);
    t_temp *= (3.30 / 1023.00);
    r_temp = (t_temp / 3.30 * 10000) / (1.0 - t_temp/3.30);
    t_temp = 1 /(A + B * log(t_temp) + C * pow(log(r_temp), 3));
    data[i + 15] = t_temp;
  }
  */
  //Need to look up conversion for thermistors that are being used!
  //Having issues with conversions... Going to have just raw voltages and calibrate later in data analysis!
  for (int i =0; i < THERMISTORSCOUNT; i++) {
    data[i + 15] = analogRead(thermistor_pins[i]);
  }
}


//Photo\\ ------------------------------------------------------------------------------------------------------------------------

/**
   Pre: data array
   Post: gives the photo resistors voltages?resistance
   Index: 19,20,21,22

*/
void photo_data(float *data) {
  for (int i = 0; i < PHOTORESISTORCOUNT; i++) {
    data[i + 19] = analogRead(photo_resistor_pins[i]);
  }
}

//SD card\\ ------------------------------------------------------------------------------------------------------------------------

/*
   Pre:
   Post:

*/
void SD_print(float *data) {
  if (testing) {
    Serial.print(millis());
    Serial.println(" ms:");
    for (int i = 0; i < 23; i++) {
      if (i == 3 || i == 15 || i == 19) Serial.print("\n");
      Serial.print(titles[i] + ": ");
      Serial.print(String(data[i]) + "   ");
    }
    Serial.print("\n\n");
  } else {
    myFile = SD.open(filename, FILE_WRITE);
    myFile.print(millis());
    for (int j = 0; j < 23; j++) {
      if (j == 22) {
        myFile.print(", " + String(data[j]) + "\n");
      } else {
        myFile.print(", " + String(data[j]));
      }
    }
    myFile.close();
  }
}

//Launching\\ ------------------------------------------------------------------------------------------------------------------------
/*
  Pre: Requires acceleration, potentailly acceleration 
  Post: Sets off global flag to launch state
  Will take in acceleration in roll axis and once it becomes zero trigger launch 
  It will initially be 1 g due to gravtiy and once zero it will be expeirencing a launch
 */
void launching(float acc_roll_axis) {
  //Logging rate 
  //File 
  //Launch mode 

  //***The issue with how I am testing as I am not simulating breaking gravity in the tests, however the 1g in x axis should still show up in test files 

  Serial.println(acc_roll_axis);
  //Be careful of direction........
  if (acc_roll_axis >= 0.85 && rail_flag == 0) {
    rail_flag = 1;
    Serial.println("Upright");
    digitalWrite(LED, HIGH);
  }
  
  if (acc_roll_axis <= 0.5 && flag == 0 && rail_flag == 1) {
    Serial.println("Launched");
    flag = 1;
    rate = 1; //100 times per second 
    started = num_recorded;
    digitalWrite(LED, LOW);
    
    //file name --> Create new file for launching? nope 
  }

  if (started != 0 && num_recorded - started >= 30000){
    rate = 100; //after 30000 lines of rapid recording or should be 300 secs go back to low sample rate... Does not seem to be working as it should be not recording enough data?
    digitalWrite(LED, HIGH);
  }
   
}
