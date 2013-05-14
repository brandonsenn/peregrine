// =============================================================================
// 
// Peregrine "Bird" firmware for Peregrine S-UAV lead follow pair
//
//==============================================================================


// libraries
//-------------
#include <Wire.h> // for 2-wire I2C serial comm
#include <FastSerial.h> // for high baud serial comm w/ GPS
#include <AP_GPS.h> // for 3D Robotics MediaTek GPS drivers
#include <AP_Common.h> // Ardu-Pilot Common Functions (required for AP_GPS fn's)
#include <AP_Math.h> // Ardu-Pilot Math Functions (required for AP_GPS fn's)
#include <Adafruit_BMP085.h>  // for pre-canned barometric pressure fns
#include <Servo.h> // for camera pointing servo control
#include <LSM303.h> // Pololu imu-9, compass module.
#include <L3G.h> //  Polulu imu-9, gyro accelerometer module
//--------------

// variables
//--------------
Adafruit_BMP085 bmp; // barometric pressure mapped to 'bmp'
LSM303 compass; // magentometer / accelerometer mapping
L3G gyro; // gyroscope mapping
FastSerialPort0(Serial); // map serial0 to fast fn
FastSerialPort1(Serial1); // map serial1 to fast fn
FastSerialPort2(Serial2); // map serial2 to fast fn
AP_GPS_MTK16 gps(&Serial1); // map gps unit to serial1
#define T6 1000000 // divisor for GPS readbacks
#define T7 10000000 // divisor for GPS readback
double BALTD; // barometric altitude double precision floating point var
double LAT; //   Latitude double precision floating point var
double LON; //   Longitude double precision floating point var
double GALTD; // GPS determined altitude double precision floating point var
Servo myservo;  // create servo object to control a servo 
int spos = 0;    // variable to store the servo position 
//-------------



//====================
//  MAIN PROGRAM
//====================

// initialize
void setup(){
  initialize();
}
   
// main program loop
void loop() 
{
   daq();
   comm();
   modes();
   controls(); 
   output(); 
}

//======================================================
// Data Acquisition (DAQ) function
//------------------------------------------------------

void daq() {
     BALTD = bmp.readAltitude(102500); // read barometric pressure corrected for sea level
     compass.read();
     int heading = compass.heading((LSM303::vector){0,-1,0});
     gyro.read();
     gps.update();
     if (gps.new_data) {
        //Serial.print("gps:");
        //Serial.print(" Lat:");
        Serial.print((float)gps.latitude / T7, DEC);
        //Serial.print(" Lon:");
        Serial.println((float)gps.longitude / T7, DEC);
        //Serial.print(" Alt:");
        //Serial.print((float)gps.altitude / 100.0, DEC);
        //Serial.print(" GSP:");
        //Serial.print(gps.ground_speed / 100.0);
        //Serial.print(" COG:");
        //Serial.print(gps.ground_course / 100.0, DEC);
        //Serial.print(" SAT:");
        //Serial.print(gps.num_sats, DEC);
        //Serial.print(" FIX:");
        //Serial.print(gps.fix, DEC);
        //Serial.print(" TIM:");
        //Serial.print(gps.time, DEC);
        gps.new_data = 0; // We have read the data
    }

}

//============================================
// Output Function
//--------------------------------------------

void output() { 
   myservo.write(0);              // tell servo to go to position 0
   //Serial.print("altitude = ");
   Serial.print(BALTD);
   //Serial.print(" meters");
  //Serial.print(" A ");
  Serial.print("aX: ");
  Serial2.print((int)compass.a.x);
  Serial2.print(",");
  Serial.print(" aY: ");
  Serial2.print((int)compass.a.y);
  Serial2.print(",");
  Serial.print(" aZ: ");
  Serial2.println((int)compass.a.z);
  //Serial.print(" M ");  
  //Serial.print("X: ");
  //Serial.print((int)compass.m.x);
  //Serial.print(" Y: ");
  //Serial.print((int)compass.m.y);
  //Serial.print(" Z: ");
  //Serial.print((int)compass.m.z);
  Serial.print(" Heading:");
  Serial.print(heading);
  //Serial.print(" G ");
  Serial.print("gX: ");
  Serial.print((int)gyro.g.x);
  Serial.print(" gY: ");
  Serial.print((int)gyro.g.y);
  Serial.print(" gZ: ");
  Serial.println((int)gyro.g.z);
  //myservo.write(180);              // tell servo to go to position 180
  //Serial.print(millis());
}

//==============================================================
// setup function
//--------------------------------------------------------------

void initialize() 
{
  Wire.begin();        // start i2c communication
  myservo.attach(9);   // attaches the servo on pin 9 to the servo object 
  compass.init();      // initialize compass
  compass.enableDefault(); // enable default config for compass
  // Calibration values. Used the Calibrate example program to get the values for my IMU
  compass.m_min.x = -470; compass.m_min.y = -612; compass.m_min.z = -443;
  compass.m_max.x = +449; compass.m_max.y = +516; compass.m_max.z = 566;
  gyro.init(); // initialize gyro
  gyro.enableDefault();   // enable default config for gyro
  Serial.begin(9600);   // start serial port at 9600 baud
  bmp.begin();          // start up barometric pressure sensor
  Serial1.begin(38400); // start up FastSerial GPS port
  Serial2.begin(9600); // start up FastSerial XBee port
  gps.print_errors = true;
  gps.init();	 // GPS Initialization
  delay(1000);
}


//============================================
// Communication Function
//--------------------------------------------

void comm(){
  
}

//============================================
// Mode Selection Function
//--------------------------------------------

void modes(){
  
}

//============================================
// Control Outpu Calculation Function
//--------------------------------------------

void controls(){
  
}

//============================================
// Output Function
//--------------------------------------------

void output(){
  
}


