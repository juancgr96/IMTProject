#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BluetoothSerial.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPS++.h>

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
unsigned long lastTime = 0;
BluetoothSerial SerialBT;
#define I2C_SPEED 115200
I2CGPS myGPS;    //I2C OBJECT GPS
TinyGPSPlus gps; //Declare gps object
//Declaracion de funciones

void displayMessage(float lat, float lng);
//
float LatA;
float LonA;
int counter = 0;
int counterGPS = 0;

uint16_t generalCounter = 0;

void setup() //This code is executed once
{
  SerialBT.begin("ReadSensorBT");
  Serial.begin(115200);
  //imu.begin();

  Serial.println("Configuring GPS!");
  if (myGPS.begin(Wire, I2C_SPEED) == false)
  {
    Serial.println("Module is not Working.");
    while (1)
      ; //Freeze!
  }
  Serial.println("GPS is Working!");
} //Fin Setup

void loop()
{
  if ((millis() - lastTime) >= 100) //To stream at 10Hz without using additional timers
  {
    lastTime = millis();
    //imu.readSensor();
    //SerialBT.printf("%.3f,%.3f,%.3f\n", imu.euler[0], imu.euler[1], imu.euler[2]);

    //Serial.print("Time Stamp: "); //To read out the Time Stamp
   // Serial.println(lastTime);

    //Serial.print("Heading(Yaw): "); //To read out the Heading (Yaw)
    //Serial.println(imu.euler[0]);   //Convert to degrees

    //Serial.print("Roll: ");       //To read out the Roll
    //Serial.println(imu.euler[1]); //Convert to degrees

    //Serial.print("Pitch: ");      //To read out the Pitch
    //Serial.println(imu.euler[2]); //Convert to degrees

    //Serial.println(); //Extra line to differentiate between packets

    while (myGPS.available())
    {
      gps.encode(myGPS.read()); //Obtain info from GPS
    }

    if (gps.time.isUpdated())
    {
      counterGPS += 1;
      if (counterGPS == 100)
      {
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        generalCounter = generalCounter + 1;
        if (generalCounter >= 30)
        {
          lat = gps.location.lat();
          lng = gps.location.lng();
          LatA = lat;
          LonA = lng;
          displayMessage(LatA, LonA);
        }
        else
        {
          Serial.println("Obteniendo datos");
          Serial.print(lat);
          Serial.print(",");
          Serial.println(lng);
        }
        counterGPS = 0;
      }
    }
  }
} //FIn loop
void displayMessage(float lat, float lng)
{
  String message;
  uint8_t speed = gps.speed.kmph();
  uint8_t sat = gps.satellites.value();
  Serial.println("MESSAGE:");
  message.concat(String(lat, 9));
  message.concat(",");
  message.concat(String(lng, 9));
  message.concat(",");
  message.concat(String(speed, 2));
  message.concat(",");
  message.concat(sat);
  message.concat("Hour:");
  message.concat(gps.time.hour());
  message.concat(":");
  message.concat(gps.time.minute());
  message.concat(":");
  message.concat(gps.time.second());
  Serial.println(message);
} //Fin Message