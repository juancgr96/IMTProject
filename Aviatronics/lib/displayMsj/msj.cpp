#include "msj.h"

displayMessage::displayMessage()
{
}

displayMessage::~displayMessage()
{
}

void displayMessage::dispMes(float lat, float ing)
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
}
