#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>

Adafruit_BNO055 bno(55);
Preferences pref;

void setup()
{
  pref.begin("offset", false);
  adafruit_bno055_offsets_t calibrationData;
  // sensor_t sensor;
  // bno.getSensor(&sensor);
  bno.setExtCrystalUse(true);
  
}