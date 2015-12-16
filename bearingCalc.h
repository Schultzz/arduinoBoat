#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(4, 3);

float oldLat = 0;
float oldLon = 0;

float curLat = 0;
float curLon = 0;

float goalLat, goalLon;
float curBearing, goalBearing;

float flat, flon;
unsigned long age;

boolean gpsSignal() {

  if (curLat != 0) {
    return true;
  }
  return false;
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  }
  while (millis() - start < ms);
}

//Udregner Bearing
double CalculateHeading(float lat1, float long1, float lat2, float long2)
{
  float a = lat1 * PI / 180;
  float b = long1 * PI / 180;
  float c = lat2 * PI / 180;
  float d = long2     * PI / 180;

  if (cos(c) * sin(d - b) == 0)
    if (c > a)
      return 0;
    else
      return 180;
  else
  {
    float angle = atan2(cos(c) * sin(d - b), sin(c) * cos(a) - sin(a) * cos(c) * cos(d - b));
    return fmod(angle * 180 / PI + 360, 360);
  }
}

float getErrorMargin(float goalLt, float goalLn) {

  if (curLat == 0) {

    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);

    while (flat == 1000) {
      gps.f_get_position(&flat, &flon, &age);
      smartdelay(200);
    };

    curLat = flat;
    curLon = flon;

    smartdelay(1000);

    return 0.0f;
  }
  else {
    oldLat = curLat;
    oldLon = curLon;

    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    smartdelay(2000);

    curLat = flat;
    curLon = flon;

    float oldCurBearing;
    float curGoalBearing;

    oldCurBearing = CalculateHeading(curLat, curLon, oldLat, oldLon);

    curGoalBearing = CalculateHeading(curLat, curLon, goalLon, goalLat);

    return curGoalBearing - oldCurBearing;

  }

}
