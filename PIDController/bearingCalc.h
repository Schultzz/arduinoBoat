#include <math.h>

TinyGPS gps;
SoftwareSerial ss(13, 12);

float curLat = 1000.00;
int noSignal = 2;
int gpsSignalDelay = 500;

float curLon, oldLon, oldLat;
float goalLat, goalLon, curBearing, goalBearing, flat, flon;
unsigned long age;

void setupThis(){
  Serial.println("setupThis - bearingCalc.h");
  ss.begin(9600);
  }

// SmartDelay START:::::

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

// SmartDelay END:::::

// gpsSignal START:::::

boolean gpsSignal() {

  while(curLat == noSignal){
    gps.f_get_position(&curLat, &curLon, &age);
    smartdelay(100);
    Serial.print("her: ");
    Serial.println(curLat);
  }
  return true;
}

// gpsSignal END:::::

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

float getErrorMargin(float goalLon, float goalLat) {

    oldLat = curLat;
    oldLon = curLon;
    
    gps.f_get_position(&curLat, &curLon, &age);
    smartdelay(gpsSignalDelay);

    float oldCurBearing = CalculateHeading(curLat, curLon, oldLat, oldLon);

    float curGoalBearing = CalculateHeading(curLat, curLon, goalLon, goalLat);

    return curGoalBearing - oldCurBearing;
}


