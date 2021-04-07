#include <SimpleKalmanFilter.h>
//#include <SFE_BMP180.h>

#include <Wire.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1024


Adafruit_BMP085 bmp;

SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);



  int count = 0;
void setup() {
  Serial.begin(115200);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
}
  
void loop() {
    Serial.print(count);
    Serial.print(", ");
    Serial.print(bmp.readAltitude());
    Serial.print(", ");
    float estimated_altitude = pressureKalmanFilter.updateEstimate(bmp.readAltitude());
    Serial.print(estimated_altitude);
    
    Serial.println();
    count++;
    delay(20);
    
}
