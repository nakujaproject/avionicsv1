#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1016

boolean deployedAlready = false;

Adafruit_BMP085 bmp;

Servo servo;

float alt;
float prev = 0;
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
  servo.attach(10);
  //servo.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
prev =alt;
//Serial.print("prev: ");
//Serial.println(prev);++
alt= bmp.readAltitude();
//Serial.print("new alt: ");
Serial.println(alt);
//Serial.println(deployedAlready);
//if(deployedAlready = false){
//if (prev < alt){
  //Serial.println("wait");
  //}

  
  //if((prev>alt) && (deployedAlready == 0)){
    //Serial.println("deploy");
    
    //delay(500);
    //servo.write(90); 
    //deployedAlready = true;
    //}
    

  delay(20);
}
