#include "FS.h"
#include "SD.h"
#include "SPI.h"

int SD_CS = 5;

String dataMessage;

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define seaLevelPressure_hPa 1024

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

float altitude, acceleration;
float q = 0.0001;

float s,v,ac, reac, res;

// The system dynamics
BLA::Matrix<3, 3> A = {1.0, 0.05, 0.00125,
                        0, 1.0, 0.05,
                        0, 0, 1};

// Relationship between measurement and states
BLA::Matrix<2, 3> H = {1.0, 0, 0,
                        0, 0, 1.0};

// Initial posteriori estimate error covariance
BLA::Matrix<3, 3> P = {1, 0, 0,
                        0, 1, 0, 
                        0, 0, 1};

// Measurement error covariance
BLA::Matrix<2, 2> R = {0.5, 0,
                        0, 0.0012};

// Process noise covariance
BLA::Matrix<3, 3> Q = {0.0001, 0, 0,
                        0, 0.0001, 0, 
                        0, 0, 0.0001};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};

BLA::Matrix<3, 1> x_hat = {1530.0,
                            0.0,
                            0.0};

BLA::Matrix<2, 1> Y = {0.0,
                       0.0};


void setup() {
 Serial.begin(115200);
  // put your setup code here, to run once:
SD.begin(SD_CS); 
if(!SD.begin(SD_CS)) {
  Serial.println("Card Mount Failed");
  return;
}
uint8_t cardType = SD.cardType();
if(cardType == CARD_NONE) {
  Serial.println("No SD card attached");
  return;
}
Serial.println("Initializing SD card...");
if (!SD.begin(SD_CS)) {
  Serial.println("ERROR - SD card initialization failed!");
  return; // init failed
}

File file = SD.open("/data1.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data1.txt", "Altitude, Velocity, Acceleration \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
  //logSDCard();

Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  

  Serial.println("");
  delay(100);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

}

void loop() {
  // put your main code here, to run repeatedly:

sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

    altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
  acceleration = a.acceleration.z;

    //Measurement matrix

    BLA::Matrix<2, 1> Z = {altitude,
                        acceleration};
    //Predicted state estimate
    BLA::Matrix<3, 1> x_hat_minus = A * x_hat;
    
    //Predicted estimate covariance
    BLA::Matrix<3, 3> P_minus = A * P * (~A) + Q;

    //Kalman gain
    BLA::Matrix<3, 2> K  = P_minus * (~H) * ((H * P_minus * (~H) + R)).Inverse();


    //Measurement residual
    Y = Z - (H * x_hat_minus);
    
    //Updated state estimate
    x_hat = x_hat_minus + K * Y;

    //Updated estimate covariance
    P = (I - K * H) * P_minus;


    x_hat = x_hat_minus + K * (Z - (H * x_hat_minus));
    P = (I - K * H) * P_minus;
    Y = Z - (H * x_hat_minus);

  //  Y = 0;
    
    
    s = x_hat(0);
    v = x_hat(1);
    ac = x_hat(2);
    res = Z(0);
  
    reac = Z(1);
logSDCard();
delay(20);
}




// Write the sensor readings on the SD card
void logSDCard() {
  
  dataMessage = String(s) + "," + String(v) + "," + String(ac) + "," +  "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data1.txt", dataMessage.c_str());
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
