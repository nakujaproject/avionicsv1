#include "FS.h"
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

// We have three scenarios:
// 1. When we start decreasing altitude
// 2. When we reach 0 vertical velocity
// 3. When leaves - 9.8 
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

#define seaLevelPressure_hPa 1024
const int SD_CS = 5;

String dataMessage;

int counter = 0;
long count = 0;

float altitude, velocity, ax, ay, az, kalmanAltitude;
float liftoffAltitude, prevAltitude, apogeeAltitude;
int measures;
bool isApogee1 = false;
bool isApogee2 = false;
bool isApogee3 = false;


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
BLA::Matrix<3, 3> Q = {q, 0, 0,
                        0, q, 0, 
                        0, 0, q};

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
    delay(2000);
    init_components();
    delay(2000);
}

void loop() {
	// put your main code here, to run repeatedly:

	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

	altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
	az = a.acceleration.z;

	//Measurement matrix

	BLA::Matrix<2, 1> Z = {altitude,
							az};
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
	Y = Z - (H * x_hat_minus);

	s = x_hat(0);
	v = x_hat(1);
	ac = x_hat(2);
	
	res = Z(0);
	reac = Z(1);

	logSDCard();
	counter++;
	delay(50);

}


// Write the sensor readings on the SD card
void logSDCard() {
  
  dataMessage = String(count) + "," + String(altitude) + "," + String(s) + "," + String(v) + "," + String(ac) + "," + String(ax) + "," + String(ay) + "," + String(az)  + "," + String(res) + "," + String(reac) + "," + String(isApogee1) + "," + String(isApogee2) + "," + String(isApogee3) + ","  "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/loggedData.txt", dataMessage.c_str());
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


void detectApogee1(float altitude) {
     //detect apogee
    if (altitude > liftoffAltitude) {
        if (altitude < prevAltitude ) {
            measures -= 1;
            if (measures == 0) {
                apogeeAltitude = altitude;
                isApogee1 = true;
            }
            else {
                prevAltitude = altitude;
                measures = 5;
            }
        }
    }
}

void detectApogee2(float velocity){
    if (velocity < 0){
        isApogee2 = true;
    }
}

void detectApogee3(float acceleration){
    bool isUnder = false;
    if (acceleration < 0){
        isUnder = true;
    }
    if ((acceleration > 0) && (isUnder == true)) {
        isApogee3 = true;
    }
}

void startWriting(fs::FS &fs) {

    File file = SD.open("/loggedData.txt");
	if(!file) {
		Serial.println("File doens't exist");
		Serial.println("Creating file...");
		writeFile(SD, "/loggedData.txt", "Index, Altitude, kalmanAltitude, kalmanVelocity, kalmanAcceleration, ax, ay, az, res, reac, isApogee1, isApogee2, isApogee3  \r\n");
	}
	else {
		Serial.println("File already exists");  
	}
	file.close();
}

void init_components(){
    Serial.println("BMP180 test!");
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
    Serial.println("BMP180 Found!");

    Serial.println("MPU6050 test!");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    Serial.print("\nInitializing SD card...");
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!SD.begin(SD_CS)) {
        Serial.println("initialization failed.");
        while (1);
    } else {
        Serial.println("Wiring is correct and a card is present.");
    }
    Serial.println("initialization done.");
    startWriting(SD);

	uint8_t cardType = SD.cardType();
	if(cardType == CARD_NONE) {
		Serial.println("No SD card attached");
		return;
	}
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}


float get_velocity(){
    return velocity;
}

float get_acceleration(){
    return acceleration;
}

float get_kalmanAltitude(){

}

