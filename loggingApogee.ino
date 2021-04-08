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
const int chipSelect = 5;

int counter = 0;
long count = 0;

float altitude, velocity, acceleration, kalmanAltitude;
float liftoffAltitude, prevAltitude, apogeeAltitude;
int measures;
bool isApogee1 = false;
bool isApogee2 = false;
bool isApogee3 = false;

float q = 0.0001;
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
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
    velocity = get_velocity();
    ax = a.acceleration.x;
    ax = a.acceleration.y;
    ax = a.acceleration.z;
    kalmanAltitude = get_kalmanAltitude();
    Write_SDcard(SD, counter, altitude, kalmanAltitude, velocity, ax, ay, az, isApogee1, isApogee2, isApogee3);
    delay(50);
    counter ++;

    
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
    File dataFile = SD.open("/Altitude.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Altitude"); //Write the first row of the excel file
        dataFile.println(); //End of Row move to next row
        dataFile.close();
    }
}

void Write_SDcard(fs::FS &fs, int counter, float altitude, float kalmanAltitude, float velocity, float ax, float ay, float az, bool STATUS1, bool STATUS2, bool STATUS3) {
    File dataFile = fs.open("/Altitude.txt", FILE_APPEND);
    if (dataFile) {
        dataFile.print(counter);
        dataFile.print(" , ");
        dataFile.print(altitude);
        dataFile.print(", ");
        dataFile.print(kalmanAltitude);
        dataFile.print(", ");
        dataFile.print(velocity);
        dataFile.print(", ");
        dataFile.print(ax);
        dataFile.print(", ");
        dataFile.print(ay);
        dataFile.print(", ");
        dataFile.print(az);
        dataFile.print(", ");
        dataFile.print(STATUS1);
        dataFile.print(", ");
        dataFile.print(STATUS2);
        dataFile.print(", ");
        dataFile.println(STATUS3);
        dataFile.close(); //Close the file
    }
    else {
        Serial.println("SD card writing failed");
    }
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
    if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed.");
        while (1);
    } else {
        Serial.println("Wiring is correct and a card is present.");
    }
    Serial.println("initialization done.");
    startWriting(SD);
}


float get_velocity(){
    return velocity;
}

float get_acceleration(){
    return acceleration;
}

float get_kalmanAltitude(){

}













