#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
// We have three scenarios:
// 1. When we start decreasing altitude
// 2. When we reach 0 vertical velocity
// 3. When leaves - 9.8 
Adafruit_BMP085 bmp;

#define seaLevelPressure_hPa 1024
const int chipSelect = 53;

int counter = 0;
long count = 0;

float liftoffAltitude, prevAltitude, apogeeAltitude;
int measures;
bool isApogee1 = false;
bool isApogee2 = false;
bool isApogee3 = false;

void setup() {
    Serial.begin(9600);
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
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
    startWriting();
}

void loop() {
    altitude = bmp.readAltitude();
    Write_SDcard(altitude, kalmanAltitude, velocity, acceleration, isApogee1, isApogee2, isApogee3);
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
    if (acceleration > 0) && (isUnder == true) {
        isApogee3 = true;
    }
}

void startWriting() {
    File dataFile = SD.open("Altitude.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Altitude"); //Write the first row of the excel file
        dataFile.println(); //End of Row move to next row
        dataFile.close();
    }
}

void Write_SDcard(int counter, float altitude, float kalmanAltitude, float velocity, float acceleration, bool STATUS, bool STATUS2, bool STATUS3) {
    File dataFile = SD.open("Altitude.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.print(counter);
        dataFile.print(" , ");
        dataFile.print(altitude);
        dataFile.print(", ");
        dataFile.print(kalmanAltitude);
        dataFile.print(", ");
        dataFile.print(velocity);
        dataFile.print(", ");
        dataFile.print(acceleration);
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