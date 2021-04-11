#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "RunningMedian.h"
#include <SimpleKalmanFilter.h>

Adafruit_BMP085 bmp;
RunningMedian samples = RunningMedian(15);
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

#define seaLevelPressure_hPa 1024
#define filterSamples   13              // filterSamples should  be an odd number, no smaller than 3

const int chipSelect = 53;
int counter = 0;
long count = 0;
float sensSmoothArray1 [filterSamples];   // array for holding raw sensor values for sensor1 
float rawData1, smoothData1, altitude;  // variables for sensor1 data

float runmedian(float x) {
    samples.add(x);
    float m = samples.getMedian();
    return m;
}

float digital(float x) {
    smoothData1 = digitalSmooth(x, sensSmoothArray1);  // every sensor you use with digitalSmooth needs its own array
    return smoothData1;
}

float digitalSmooth(float rawIn, float *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
    int j, k, temp, top, bottom;
    double total;
    static int i;
    static int sorted[filterSamples];
    boolean done;
    i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
    sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot
    for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
        sorted[j] = sensSmoothArray[j];
    }
    done = 0;                // flag to know when we're done sorting              
    while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
        done = 1;
        for (j = 0; j < (filterSamples - 1); j++){
            if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
                temp = sorted[j + 1];
                sorted [j+1] =  sorted[j] ;
                sorted [j] = temp;
                done = 0;
            }
        }
    }
    bottom = max(((filterSamples * 15)  / 100), 1); 
    top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
    k = 0;
    total = 0;
    for ( j = bottom; j< top; j++){
        total += sorted[j];  // total remaining indices
        k++; 
    }
    return total / k;    // divide by number of samples
}

float kalman(float x) {
    float estimated_altitude = pressureKalmanFilter.updateEstimate(x);
    return estimated_altitude;
}

void printSerial(float altitude) {
    Serial.print(counter);
    Serial.print(" , ");
    Serial.print(altitude);
    Serial.print(", ");
    Serial.print(runmedian(altitude));
    Serial.print(", ");
    Serial.print(digital(altitude));
    Serial.print(", ");
    Serial.println(kalman(altitude));
} 

void startWriting() {
    File dataFile = SD.open("Altitude.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Altitude"); //Write the first row of the excel file
        dataFile.println(); //End of Row move to next row
        dataFile.close();
    }
}

void Write_SDcard(float altitude) {
    File dataFile = SD.open("Altitude.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.print(counter);
        dataFile.print(" , ");
        dataFile.print(altitude);
        dataFile.print(", ");
        dataFile.print(runmedian(altitude));
        dataFile.print(", ");
        dataFile.print(digital(altitude));
        dataFile.print(", ");
        dataFile.println(kalman(altitude));
        dataFile.close(); //Close the file
    }
    else {
        Serial.println("SD card writing failed");
    }
}
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
    printSerial(altitude);
    Write_SDcard(altitude);
    delay(50);
    counter ++;
}
