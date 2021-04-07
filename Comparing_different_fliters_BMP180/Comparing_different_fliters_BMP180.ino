//output realvalue, mav1,mav2,mav3,expo,runningmedian, digital,kalman


#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "RunningMedian.h"
#include <MegunoLink.h>

#include <Filter.h>
#define seaLevelPressure_hPa 1024
 
// Create a new exponential filter with a weight of 5 and an initial value of 0. 
int FilterWeight = 5;

ExponentialFilter<float> ADCFilter(FilterWeight, 0);

int counter = 0;

#define WINDOW_SIZE 3

int INDEX = 0;
float VALUE = 0;
float SUM = 0;
int READINGS[WINDOW_SIZE];
float AVERAGED = 0;

#define WINDOW_SIZE_2 2

int INDEX_2 = 0;
float VALUE_2 = 0;
float SUM_2 = 0;
int READINGS_2[WINDOW_SIZE_2];
float AVERAGED2 = 0;

#define WINDOW_SIZE_1 1

int INDEX_1 = 0;
float VALUE_1 = 0;
float SUM_1 = 0;
int READINGS_1[WINDOW_SIZE_1];
float AVERAGED1 = 0;



Adafruit_BMP085 bmp;

RunningMedian samples = RunningMedian(15);

long count = 0;

#define filterSamples   13              // filterSamples should  be an odd number, no smaller than 3
float sensSmoothArray1 [filterSamples];   // array for holding raw sensor values for sensor1 

float rawData1, smoothData1;  // variables for sensor1 data

//#include <firFilter.h>

//firFilter Filter;



//float firvalue;

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

void setup() {
  Serial.begin(115200);
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}
float READING;
//moving average
float mav3() {
  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = bmp.readAltitude();        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

  return AVERAGED;
}

float mav2() {
  SUM_2 = SUM_2 - READINGS_2[INDEX_2];       // Remove the oldest entry from the sum
  VALUE_2 = bmp.readAltitude();        // Read the next sensor value
  READINGS_2[INDEX_2] = VALUE_2;           // Add the newest reading to the window
  SUM_2 = SUM_2 + VALUE_2;                 // Add the newest reading to the sum
  INDEX_2 = (INDEX_2 + 1) % 2; // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED2 = SUM_2 / 2;      // Divide the sum of the window by the window size for the result

  return AVERAGED2;
}

float mav1() {
  SUM_1 = SUM_1 - READINGS_1[INDEX_1];       // Remove the oldest entry from the sum
  VALUE_1 = bmp.readAltitude();        // Read the next sensor value
  READINGS_1[INDEX_1] = VALUE_1;           // Add the newest reading to the window
  SUM_1 = SUM_1 + VALUE_1;                 // Add the newest reading to the sum
  INDEX_1 = (INDEX_1 + 1) % 1; // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED1 = SUM_1 / 1;      // Divide the sum of the window by the window size for the result

  return AVERAGED1;
}

//expofilter
float expofiltered;

float expofilter() {
  float RawValue = bmp.readAltitude() ;
  ADCFilter.Filter(RawValue);
  expofiltered = ADCFilter.Current();
  return expofiltered; 

}

float runmedian()
{
  float x = bmp.readAltitude();
      
  //Serial.print(x);
  

  samples.add(x);


  float m = samples.getMedian();
 // float a = samples.getAverage();
 // float a3 = samples.getAverage(3);
  
  //Serial.print(millis());
  return m;

}

float digital(){
  rawData1 =   bmp.readAltitude();                      // read sensor 1
  smoothData1 = digitalSmooth(rawData1, sensSmoothArray1);  // every sensor you use with digitalSmooth needs its own array
return smoothData1;
  }
  float digitalSmooth(float rawIn, float *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k;
  int temp, top, bottom;
  double total;
  static int i;
 // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

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


  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}


//fir filter
/*float fir(){
  float reading = bmp.readAltitude(seaLevelPressure_hPa * 100);
  firvalue = Filter.run(reading);
  return firvalue;
  }*/


float kalman(){
  float estimated_altitude = pressureKalmanFilter.updateEstimate(bmp.readAltitude());
  return estimated_altitude;
  }
void loop(){
//Serial.println("Real altitude = ");

  VALUE = bmp.readAltitude();
  
  Serial.print(counter);
  Serial.print(", ");
  
  Serial.print(VALUE);
  Serial.print(", ");
  Serial.print(mav3());
  
  Serial.print(", ");
  Serial.print(mav2());
  
  Serial.print(", ");
  Serial.print(mav1());

  Serial.print(", ");
  Serial.print(expofilter());

  Serial.print(", ");
  Serial.print(runmedian());

  Serial.print(", ");
  Serial.print(digital());

  /*Serial.print(", ");
  Serial.println(fir());*/
  
  Serial.print(", ");
  Serial.println(kalman());

  

  counter++;

  delay(10);
}



 
