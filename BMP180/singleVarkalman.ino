#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

float altitude, estimated_altitude;

float f_1 = 1.00000; //cast as float
float update_x = 0;
float prev_x = 0;
float update_p = 0;
float prev_p = 0;
float kalman_gain = 0;
float error_cov = 0.05;
float process_noise = 1e-3;
float x_temp = 0;
float p_temp = 0;


void setup() {
  // open a serial connection to display values
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}

void loop() {
  altitude = bmp.readAltitude(102400);
  estimated_altitude = singleVarKalmanCal(altitude);
  Serial.print(altitude);
  Serial.print(" , ");
  Serial.println(estimated_altitude);
  delay(20);
}

//singleVarKalmanCal() - Calculates new Kalman values from float value "altitude"
float singleVarKalmanCal (float altitude) {

  //Predict x_temp, p_temp
  x_temp = prev_x;
  p_temp = prev_p + process_noise;

  //Update kalman values
  kalman_gain = (f_1 / (p_temp + error_cov)) * p_temp;
  update_x = x_temp + (kalman_gain * (altitude - x_temp));
  update_p = (f_1 - kalman_gain) * p_temp;

  //Save this state for next time
  prev_x = update_x;
  prev_p = update_p;

  return update_x;
}

