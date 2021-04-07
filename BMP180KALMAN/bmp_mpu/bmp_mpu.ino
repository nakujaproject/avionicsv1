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



void setup(void) {
  Serial.begin(115200);
  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens

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
int count = 0;
void loop() {
  /* Get new sensor events with the readings */
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

    
    
    
  //  Y = 0;
    float s,v,ac, reac, res;
    
    s = x_hat(0);
    v = x_hat(1);
    ac = x_hat(2);
    res = Z(0);
    
    reac = Z(1);
    
    //Serial.print(count);
    //Serial.print(", ");
    
    Serial.print(res);
    Serial.print(", ");
    Serial.print(s);
    Serial.print(", ");
    Serial.print(reac);
    Serial.print(", ");
    Serial.println(ac);
    //Serial.print(ac);Serial.print("\t");
    //Serial.println(reac);Serial.println("\t");
    delay(20);

count ++;
}
