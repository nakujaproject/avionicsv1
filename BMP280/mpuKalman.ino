#include <Wire.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>
#include <MPU6050_tockn.h>

using namespace BLA;

MPU6050 mpu6050(Wire);

float q = 0.0001;
float q2 = 0.1;

// The system dynamics
BLA::Matrix<4, 4> A = {1.0, 0.005, 0.0000125, 0,
                        0, 1.0, 0.005, 0,
                        0, 0, 1.0, 0,
                        0, 0, 0, 1};

// Relationship between measurement and states
BLA::Matrix<3, 4> H = {1.0, 0, 0, 0,
                        0, 1.0, 0, 1.0,
                        0, 0, 1.0, 0};

// Initial posteriori estimate error covariance
BLA::Matrix<4, 4> P = {0.039776, 0, 0, 0,
                        0, 0.012275, 0, 0,
                        0, 0, 0.018422, 0,
                        0, 0, 0, 0};

// Initial posteriori estimate error covariance
BLA::Matrix<4, 4> P2 = {0.039776, 0, 0, 0,
                        0, 0.012275, 0, 0,
                        0, 0, 0.018422, 0,
                        0, 0, 0, 0};

// Measurement error covariance
BLA::Matrix<3, 3> R = {0.039776, 0, 0,
                        0, 0.012275, 0,
                        0, 0, 0.018422};

// Process noise covariance
BLA::Matrix<4, 4> Q = {q, 0, 0, 0,
                        0, q, 0, 0,
                        0, 0, q, 0,
                        0, 0, 0, q};

// Process noise covariance
BLA::Matrix<4, 4> Q2 = {q2, 0, 0, 0,
                        0, q2, 0, 0,
                        0, 0, q2, 0,
                        0, 0, 0, q2};

// Identity Matrix
BLA::Matrix<4, 4> I = {1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1};

BLA::Matrix<4, 1> x_hat = {0.0,
                            0.0,
                            0.0,
                            0.0};

BLA::Matrix<4, 1> x_hat2 = {0.0,
                            0.0,
                            0.0,
                            0.0};

// used to find sensors means and variances
#define NUM_OF_ITERATIONS 200
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch_comp, yaw, pitch_kal;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
float AngAccZ;
float intergral = 0.0;
int c = 0;
int pos;
long counter = 0;
long GyroErrors = 0;
bool isFirstRun = true;
long loopTime = 10000;
unsigned long timer = 0;
int WhichError = 0;


void setup() {
    Serial.begin(115200);
    timer = micros();
    // init_mpu();
    // // Call this function if you need to get the IMU error values for your module
    // GyroErrorX = calculate_IMU_error(0);
    // GyroErrorY = calculate_IMU_error(1);
    // GyroErrorZ = calculate_IMU_error(2);
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    delay(20);
}

void loop() {
    // === Read acceleromter data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 - 0.02; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 - 0.02; // Z-axis value

    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
    AngAccZ = AccZ - 1 + sin(accAngleY);

    // === Read gyroscope data === //
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

    Wire.beginTransmission(MPU);
    Wire.write(0x45); // Gyro data Y first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true); // Read 2 registers total, each axis value is stored in 2 registers
    
    // GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    // GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    
    // Correct the outputs with the calculated error values
    GyroX = GyroX + GyroErrorX; // GyroErrorX ~(-0.56)
    GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
    GyroZ = GyroZ + GyroErrorZ; // GyroErrorZ ~ (-0.8)
    // mpu6050.update();

    // GyroX = mpu6050.getGyroX();
    // GyroY = mpu6050.getGyroY();
    // GyroZ = mpu6050.getGyroZ();

    // AccX = mpu6050.getAccX();
    // AccY = mpu6050.getAccY();
    // AccZ = mpu6050.getAccZ();

    Serial.print("accX : ");Serial.print(AccX);
    Serial.print("\taccY : ");Serial.print(AccY);
    Serial.print("\taccZ : ");Serial.println(AccZ);
    
    if (isFirstRun){
        gyroAngleY = accAngleY;
        BLA::Matrix<4, 1> x_hat = {accAngleY,
                                    GyroY,
                                    AngAccZ,
                                    0};
        BLA::Matrix<4, 1> x_hat2 = {accAngleY,
                                    GyroY,
                                    AngAccZ,
                                    0};
    }
    isFirstRun = false;

    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    // gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    // yaw =  yaw + GyroZ * elapsedTime;
    
    // Complementary filter - combine acceleromter and gyro angle values
    // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch_comp = float (0.96 * gyroAngleY + 0.04 * accAngleY);

    BLA::Matrix<3, 1> z = {accAngleY,
                            GyroY,
                            AngAccZ};
    BLA::Matrix<4, 1> x_hat_minus = A * x_hat;
    BLA::Matrix<4, 4> P_minus = A * P * (~A) + Q;
    BLA::Matrix<4, 3> K = P_minus * (~H) * ((H * P_minus * (~H) + R)).Inverse();
    x_hat = x_hat_minus + K * (z - (H * x_hat_minus));
    P = (I - K * H) * P_minus;

    BLA::Matrix<4, 1> x_hat_minus2 = A * x_hat2;
    BLA::Matrix<4, 4> P_minus2 = A * P2 * (~A) + Q2;
    BLA::Matrix<4, 3> K2 = P_minus2 * (~H) * ((H * P_minus2 * (~H) + R)).Inverse();
    x_hat2 = x_hat_minus2 + K2 * (z - (H * x_hat_minus2));
    P2 = (I - K2 * H) * P_minus2;

    float pitch_kal = x_hat(0);
    float pitch_kal2 = x_hat2(0);
    float demo0 = x_hat(0);
    float demo1 = x_hat(1);
    float demo2 = x_hat(2);
    float demo3 = x_hat(3);
    
    Serial.print(demo0);Serial.print("\t");
    Serial.print(demo1);Serial.print("\t");
    Serial.print(demo2);Serial.print("\t");
    Serial.print(demo3);Serial.println("\t");
    // Serial.print(pitch_kal);
    // Serial.print("\t");
    // Serial.println(pitch_kal2);
    //sendToPC(&pitch_kal, &pitch_kal2);

}

float calculate_IMU_error(int WhichError) {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  float result;
  float AccErrorX = 0, AccErrorY = 0, AccErrorZ = 0;
  float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  c = 0;

  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / NUM_OF_ITERATIONS;
  AccErrorY = AccErrorY / NUM_OF_ITERATIONS;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / NUM_OF_ITERATIONS;
  GyroErrorY = GyroErrorY / NUM_OF_ITERATIONS;
  GyroErrorZ = GyroErrorZ / NUM_OF_ITERATIONS;

  if (WhichError == 0){
      result = GyroErrorX;
  }
  else if (WhichError == 1){
      result = GyroErrorY;
  }
  else {
      result = GyroErrorZ;
  }
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  return result;
}

void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}

void sendToPC(float* data1, float* data2)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte buf[8] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3]};
  Serial.write(buf, 8);
}

void init_mpu(){
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
}







