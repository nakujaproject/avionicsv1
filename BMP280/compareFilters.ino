/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

  This software may be distributed and modified under the terms of the GNU
  General Public License version 2 (GPL2) as published by the Free Software
  Foundation and appearing in the file GPL2.TXT included in the packaging of
  this file. Please note that GPL2 Section 2[b] requires that all works based
  on this software must also be made publicly available under the terms of
  the GPL2 ("Copyleft").

  Contact information
  -------------------

  Kristian Lauszus, TKJ Electronics
  Web      :  http://www.tkjelectronics.com
  e-mail   :  kristianl@tkjelectronics.com

  Modified by Brian Palmer aug, 2020 - added Moving Averaging filter and added Kalman filter on Grade.
*/

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <MovingAverageFilter.h>

// Moving Average filters
MovingAverageFilter movingAverageFilter_x(9);        //
MovingAverageFilter movingAverageFilter_y(9);        // Moving average filters for the accelerometers
MovingAverageFilter movingAverageFilter_z(9);
float moveAvgX, moveAvgY, moveAvgZ;

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanAngleX; // Create the Kalman instances
Kalman kalmanAngleY;
Kalman kalmanGradeX;
Kalman kalmanGradeY;

/* IMU Data */
float accX, accY, accZ;
float gyroX, gyroY, gyroZ; //rotational speed in dps (degrees per second).
float tempRaw;
double gyroXangle, gyroYangle; // Calculated angle using the gyro only
double gyroXgrade, gyroYgrade; // Calculated grade using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double compGradeX, compGradeY; // Calculated grade using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double kalGradeX, kalGradeY; // Calculated grade using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void writeTo(byte device, byte toAddress, byte val) 
{
  Wire.beginTransmission(device);
  Wire.write(toAddress);
  Wire.write(val);
  Wire.endTransmission();
}

// fonction lecture I2C librairie "wire.h"
//----------------------------------------

void readFrom(byte device, byte fromAddress, int num, byte result[]) 
{
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  
  // condition de test pour la lecture I2C
  //--------------------------------------

  int i = 0;
  
  while(Wire.available())
  {
    result[i] = Wire.read();
    i++;
  }
}

void getGyroscopeReadings(int Gyro_out[]) 
{
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer); 
  
  Gyro_out[0]=(((int)buffer[0]) << 8 ) | buffer[1];
  Gyro_out[1]=(((int)buffer[2]) << 8 ) | buffer[3];
  Gyro_out[2]=(((int)buffer[4]) << 8 ) | buffer[5];
}

//datasheet lecture accelerometre I2C ADXL345
//-------------------------------------------

void getAccelerometerReadings(int Accel_out[]) 
{
  byte buffer[6];
  readFrom(0x53,0x32,6,buffer); 
  
  Accel_out[0]=(((int)buffer[1]) << 8 ) | buffer[0];
  Accel_out[1]=(((int)buffer[3]) << 8 ) | buffer[2];
  Accel_out[2]=(((int)buffer[5]) << 8 ) | buffer[4];
}

// TODO: Make calibration routine
/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  Wire.begin();
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }  //  nano 33 iot Accelerometer sample rate = 104.00 Hz
  //  Serial.print("Accelerometer sample rate = ");
  //  Serial.print(IMU.accelerationSampleRate());
  //  Serial.println(" Hz");

  delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double rollAngle  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitchAngle = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double rollAngle  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitchAngle = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanAngleX.setAngle(rollAngle); // Set starting angle
  kalmanAngleX.setAngle(pitchAngle);
  gyroXangle = rollAngle;
  gyroYangle = pitchAngle;
  compAngleX = rollAngle;
  compAngleY = pitchAngle;

  timer = micros();
  delay(5000);
}

void loop() {
    /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double rollRiseRun = atan2(accY, accZ);
  double rollGrade = rollRiseRun * 100;
  double rollAngle = rollRiseRun * RAD_TO_DEG;

  double pitchRiseRun = sqrt(accY * accY + accZ * accZ);
  double pitchGrade = pitchRiseRun * 100;
  double pitchAngle = atan(-accX / pitchRiseRun) * RAD_TO_DEG;

#else // Eq. 28 and 29
  double rollRiseRun = sqrt(accX * accX + accZ * accZ);
  double rollGrade = rollRiseRun * 100;
  double rollAngle = atan(accY / rollRiseRun) * RAD_TO_DEG;;

  double pitchRiseRun = atan2(-accX, accZ);
  double pitchGrade = pitchRiseRun * 100;
  double pitchAngle =  pitchRiseRun * RAD_TO_DEG;
#endif

  // 131 corresponds to a rotation rate of 1 degree per second.
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((rollAngle < -90 && kalAngleX > 90) || (rollAngle > 90 && kalAngleX < -90)) {
    kalmanAngleX.setAngle(rollAngle);
    kalmanGradeX.setAngle(rollGrade);
    compAngleX = rollAngle;
    kalAngleX = rollAngle;
    gyroXangle = rollAngle;
    gyroXgrade = rollGrade;
  } else {
    kalAngleX = kalmanAngleX.getAngle(rollAngle, gyroXrate, dt); // Calculate the angle using a Kalman filter
    kalGradeX = kalmanGradeX.getAngle(rollGrade, gyroXrate, dt); // Calculate the grade using a Kalman filter
  }
  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanAngleY.getAngle(pitchAngle, gyroYrate, dt);
  kalGradeY = kalmanGradeY.getAngle(pitchGrade, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitchAngle < -90 && kalAngleY > 90) || (pitchAngle > 90 && kalAngleY < -90)) {
    kalmanAngleY.setAngle(pitchAngle);
    kalmanGradeY.setAngle(pitchGrade);
    compAngleY = pitchAngle;
    kalAngleY = pitchAngle;
    gyroYangle = pitchAngle;
  } else {
    kalAngleY = kalmanAngleY.getAngle(pitchAngle, gyroYrate, dt); // Calculate the angle using a Kalman filter
    kalGradeY = kalmanGradeY.getAngle(pitchGrade, gyroYrate, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleX = kalmanAngleX.getAngle(rollAngle, gyroXrate, dt); // Calculate the angle using a Kalman filter
  kalGradeX = kalmanGradeX.getAngle(rollGrade, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  // Calculate the angle using a Complimentary filter
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * rollAngle;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitchAngle;

  // Calculate the grade using a Complimentary filter
  compGradeX = 0.93 * (compGradeX + gyroXrate * dt) + 0.07 * rollGrade;
  compGradeY = 0.93 * (compGradeY + gyroYrate * dt) + 0.07 * pitchGrade;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  // moving average filter incline

  moveAvgX = movingAverageFilter_x.process(accX);      //
  moveAvgY = movingAverageFilter_y.process(accY);      //   Apply moving average filters to reduce noise
  moveAvgZ = movingAverageFilter_z.process(accZ);      //
  double mvgAvgGradeX = calcGrade(moveAvgX, moveAvgY, moveAvgZ);

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print("accY:"); Serial.print(accY); Serial.print("\t");
  Serial.print("accZ:"); Serial.print(accZ); Serial.print("\t");

  Serial.print("gyroX:"); Serial.print(gyroX); Serial.print("\t");
  Serial.print("gyroY:"); Serial.print(gyroY); Serial.print("\t");
  Serial.print("gyroZ:"); Serial.print(gyroZ); Serial.print("\t");
#endif
  //  Angles
  //  Serial.print("unFiltAngle:"); Serial.print(rollAngle); Serial.print("\t");
  //  Serial.print("gyroXangle:");Serial.print(gyroXangle); Serial.print("\t");
  //  Serial.print("complAngle:"); Serial.print(compAngleX); Serial.print("\t");
  //  Serial.print("kalmanAngle:"); Serial.print(kalAngleX); Serial.print("\t");

  //  Serial.print("pitch:"); Serial.print(pitch); Serial.print("\t");
  //  Serial.print("gyroYangle:");Serial.print(gyroYangle); Serial.print("\t");
  //  Serial.print("compAngleY:"); Serial.print(compAngleY); Serial.print("\t");
  //  Serial.print("kalAngleY:"); Serial.print(kalAngleY); Serial.print("\t");

  // Grades
  Serial.print("unfiltered:"); Serial.print(rollGrade); Serial.print("\t");
  Serial.print("complementaryFilter:"); Serial.print(compGradeX); Serial.print("\t");
  Serial.print("kalmanFilter:"); Serial.print(kalGradeX); Serial.print("\t");
  Serial.print("movingAvg:"); Serial.print(mvgAvgGradeX); //Serial.print("\t");
  
  Serial.print("\r\n");
  delay(2);
}

double calcGrade(float x, float y, float z) {
  double grade = 0;

  // find pitch in radians
  float radpitch = atan2( (y) , sqrt(x * x + z * z) );
  // find the % grade from the pitch
  grade = tan(radpitch) * 100;

  //grade = grade * -1; // flip the sign since its mounted with the USB port on the left.
  return grade;
}