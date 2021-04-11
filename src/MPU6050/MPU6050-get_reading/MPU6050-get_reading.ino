#include <SimpleKalmanFilter.h>


#include <MPU6050_tockn.h>
#include <Wire.h>


SimpleKalmanFilter simpleKalmanFilter1(2,2,0.01);
SimpleKalmanFilter simpleKalmanFilter2(2,2,0.01);
SimpleKalmanFilter simpleKalmanFilter3(2,2,0.01);
SimpleKalmanFilter simpleKalmanFilter4(2,2,0.01);
SimpleKalmanFilter simpleKalmanFilter5(2,2,0.01);
SimpleKalmanFilter simpleKalmanFilter6(2,2,0.01);

MPU6050 mpu6050(Wire);

long timer = 0;
double gyroX,gyroY, gyroZ,actual_gyroX, actual_gyroY, actual_gyroZ;
double accX, accY, accZ, act_accX, act_accY, act_accZ;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  gyroX = mpu6050.getGyroX();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();

  actual_gyroX = simpleKalmanFilter1.updateEstimate(gyroX);
  actual_gyroY = simpleKalmanFilter2.updateEstimate(gyroY);
  actual_gyroZ = simpleKalmanFilter3.updateEstimate(gyroZ);
  act_accX = simpleKalmanFilter4.updateEstimate(accX);
  act_accY = simpleKalmanFilter5.updateEstimate(accY);
  act_accZ = simpleKalmanFilter6.updateEstimate(accZ);

  if(millis() - timer > 100){
    
    Serial.println("=======================================================");

    Serial.print("accX : ");Serial.print(act_accX);
    Serial.print("\taccY : ");Serial.print(act_accY);
    Serial.print("\taccZ : ");Serial.println(act_accZ);
////  
//    Serial.print("\ngyroX : ");Serial.print(actual_gyroX);
//    Serial.print("\ngyroY : ");Serial.print(actual_gyroY);
//    Serial.print("\ngyroZ : ");Serial.print(actual_gyroZ);
////  
//    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
//    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
//  
//    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
//    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
//    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
//    
//    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
//    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
//    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
//    Serial.println("=======================================================\n");
    timer = millis();
    
  }

}
//double get_kalman_value(double function){
//  double raw_value, kalman_value;
//
//  raw_value = function;
//  kalman_value = simpleKalmanFilter.updateEstimate(raw_value);
//
//  return kalman_value;
//}
