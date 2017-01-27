#ifndef IMU_h
#define IMU_h

#include "Arduino.h"

class IMU{
  public:
    double getGyroX();
    double getGyroY();
    double getGyroZ();
  private:
    const int MPU_ADRR=0x68;  // I2C address of the MPU-6050
   int16_t GyX,GyY,GyZ;
 };
   #endif
  
