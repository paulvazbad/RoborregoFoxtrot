#ifndef IMU_h
#define IMU_h
#include "Arduino.h"

class IMU{
  public:
    double getAngle();
    
  private:
   const int CMPS11_ADDRESS= 0x60;  // I2C address of the MPU-6050
   const int ANGLE_8 = 1;
   int iAngle8 = 0;
 };
   #endif
  
