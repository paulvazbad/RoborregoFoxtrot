#include "Arduino.h"
#include "IMU.h" 
#include <Wire.h>

IMU::IMU(){
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.write(0x1B);                                                    //Send the requested starting register
    //Wire.write(0x08);                                                    //Set the requested starting register
    Wire.endTransmission();    
    
    }
  double IMU:: getGyroX(){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,2,true);  // request a total of 6 registers
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    Wire.endTransmission(true);
    return GyX;    
    }
  double IMU::getGyroY(){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x45);  // starting with register 0x45 (GYRO_YOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,2,true);  // request a total of 2 registers
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    Wire.endTransmission(true);
    return GyY;
    }
  double IMU::getGyroZ(){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x47);  // starting with register 0x47 (GYRO_ZOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,2,true);  // request a total of 2 registers
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    return GyZ;
    }