#include "Arduino.h"
#include "IMU.h" 
#include <Wire.h>

IMU::IMU(){
    Wire.begin();
    Wire.beginTransmission(CMPS11_ADDRESS);
    Wire.write(ANGLE_8);//Set the requested starting register
    Wire.endTransmission();    
    
    }
  double IMU:: getAngle(){
    Wire.requestFrom(CMPS11_ADDRESS, 1);
    iAngle8 = Wire.read(); 
    Wire.endTransmission();
    return iAngle8;
    
    }
