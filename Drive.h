#ifndef Drive_h
#define Drive_h
#include "Arduino.h"

class Drive{
  public:
    void moveForward();
    void moveBackward();
    void Spin();

  private:
  int iSpeed;

 };
   #endif

