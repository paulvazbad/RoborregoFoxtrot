ifndef Drive_h
#define Drive_h
#include "Arduino.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

class Drive{
  public:
    Drive();
    Drive(int,int,int,int);
    void moveForward();
    void moveBackward();
    void Spin();
    void Begin(int);
  private:
  int iSpeed;
  Adafruit_MotorShield AdaShield = Adafruit_MotorShield();
  Adafruit_DCMotor* IzquierdaDelante;
  Adafruit_DCMotor* DerechaDelante;
  Adafruit_DCMotor* IzquierdaAtras;
  Adafruit_DCMotor* DerechaAtras;

 };
   #endif
