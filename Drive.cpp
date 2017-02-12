#include "Arduino.h"
#include "Drive.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Drive::Drive(){
    
    Adafruit_DCMotor* Motor1 = AdaShield.getMotor(1);
    Adafruit_DCMotor* Motor2 = AdaShield.getMotor(2);
    Adafruit_DCMotor* Motor3 = AdaShield.getMotor(3);
    Adafruit_DCMotor* Motor4 = AdaShield.getMotor(4);
    }
Drive :: Drive(int iPort1, int iPort2 , int iPort3, int iPort4){
    AdaShield = Adafruit_MotorShield();
    IzquierdaDelante = AdaShield.getMotor(iPort1);
    DerechaDelante = AdaShield.getMotor(iPort2);
    IzquierdaAtras = AdaShield.getMotor(iPort3);
    DerechaAtras = AdaShield.getMotor(iPort4);


}
void Drive:: Begin(int iDefaultSpeed){
    AdaShield.begin();
    IzquierdaDelante->setSpeed(iDefaultSpeed);
    DerechaDelante->setSpeed(iDefaultSpeed);
    IzquierdaAtras->setSpeed(iDefaultSpeed);
    DerechaAtras->setSpeed(iDefaultSpeed);

}
void Drive:: moveForward(){
    IzquierdaDelante->run(FORWARD);
    DerechaDelante->run(FORWARD);
    IzquierdaAtras->run(FORWARD);
    DerechaAtras->run(FORWARD);
    //Velocidades para que avance hacia delante

    }
void Drive:: moveBackward(){
  //Velocidades para que retrocede
}
void Drive:: Spin(){
//Velocidades para que gire a lo loco.

}
