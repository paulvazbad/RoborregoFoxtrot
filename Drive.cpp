#include "Arduino.h"
#include "Drive.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Drive::Drive(){
    Adafruit_MotorShield AdaShield = Adafruit_MotorShield();
    Adafruit_DCMotor Motor1 = AdaShield.getMotor(1);
    Adafruit_DCMotor Motor2 = AdaShield.getMotor(2);
    Adafruit_DCMotor Motor3 = AdaShield.getMotor(3);
    Adafruit_DCMotor Motor4 = AdaShield.getMotor(4);
    }
Drive :: Drive(int iPort1, int iPort2 , int iPort3, int iPort4){
    Adafruit_MotorShield AdaShield = Adafruit_MotorShield();
    Adafruit_DCMotor IzquierdaDelante = AdaShield.getMotor(iPort1);
    Adafruit_DCMotor DerechaDelante = AdaShield.getMotor(iPort2);
    Adafruit_DCMotor IzquierdaAtras = AdaShield.getMotor(iPort3);
    Adafruit_DCMotor DerechaAtras = AdaShield.getMotor(iPort4);


}
Drive:: Begin(int iDefaultSpeed){
    AdaShield.begin();
    IzquierdaDelante->setSpeed(iDefaultSpeed);
    DerechaDelante->setSpeed(iDefaultSpeed);
    IzquierdaAtras->setSpeed(iDefaultSpeed);
    DerechaAtras->setSpeed(iDefaultSpeed);

}
void Drive:: moveForward(){
      //Velocidades para que avance hacia delante

    }
void Drive:: moveBackward(){
}
void Drive:: Spin(){


}

