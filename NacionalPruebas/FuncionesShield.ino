#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AdaShield = Adafruit_MotorShield();
Adafruit_DCMotor *FrontLeft = AdaShield.getMotor(1);
Adafruit_DCMotor *FrontRight = AdaShield.getMotor(3);
Adafruit_DCMotor *BackLeft = AdaShield.getMotor(2);
Adafruit_DCMotor *BackRight = AdaShield.getMotor(4);

void setup(){
  AdaShield.begin();

}
void movPers(int p1,int p2, int p3, int p4){
    int fL=p1;
    int fR=p2;
    int bL=p3;
    int bR=p4;

    //FrontLeft--------------------------------
    fL=map(fL,0,100,0,255);
    FrontLeft->setSpeed(fL);
    if(fL>0){
      FrontLeft->run(FORWARD);
    }
    else{
      FrontLeft->run(BACKWARD);

    }
    //FrontRight--------------------------------
    fR=map(fR,0,100,0,255);
    FrontRight->setSpeed(fR);
    if(fR>0){
      FrontRight->run(FORWARD);
    }
    else{
      FrontRight->run(BACKWARD);

    }
    //BackLeft--------------------------------
    bL=map(bL,0,100,0,255);
    BackLeft->setSpeed(bL);
    if(bL>0){
      BackLeft->run(FORWARD);
    }
    else{
      BackLeft->run(BACKWARD);
    }
    //BackRight--------------------------------
    bR=map(bR,0,100,0,255);
    BackRight->setSpeed(bR);
    if(bR>0){
      BackRight->run(FORWARD);
    }
    else{
      BackRight->run(BACKWARD);

    }
 }
 void moveStay(){
  FrontLeft->run(BRAKE);
  FrontRight->run(BRAKE);
  BackRight->run(BRAKE);
  BackLeft->run(BRAKE);
 }
void loop(){
  movPers(40,40,40,40);
  delay(2000);
}
