#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AdaShield = Adafruit_MotorShield();
Adafruit_DCMotor Motor1 = AdaShield.getMotor(1);
Adafruit_DCMotor Motor2 = AdaShield.getMotor(2);
Adafruit_DCMotor Motor3 = AdaShield.getMotor(3);
Adafruit_DCMotor Motor4 = AdaShield.getMotor(4);



void setup() {
  AdaShield.begin();
  Motor1->setSpeed(100);
  Motor2->setSpeed(100);
  Motor3->setSpeed(100);
  Motor4->setSpeed(100);
  // put your setup code here, to run once:

}

void loop() {
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  Motor3->run(FORWARD);
  Motor4->run(FORWARD);
  // put your main code here, to run repeatedly:

}

Drive.moveForward(200)
