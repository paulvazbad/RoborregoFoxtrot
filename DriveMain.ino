#include <Drive.h>
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Drive Mili(1,2,3,4);
  Mili.Begin();
  // put your setup code here, to run once:

}

void loop() {
  Mili.moveForward();
  // put your main code here, to run repeatedly:

}
