#include "Drive.h"
#include <Wire.h>

Drive Mili(1,2,3,4);
void setup() {
  Serial.begin(9600);
  
  Mili.Begin(200);
  // put your setup code here, to run once:

}

void loop() {
  Mili.moveForward();
  // put your main code here, to run repeatedly:

}
