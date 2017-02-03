#include <IMU.h>
#include<Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU I1;
  double iAngle;
}

void loop() {
 iAngle = I1.getAngle(); 
  // put your main code here, to run repeatedly:

}
