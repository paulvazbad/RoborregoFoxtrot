#include <IMU.h>
#include<Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU I1;
  double dGx, dGy;
}

void loop() {
  dGx= I1.getGyroX();
  dGy= I1.getGyroY();  
  // put your main code here, to run repeatedly:

}
