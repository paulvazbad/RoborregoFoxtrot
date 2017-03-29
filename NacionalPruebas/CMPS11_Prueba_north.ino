
#include <Wire.h>
#define CMPS11_ADDRESS 0x60  
#define ANGLE_8  1           

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;

void setup()
{
  Serial.begin(9600);  // Start serial port
  Wire.begin();
}

void loop()
{
  delay(2000);
  
  while(true){
    Wire.beginTransmission(CMPS11_ADDRESS);  
    Wire.write(ANGLE_8);                    
    Wire.endTransmission();
   
   
    Wire.requestFrom(CMPS11_ADDRESS, 5);       
    
    while(Wire.available() < 5);        
    
    angle8 = Wire.read();               
    high_byte = Wire.read();
    low_byte = Wire.read();
    angle16 = high_byte;                 
    angle16 <<= 8;
    angle16 += low_byte;
      
    
    Serial.print("    angle full: ");     
    Serial.print(angle16 / 10, DEC);
    Serial.print(".");
    Serial.print(angle16 % 10, DEC);
    
    Serial.print("    angle 8: ");        
    Serial.println(angle8, DEC);
    
    delay(100);
  }                     
}
