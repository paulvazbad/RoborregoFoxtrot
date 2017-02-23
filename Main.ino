#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define qd A0
#define qi A1
#define qe A2
#define qa A3
float DerD, IzqD, EnfrD, AtrasD;

#define FrontMotorLeftS1 4
#define FrontMotorLeftS2 5
#define FrontMotorRightS1 6
#define FrontMotorRightS2 7

#define BackMotorLeftS1 8
#define BackMotorLeftS2 9
#define BackMotorRightS1 10
#define BackMotorRightS2 11
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void setup() {
  Serial.begin(9600);
  //Qrd's
  pinMode(qd,INPUT);
  pinMode(qi,INPUT);
  pinMode(qa,INPUT);
  pinMode(qe,INPUT);

  //Puentes H
  pinMode(FrontMotorLeftS1,OUTPUT);
  pinMode(FrontMotorLeftS2,OUTPUT);
  pinMode(FrontMotorRightS1,OUTPUT);
  pinMode(FrontMotorRightS2,OUTPUT);

  pinMode(BackMotorLeftS1,OUTPUT);
  pinMode(BackMotorLeftS2,OUTPUT);
  pinMode(BackMotorRightS1,OUTPUT);
  pinMode(BackMotorRightS2,OUTPUT);
}

void moveForward(int tiempo){
  digitalWrite(FrontMotorLeftS1,HIGH);
  digitalWrite(FrontMotorLeftS2,LOW);
  digitalWrite(FrontMotorRightS1,LOW);
  digitalWrite(FrontMotorRightS2,HIGH);
  digitalWrite(BackMotorLeftS1,HIGH);
  digitalWrite(BackMotorLeftS2,LOW);
  digitalWrite(BackMotorRightS1,LOW);
  digitalWrite(BackMotorRightS2,HIGH);
  delay(tiempo);
}
void moveRight(int dOrientacionGrados){
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  double dOrientacionAct=event.orientation.x;
  double dOrientacionF=dOrientacionAct+dOrientacionGrados;
  if(dOrientacionF<=360){
    while(dOrientacionAct<=dOrientacionF){
     dOrientacionAct=event.orientation.x;
     digitalWrite(FrontMotorLeftS1,HIGH);
     digitalWrite(FrontMotorLeftS2,LOW);
     digitalWrite(FrontMotorRightS1,HIGH);
     digitalWrite(FrontMotorRightS2,LOW);
     digitalWrite(BackMotorLeftS1,LOW);
     digitalWrite(BackMotorLeftS2,HIGH);
     digitalWrite(BackMotorRightS1,LOW);
     digitalWrite(BackMotorRightS2,HIGH);

   }
 }
 else{
   double dSobrante=dOrientacionF-360;
   while(dOrientacionAct<=dOrientacionF){
     dOrientacionAct=event.orientation.x;
     digitalWrite(FrontMotorLeftS1,LOW);
     digitalWrite(FrontMotorLeftS2,HIGH);
     digitalWrite(FrontMotorRightS1,LOW);
     digitalWrite(FrontMotorRightS2,HIGH);
     digitalWrite(BackMotorLeftS1,HIGH);
     digitalWrite(BackMotorLeftS2,LOW);
     digitalWrite(BackMotorRightS1,HIGH);
     digitalWrite(BackMotorRightS2,LOW);
   }
   while(dOrientacionAct<=dSobrante){
     dOrientacionAct=event.orientation.x;
     digitalWrite(FrontMotorLeftS1,LOW);
     digitalWrite(FrontMotorLeftS2,HIGH);
     digitalWrite(FrontMotorRightS1,LOW);
     digitalWrite(FrontMotorRightS2,HIGH);
     digitalWrite(BackMotorLeftS1,HIGH);
     digitalWrite(BackMotorLeftS2,LOW);
     digitalWrite(BackMotorRightS1,HIGH);
     digitalWrite(BackMotorRightS2,LOW);
   }
}
}
void moveLeft(int dOrientacionGrados){
  //Falta revisar este bloque
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  double dOrientacionAct=event.orientation.x;
  double dOrientacionF=dOrientacionAct-dOrientacionGrados;
  if(dOrientacionF>=0){
    while(dOrientacionAct>dOrientacionF){
       dOrientacionAct=event.orientation.x;
       digitalWrite(FrontMotorLeftS1,LOW);
       digitalWrite(FrontMotorLeftS2,HIGH);
       digitalWrite(FrontMotorRightS1,LOW);
       digitalWrite(FrontMotorRightS2,HIGH);
       digitalWrite(BackMotorLeftS1,HIGH);
       digitalWrite(BackMotorLeftS2,LOW);
       digitalWrite(BackMotorRightS1,HIGH);
       digitalWrite(BackMotorRightS2,LOW);
     }
  }
  else{
    double dSobrante=dOrientacionF+360;
    while(dOrientacionAct>dOrientacionF){
       dOrientacionAct=event.orientation.x;
       digitalWrite(FrontMotorLeftS1,LOW);
       digitalWrite(FrontMotorLeftS2,HIGH);
       digitalWrite(FrontMotorRightS1,LOW);
       digitalWrite(FrontMotorRightS2,HIGH);
       digitalWrite(BackMotorLeftS1,HIGH);
       digitalWrite(BackMotorLeftS2,LOW);
       digitalWrite(BackMotorRightS1,HIGH);
       digitalWrite(BackMotorRightS2,LOW);
     }
     while(dOrientacionAct>dSobrante){
       dOrientacionAct=event.orientation.x;
       digitalWrite(FrontMotorLeftS1,LOW);
       digitalWrite(FrontMotorLeftS2,HIGH);
       digitalWrite(FrontMotorRightS1,LOW);
       digitalWrite(FrontMotorRightS2,HIGH);
       digitalWrite(BackMotorLeftS1,HIGH);
       digitalWrite(BackMotorLeftS2,LOW);
       digitalWrite(BackMotorRightS1,HIGH);
       digitalWrite(BackMotorRightS2,LOW);
     }
  }
  }
void moveBackward(int tiempo){
  digitalWrite(FrontMotorLeftS1,LOW);
  digitalWrite(FrontMotorLeftS2,HIGH);
  digitalWrite(FrontMotorRightS1,HIGH);
  digitalWrite(FrontMotorRightS2,LOW);

  digitalWrite(BackMotorLeftS1,LOW);
  digitalWrite(BackMotorLeftS2,HIGH);
  digitalWrite(BackMotorRightS1,HIGH);
  digitalWrite(BackMotorRightS2,LOW);
  delay(tiempo);
}
void Qrds(){
  //Valores de Luz de los Qrd's
  DerD=analogRead(qd);
  IzqD=analogRead(qi);
  EnfrD=analogRead(qe);
  AtrasD=analogRead(qa);
  Serial.print("El TCRT Enfrente: ");
  Serial.println(EnfrD);

 /* Serial.print("El TCRT Atras: ");
  Serial.println(AtrasD);
  Serial.print("El TCRT Derecho: ");
  Serial.println(DerD);
  Serial.print("El TCRT Izquierdo: ");
  Serial.println(IzqD);
*/}

void Siguelineas(){
  if (DerD == 1)
  {
     moveLeft(1500);

  }
  if (IzqD == 1 )
  {
     moveRight(1500);
  }
  if (AtrasD==1)
  {
    moveForward(1500);
  }
  if(EnfrD==1){
    moveBackward(1500);
  }



}
void loop(){
  moveForward(2000);
Qrds();
delay(500);
}

