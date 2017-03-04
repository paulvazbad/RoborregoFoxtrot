
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//Definiendo TCRTS
#define qd 2
#define qi 3
#define qe 18
#define qa 19
float DerD, IzqD, EnfrD, AtrasD;
//Definiendo motores
#define FrontMotorLeftS1 10
#define FrontMotorLeftS2 11
#define FrontMotorRightS1 6
#define FrontMotorRightS2 7
#define BackMotorLeftS1 9
#define BackMotorLeftS2 8
#define BackMotorRightS1 4
#define BackMotorRightS2 5
#define RodilloS1       3
#define RodilloS2       2
//Definiendo TSOPS
#define FrontIr 22
#define FrontLeftIr 23
#define FrontRightIr 24
#define LeftIr 25
#define LeftIrRight 26
#define LeftIrLeft 27
#define RightIr 28
#define RightIrRight 29
#define RightIrLeft 30
#define BackIr 31
#define BackLeftIr 32
#define BackRightIr 33


//Inicialiando el IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Variable de posicion de la pelota global
bool BallFront, BallLeft, BallRight, BallBack;
/*
dProm[0]=Enfrente
dProm[1]=DerechaEnfrente
*/
double dProm[7];

void setup(void) {
  delay(200);
  Serial.begin(9600);
  bno.begin();
  delay(50);
  bno.setExtCrystalUse(true);
  double dNorte=dGetDirect();
  Serial.println("El norte esta aqui amiguito: ");
  Serial.println(dNorte);
  //Qrd's
  pinMode(qd,INPUT);
  pinMode(qi,INPUT);
  pinMode(qa,INPUT);
  pinMode(qe,INPUT);
  /*attachInterrupt(digitalPinToInterrupt(qe), INTERmoveBackward, LOW);
  attachInterrupt(digitalPinToInterrupt(qa), INTERmoveForward, LOW);
  attachInterrupt(digitalPinToInterrupt(qi), INTERmoveRight, LOW);
  attachInterrupt(digitalPinToInterrupt(qd), INTERmoveLeft, LOW);
  */
  //Puentes H
  pinMode(FrontMotorLeftS1,OUTPUT);
  pinMode(FrontMotorLeftS2,OUTPUT);
  pinMode(FrontMotorRightS1,OUTPUT);
  pinMode(FrontMotorRightS2,OUTPUT);

  pinMode(BackMotorLeftS1,OUTPUT);
  pinMode(BackMotorLeftS2,OUTPUT);
  pinMode(BackMotorRightS1,OUTPUT);
  pinMode(BackMotorRightS2,OUTPUT);
  pinMode(RodilloS1, OUTPUT);
  pinMode(RodilloS2, OUTPUT);
  //TSOPS
  pinMode(FrontIr,INPUT);
  pinMode(LeftIr,INPUT);
  pinMode(RightIr,INPUT);
  pinMode(BackIr,INPUT);

  pinMode(FrontLeftIr,INPUT);
  pinMode(FrontRightIr,INPUT);
  pinMode(BackLeftIr,INPUT);
  }

void moveStay(){
  analogWrite(FrontMotorLeftS1,LOW);
  analogWrite(FrontMotorLeftS2,LOW);
  analogWrite(FrontMotorRightS1,LOW);
  analogWrite(FrontMotorRightS2,LOW);
  analogWrite(BackMotorLeftS1,LOW);
  analogWrite(BackMotorLeftS2,LOW);
  analogWrite(BackMotorRightS1,LOW);
  analogWrite(BackMotorRightS2,LOW);
}
//Funciones de interrupcion ---------------------------------------------------------------------------------------------
void INTERmoveForward(){
  Serial.println("Se interrumpio esto");
  int pot=200;
  moveStay();
  analogWrite(FrontMotorLeftS1,pot);
  analogWrite(FrontMotorLeftS2,LOW);
  analogWrite(FrontMotorRightS1,LOW);
  analogWrite(FrontMotorRightS2,pot);
  analogWrite(BackMotorLeftS1,pot);
  analogWrite(BackMotorLeftS2,LOW);
  analogWrite(BackMotorRightS1,LOW);
  analogWrite(BackMotorRightS2,pot);
  delay(500);
}
void INTERmoveRight(){
  Serial.println("Se interrumpio esto");
  int pot=200;
  moveStay();
  analogWrite(FrontMotorLeftS1,pot);
  analogWrite(FrontMotorLeftS2,LOW);
  analogWrite(FrontMotorRightS1,pot);
  analogWrite(FrontMotorRightS2,LOW);
  analogWrite(BackMotorLeftS1,LOW);
  analogWrite(BackMotorLeftS2,pot);
  analogWrite(BackMotorRightS1,LOW);
  analogWrite(BackMotorRightS2,pot);
}
void INTERmoveBackward(){
  Serial.println("Se interrumpio esto");
  moveStay();
  int pot=200;
  analogWrite(FrontMotorLeftS1,LOW);
  analogWrite(FrontMotorLeftS2,pot);
  analogWrite(FrontMotorRightS1,pot);
  analogWrite(FrontMotorRightS2,LOW);

  analogWrite(BackMotorLeftS1,LOW);
  analogWrite(BackMotorLeftS2,pot);
  analogWrite(BackMotorRightS1,pot);
  analogWrite(BackMotorRightS2,LOW);
  delay(500);
}
void INTERmoveLeft(){
  Serial.println("Se interrumpio esto");
  int pot=200;
  moveStay();
  analogWrite(FrontMotorLeftS1,LOW);
  analogWrite(FrontMotorLeftS2,pot);
  analogWrite(FrontMotorRightS1,LOW);
  analogWrite(FrontMotorRightS2,pot);
  analogWrite(BackMotorLeftS1,pot);
  analogWrite(BackMotorLeftS2,LOW);
  analogWrite(BackMotorRightS1,pot);
  analogWrite(BackMotorRightS2,LOW);
  delay(500);
  }

//----------------------------------------------------------------------------------------------------------------------
//Funcion de movimiento ------------------------------------------------------------------------------------------------
void shoot(){
  analogWrite(RodilloS1,HIGH);
  analogWrite(RodilloS2,LOW);
}
void  suck() {
  analogWrite(RodilloS1,LOW);
  analogWrite(RodilloS2,HIGH);
}
void  stopR() {
  analogWrite(RodilloS1,LOW);
  analogWrite(RodilloS2,LOW);
}

void moveForward(int pot, int tiempo){
  moveStay();
  analogWrite(FrontMotorLeftS1,pot);
  analogWrite(FrontMotorLeftS2,LOW);
  analogWrite(FrontMotorRightS1,LOW);
  analogWrite(FrontMotorRightS2,pot);
  analogWrite(BackMotorLeftS1,pot);
  analogWrite(BackMotorLeftS2,LOW);
  analogWrite(BackMotorRightS1,LOW);
  analogWrite(BackMotorRightS2,pot);
  delay(tiempo);
}
void moveRight(int pot, int tiempo){
  moveStay();
  analogWrite(FrontMotorLeftS1,pot);
  analogWrite(FrontMotorLeftS2,LOW);
  analogWrite(FrontMotorRightS1,pot);
  analogWrite(FrontMotorRightS2,LOW);
  analogWrite(BackMotorLeftS1,LOW);
  analogWrite(BackMotorLeftS2,pot);
  analogWrite(BackMotorRightS1,LOW);
  analogWrite(BackMotorRightS2,pot);
  delay(tiempo);
}
void moveLeft(int pot, int tiempo){
  moveStay();
  analogWrite(FrontMotorLeftS1,LOW);
  analogWrite(FrontMotorLeftS2,pot);
  analogWrite(FrontMotorRightS1,LOW);
  analogWrite(FrontMotorRightS2,pot);
  analogWrite(BackMotorLeftS1,pot);
  analogWrite(BackMotorLeftS2,LOW);
  analogWrite(BackMotorRightS1,pot);
  analogWrite(BackMotorRightS2,LOW);
  delay(tiempo);
  }
void moveBackward(int pot, int tiempo){
  moveStay();
  analogWrite(FrontMotorLeftS1,LOW);
  analogWrite(FrontMotorLeftS2,pot);
  analogWrite(FrontMotorRightS1,pot);
  analogWrite(FrontMotorRightS2,LOW);

  analogWrite(BackMotorLeftS1,LOW);
  analogWrite(BackMotorLeftS2,pot);
  analogWrite(BackMotorRightS1,pot);
  analogWrite(BackMotorRightS2,LOW);
  delay(tiempo);
}
void movPers(int p1,int p2, int p3, int p4){
  int fL=p1;
  int fR=p2;
  int bL=p3;
  int bR=p4;
  //FrontLeft--------------------------------
  if(fL>=0){
  fL=constrain(fL,0,100);
  fL=map(fL,0,100,0,255);
  analogWrite(FrontMotorLeftS1,fL);
  analogWrite(FrontMotorLeftS2,LOW);
  }
  else if(fL<0){
  fL= fL*-1;
  fL=constrain(fL,0,100);
  fL=map(fL,0,100,0,255);
  analogWrite(FrontMotorLeftS1,LOW);
  analogWrite(FrontMotorLeftS2,fL);
  }
  //FrontRight--------------------------------
  if(fR>=0){
  fR=constrain(fR,0,100);
  fR=map(fR,0,100,0,255);
  analogWrite(FrontMotorRightS1,LOW);
  analogWrite(FrontMotorRightS2,fR);
  }
  else if(fR<0){
  fR= fR*-1;
  fR=constrain(fR,0,100);
  fR=map(fR,0,100,0,255);
  analogWrite(FrontMotorRightS1,fR);
  analogWrite(FrontMotorRightS2,LOW);
  }
  //BackLeft--------------------------------
  if(bL>=0){
  bL=constrain(bL,0,100);
  bL=map(bL,0,100,0,255);
  analogWrite(BackMotorLeftS1,bL);
  analogWrite(BackMotorLeftS2,LOW);
  }
  else if(bL<0){
  bL= bL*-1;
  bL=constrain(bL,0,100);
  bL=map(bL,0,100,0,255);
  analogWrite(BackMotorLeftS1,LOW);
  analogWrite(BackMotorLeftS2,bL);
  }
  //BackRight--------------------------------
  if(bR>=0){
  bR=constrain(bR,0,100);
  bR=map(bR,0,100,0,255);
  analogWrite(BackMotorRightS1,LOW);
  analogWrite(BackMotorRightS2,bR);
  }
  else if(bR<0){
  bR= bR*-1;
  bR=constrain(bR,0,100);
  bR=map(bR,0,100,0,255);
  analogWrite(BackMotorRightS1,bR);
  analogWrite(BackMotorRightS2,LOW);
  }
}
void goNorth(){
  Serial.println("Me posiciono hacia la porteria");
  double pot=400;
  moveStay();
  double dOrientacionAct=dGetDirect();
  double dOrientacionF=0;
  int iGradosGiroD=360-dOrientacionAct;
  if(dOrientacionAct>=180){
      turnRight(iGradosGiroD);
  }
 else{
    turnLeft(dOrientacionAct);
  }
}
void turnRight(int dOrientacionGrados){
  Serial.println("Entre a la vuelta derecha");
  double pot=200;
  moveStay();
  double dOrientacionAct=dGetDirect();
  double dOrientacionIni=dOrientacionAct;
  double dOrientacionF=dOrientacionAct+dOrientacionGrados;
  if(dOrientacionF<=360){
    while(dOrientacionAct<=dOrientacionF){
      dOrientacionAct=dGetDirect();
      Serial.println("NO El angulo a girar excede de los 360");
      delay(500);
      pot = dOrientacionF-dOrientacionAct+110;
      Serial.print("La potencia es de: ");
      Serial.println(pot);
     digitalWrite(FrontMotorLeftS1,pot);
     digitalWrite(FrontMotorLeftS2,LOW);
     digitalWrite(FrontMotorRightS1,pot);
     digitalWrite(FrontMotorRightS2,LOW);
     digitalWrite(BackMotorLeftS1,pot);
     digitalWrite(BackMotorLeftS2,LOW);
     digitalWrite(BackMotorRightS1,pot);
     digitalWrite(BackMotorRightS2,LOW);

   }
 }
 else{
   double dSobrante=dOrientacionF-360;
   while(dOrientacionAct<=360 && dOrientacionAct!=0 ){
     //Potencia original
     dOrientacionAct=dGetDirect();
     pot=200;
     Serial.print("La potencia es de: ");
     Serial.println(pot);
     Serial.println(dOrientacionAct);
     digitalWrite(FrontMotorLeftS1,pot);
     digitalWrite(FrontMotorLeftS2,LOW);
     digitalWrite(FrontMotorRightS1,pot);
     digitalWrite(FrontMotorRightS2,LOW);
     digitalWrite(BackMotorLeftS1,pot);
     digitalWrite(BackMotorLeftS2,LOW);
     digitalWrite(BackMotorRightS1,pot);
     digitalWrite(BackMotorRightS2,LOW);

    }
   while(dOrientacionAct<=dSobrante){
     dOrientacionAct=dGetDirect();
     pot=  dOrientacionF-dOrientacionAct+110;
     Serial.print("La potencia es de: ");
     Serial.println(pot);
     digitalWrite(FrontMotorLeftS1,pot);
     digitalWrite(FrontMotorLeftS2,LOW);
     digitalWrite(FrontMotorRightS1,pot);
     digitalWrite(FrontMotorRightS2,LOW);
     digitalWrite(BackMotorLeftS1,pot);
     digitalWrite(BackMotorLeftS2,LOW);
     digitalWrite(BackMotorRightS1,pot);
     digitalWrite(BackMotorRightS2,LOW);

    }
  }
}
void turnLeft(int dOrientacionGrados){
  Serial.println("Entre a la vuelta izquierda");
  //Falta revisar este bloque
  double pot=200;
  moveStay();
  double dOrientacionAct=dGetDirect();
  double dOrientacionIni=dOrientacionAct;
  double dOrientacionF=dOrientacionAct-dOrientacionGrados;
  if(dOrientacionF>=0){
    while(dOrientacionAct>dOrientacionF){
       dOrientacionAct=dGetDirect();
       pot = dOrientacionAct-dOrientacionF+100;
       Serial.print("La potencia es de: ");
       Serial.println(pot);
       analogWrite(FrontMotorLeftS1,LOW);
       analogWrite(FrontMotorLeftS2,pot);
       analogWrite(FrontMotorRightS1,LOW);
       analogWrite(FrontMotorRightS2,pot);
       analogWrite(BackMotorLeftS1,LOW);
       analogWrite(BackMotorLeftS2,pot);
       analogWrite(BackMotorRightS1,LOW);
       analogWrite(BackMotorRightS2,pot);
     }
  }
  else{
    double dSobrante=dOrientacionF+360;
    while(dOrientacionAct>=0 && dOrientacionAct!=360){
      //POtencia original
       dOrientacionAct=dGetDirect();
       Serial.print("La potencia es de: ");
       Serial.println(pot);
       analogWrite(FrontMotorLeftS1,LOW);
       analogWrite(FrontMotorLeftS2,pot);
       analogWrite(FrontMotorRightS1,LOW);
       analogWrite(FrontMotorRightS2,pot);
       analogWrite(BackMotorLeftS1,LOW);
       analogWrite(BackMotorLeftS2,pot);
       analogWrite(BackMotorRightS1,LOW);
       analogWrite(BackMotorRightS2,pot);
     }
     while(dOrientacionAct>dSobrante){
       dOrientacionAct=dGetDirect();
       pot = dSobrante;
       Serial.print("La potencia es de: ");
       Serial.println(pot);
       analogWrite(FrontMotorLeftS1,LOW);
       analogWrite(FrontMotorLeftS2,pot);
       analogWrite(FrontMotorRightS1,LOW);
       analogWrite(FrontMotorRightS2,pot);
       analogWrite(BackMotorLeftS1,LOW);
       analogWrite(BackMotorLeftS2,pot);
       analogWrite(BackMotorRightS1,LOW);
       analogWrite(BackMotorRightS2,pot);
     }
   }
  }



  //-----------------------------------------------------------------------------------------------------------------------------------------------
  //TSOPS ------------------------------------------------------------------------------------------------------------------------------------------
void IrValues(){
  bool BallFront, BallLeft, BallRight, BallBack;
  double a[50];
  int iConTsop=0;
  for(int iNumT=22; iNumT<35; iNumT++){
    double dValores=0;
    for(int iX=0; iX<5; iX++){
        dValores+=pulseIn(iNumT,LOW);
      }
      dValores=dValores/5;
      a[iConTsop]=dValores;
      iConTsop++;
  }
  double dMayor=0;
  int iTSOP;
  for(int iY=0;iY<12; iY++){
    if(a[iY]>dMayor && a[iY]<1000){
      dMayor=a[iY];
      iTSOP=iY+1;
    }
  }
  if (dMayor<100){
    return 0;
  }
  Serial.print("La posicion de la pelota es en: ");
  Serial.println(iTSOP);
  Serial.print("Con la intensidad prom de:");
  Serial.println(dMayor);

  //Reiniciar aqui
  int iBall= iTSOP;
  return iBall;
}
//---------------------------------------------------------------------------------------------------------------
double dGetDirect(){
  sensors_event_t event;
  bno.getEvent(&event);
  double dOrientacionAct=event.orientation.x;
  delay(500);
  return dOrientacionAct;


}
void loop(){
  Serial.println(dGetDirect());
  if(Serial.read()=='l'){
    turnLeft(100);
  }
  else if(Serial.read()=='r'){
    turnRight(100);
  }

}
