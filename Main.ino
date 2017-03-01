
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
#define LeftIr 23
#define RightIr 24
#define BackIr 25

#define FrontLeftIr 26
#define FrontRightIr 27
#define BackLeftIr 28
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
  Serial.begin(9600);
  bno.begin();
  delay(1000);
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
  digitalWrite(FrontMotorLeftS1,LOW);
  digitalWrite(FrontMotorLeftS2,LOW);
  digitalWrite(FrontMotorRightS1,LOW);
  digitalWrite(FrontMotorRightS2,LOW);
  digitalWrite(BackMotorLeftS1,LOW);
  digitalWrite(BackMotorLeftS2,LOW);
  digitalWrite(BackMotorRightS1,LOW);
  digitalWrite(BackMotorRightS2,LOW);
}
//Funciones de interrupcion ---------------------------------------------------------------------------------------------
void INTERmoveForward(){
  Serial.println("Se interrumpio esto");
  int pot=200;
  moveStay();
  digitalWrite(FrontMotorLeftS1,pot);
  digitalWrite(FrontMotorLeftS2,LOW);
  digitalWrite(FrontMotorRightS1,LOW);
  digitalWrite(FrontMotorRightS2,pot);
  digitalWrite(BackMotorLeftS1,pot);
  digitalWrite(BackMotorLeftS2,LOW);
  digitalWrite(BackMotorRightS1,LOW);
  digitalWrite(BackMotorRightS2,pot);
  delay(500);
}
void INTERmoveRight(){
  Serial.println("Se interrumpio esto");
  int pot=200;
  moveStay();
  digitalWrite(FrontMotorLeftS1,pot);
  digitalWrite(FrontMotorLeftS2,LOW);
  digitalWrite(FrontMotorRightS1,pot);
  digitalWrite(FrontMotorRightS2,LOW);
  digitalWrite(BackMotorLeftS1,LOW);
  digitalWrite(BackMotorLeftS2,pot);
  digitalWrite(BackMotorRightS1,LOW);
  digitalWrite(BackMotorRightS2,pot);
}
void INTERmoveBackward(){
  Serial.println("Se interrumpio esto");
  moveStay();
  int pot=200;
  digitalWrite(FrontMotorLeftS1,LOW);
  digitalWrite(FrontMotorLeftS2,pot);
  digitalWrite(FrontMotorRightS1,pot);
  digitalWrite(FrontMotorRightS2,LOW);

  digitalWrite(BackMotorLeftS1,LOW);
  digitalWrite(BackMotorLeftS2,pot);
  digitalWrite(BackMotorRightS1,pot);
  digitalWrite(BackMotorRightS2,LOW);
  delay(500);
}
void INTERmoveLeft(){
  Serial.println("Se interrumpio esto");
  int pot=200;
  moveStay();
  digitalWrite(FrontMotorLeftS1,LOW);
  digitalWrite(FrontMotorLeftS2,pot);
  digitalWrite(FrontMotorRightS1,LOW);
  digitalWrite(FrontMotorRightS2,pot);
  digitalWrite(BackMotorLeftS1,pot);
  digitalWrite(BackMotorLeftS2,LOW);
  digitalWrite(BackMotorRightS1,pot);
  digitalWrite(BackMotorRightS2,LOW);
  delay(500);
  }

//----------------------------------------------------------------------------------------------------------------------
//Funcion de movimiento ------------------------------------------------------------------------------------------------
void shoot(){
  digitalWrite(RodilloS1,HIGH);
  digitalWrite(RodilloS2,LOW);
}
void  suck() {
  digitalWrite(RodilloS1,LOW);
  digitalWrite(RodilloS2,HIGH);
}
void  stopR() {
  analogWrite(RodilloS1,LOW);
  analogWrite(RodilloS2,LOW);
}

void moveForward(int pot, int tiempo){
  moveStay();
  digitalWrite(FrontMotorLeftS1,pot);
  digitalWrite(FrontMotorLeftS2,LOW);
  digitalWrite(FrontMotorRightS1,LOW);
  digitalWrite(FrontMotorRightS2,pot);
  digitalWrite(BackMotorLeftS1,pot);
  digitalWrite(BackMotorLeftS2,LOW);
  digitalWrite(BackMotorRightS1,LOW);
  digitalWrite(BackMotorRightS2,pot);
  delay(tiempo);
}
void moveRight(int pot, int tiempo){
  moveStay();
  digitalWrite(FrontMotorLeftS1,pot);
  digitalWrite(FrontMotorLeftS2,LOW);
  digitalWrite(FrontMotorRightS1,pot);
  digitalWrite(FrontMotorRightS2,LOW);
  digitalWrite(BackMotorLeftS1,LOW);
  digitalWrite(BackMotorLeftS2,pot);
  digitalWrite(BackMotorRightS1,LOW);
  digitalWrite(BackMotorRightS2,pot);
  delay(tiempo);
}
void moveLeft(int pot, int tiempo){
  moveStay();
  digitalWrite(FrontMotorLeftS1,LOW);
  digitalWrite(FrontMotorLeftS2,pot);
  digitalWrite(FrontMotorRightS1,LOW);
  digitalWrite(FrontMotorRightS2,pot);
  digitalWrite(BackMotorLeftS1,pot);
  digitalWrite(BackMotorLeftS2,LOW);
  digitalWrite(BackMotorRightS1,pot);
  digitalWrite(BackMotorRightS2,LOW);
  delay(tiempo);
  }
void moveBackward(int pot, int tiempo){
  moveStay();
  digitalWrite(FrontMotorLeftS1,LOW);
  digitalWrite(FrontMotorLeftS2,pot);
  digitalWrite(FrontMotorRightS1,pot);
  digitalWrite(FrontMotorRightS2,LOW);

  digitalWrite(BackMotorLeftS1,LOW);
  digitalWrite(BackMotorLeftS2,pot);
  digitalWrite(BackMotorRightS1,pot);
  digitalWrite(BackMotorRightS2,LOW);
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
void turnRight(int dOrientacionGrados){
  int pot=200;
  moveStay();
  double dOrientacionAct=dGetDirect();
  double dOrientacionIni=dOrientacionAct;
  double dOrientacionF=dOrientacionAct+dOrientacionGrados;
  if(dOrientacionF<=360){
    while(dOrientacionAct<=dOrientacionF){
      Serial.println("El angulo a girar excede de los 360");
      delay(500);
     int pot = map(dOrientacionAct,dOrientacionF,dOrientacionIni,100,200);
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
     pot = map(dOrientacionAct,dOrientacionIni,dOrientacionF,200,100);
     dOrientacionAct=dGetDirect();
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
    pot= map(dOrientacionAct,0,dSobrante,pot,100);
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
  //Falta revisar este bloque
  int pot=200;
  moveStay();
  double dOrientacionAct=dGetDirect();
  double dOrientacionIni=dOrientacionAct;
  double dOrientacionF=dOrientacionAct-dOrientacionGrados;
  if(dOrientacionF>=0){
    while(dOrientacionAct>dOrientacionF){
       dOrientacionAct=dGetDirect();
       int pot = map(dOrientacionAct,dOrientacionIni,dOrientacionF,200,100);
       digitalWrite(FrontMotorLeftS1,LOW);
       digitalWrite(FrontMotorLeftS2,pot);
       digitalWrite(FrontMotorRightS1,LOW);
       digitalWrite(FrontMotorRightS2,pot);
       digitalWrite(BackMotorLeftS1,LOW);
       digitalWrite(BackMotorLeftS2,pot);
       digitalWrite(BackMotorRightS1,LOW);
       digitalWrite(BackMotorRightS2,pot);
     }
  }
  else{
    double dSobrante=dOrientacionF+360;
    while(dOrientacionAct>=0 && dOrientacionAct!=360){
       pot = map(dOrientacionAct,dOrientacionIni,dOrientacionF,200,100);
       dOrientacionAct=dGetDirect();
       digitalWrite(FrontMotorLeftS1,LOW);
       digitalWrite(FrontMotorLeftS2,pot);
       digitalWrite(FrontMotorRightS1,LOW);
       digitalWrite(FrontMotorRightS2,pot);
       digitalWrite(BackMotorLeftS1,LOW);
       digitalWrite(BackMotorLeftS2,pot);
       digitalWrite(BackMotorRightS1,LOW);
       digitalWrite(BackMotorRightS2,pot);
     }
     while(dOrientacionAct>dSobrante){

       dOrientacionAct=dGetDirect();
       pot= map(dOrientacionAct,360,dSobrante,pot,100);
       pot = map(dOrientacionAct,dOrientacionIni,dOrientacionF,200,100);
       digitalWrite(FrontMotorLeftS1,LOW);
       digitalWrite(FrontMotorLeftS2,pot);
       digitalWrite(FrontMotorRightS1,LOW);
       digitalWrite(FrontMotorRightS2,pot);
       digitalWrite(BackMotorLeftS1,LOW);
       digitalWrite(BackMotorLeftS2,pot);
       digitalWrite(BackMotorRightS1,LOW);
       digitalWrite(BackMotorRightS2,pot);
     }
   }
  }

  
  
  //-----------------------------------------------------------------------------------------------------------------------------------------------
  //TSOPS ------------------------------------------------------------------------------------------------------------------------------------------
void prom(double a[][29], int largo) {
  for(int iNumT=22; iNumT<29; iNumT++){
    for(int i=0; i<(largo-1); i++) {
        for(int o=0; o<(largo-(i+1)); o++) {
                if(a[iNumT][o] > a[iNumT][o+1]) {
                    double t = a[iNumT][o];
                    a[iNumT][o] = a[iNumT][o+1];
                    a[iNumT][o+1] = t;
                }
        }
    }

   double dPromedio=(a[iNumT][1]+a[iNumT][2]+a[iNumT][3]+a[iNumT][4]+a[iNumT][5])/5;
   dProm[iNumT]=dPromedio;
 }
}
void IrValues(){
  //Col 0 = Lecturas 1
  //Renglon 0 TSOP 1
  bool BallFront, BallLeft, BallRight, BallBack;
  double MatLec[29][29];

  for(int iNumT=22; iNumT<29; iNumT++){
    for(int iLect=0; iLect<7; iLect++){
        MatLec[iNumT][iLect]=pulseIn(iNumT,LOW);
    }
  }
  prom(MatLec,7);
}

void IrBalls(){
  if(dProm[0]>=280 && dProm[0]<=350){
    BallFront=true;
  }
  else {
    BallFront=false;
  }
  if(dProm[1]>=280 && dProm[1]<=350){
    BallLeft=true;
  }
  else{
    BallLeft=false;
  }
  if(dProm[2]>=280 && dProm[2]<=350){
    BallRight=true;
  }
  else{
    BallRight=false;
  }
  if(dProm[3]>=280 && dProm[3]<=350){
    BallBack=true;
  }
  else{
    BallBack=false;
  }

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
  moveForward(200, 3000);
  /*IrValues();
  IrBalls();
  if(BallFront){
    moveForward(150,100);
    suck();
    }
    shoot();
  else{
    moveBackward(100, 100);
    }
    */
}
