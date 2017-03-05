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
#define FrontMotorLeftS1 5
#define FrontMotorLeftS2 4
#define FrontMotorRightS1 11
#define FrontMotorRightS2 10
#define BackMotorLeftS1 6
#define BackMotorLeftS2 7
#define BackMotorRightS1 8
#define BackMotorRightS2 9
#define RodilloS1       13
#define RodilloS2       12
//Definiendo TSOPS
#define FrontIr 24 //0
//#define FrontLeftIr 27 //5
//#define FrontRightIr 24 //2
#define LeftIr 31 //3
//#define LeftIrRight 26 //4
//#define LeftIrLeft 23  //1
#define RightIr 30
//#define RightIrRight 29
//#define RightIrLeft 30
#define BackIr 25
//#define BackLeftIr 32
//#define BackRightIr 33
#define BottonHacks 48

//Inicialiando el IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
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
/*  attachInterrupt(digitalPinToInterrupt(qe), INTERmoveBackward, LOW);
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
  //pinMode(FrontLeftIr,INPUT);
  //pinMode(FrontRightIr,INPUT);
  /*pinMode(LeftIrLeft,INPUT);
  pinMode(LeftIrRight,INPUT);
  pinMode(RightIrLeft,INPUT);
  pinMode(RightIrRight,INPUT);
    pinMode(BackLeftIr,INPUT);
  pinMode(BackRightIr,INPUT);
  */
  pinMode(12,OUTPUT);
  digitalWrite(12, HIGH);
  //Boton Hacks xdxd
  pinMode(BottonHacks,INPUT);
  //Inicialiando
    movPers(90,90,85, 90);
    delay(600);
    goNorth();
    movPers(90,90,85, 90);
    delay(600);
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
  delay(500);
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

void moveForward(int pot, double dOrientacionIni){
  moveStay();
  Serial.print("La actual: ");
  Serial.println(dGetDirect());

  if(dGetDirect()>180){
    double dDif = dGetDirect()-dOrientacionIni+2;
    Serial.println("Aumento el lado derecho");
    analogWrite(FrontMotorRightS1,LOW);
    analogWrite(FrontMotorRightS2,pot);
    analogWrite(BackMotorRightS1,LOW);
    analogWrite(BackMotorRightS2,pot);
    analogWrite(FrontMotorLeftS1,100);
    analogWrite(FrontMotorLeftS2,LOW);
    analogWrite(BackMotorLeftS1,100);
    analogWrite(BackMotorLeftS2,LOW);
    //Aumentar lado derecho
  }
  else if(dGetDirect()<180){
  //Aumentar lado izquierda
  Serial.println("Aumento el lado izquierdo");
  analogWrite(FrontMotorLeftS1,pot);
  analogWrite(FrontMotorLeftS2,LOW);
  analogWrite(FrontMotorRightS1,LOW);
  analogWrite(FrontMotorRightS2,100);
  analogWrite(BackMotorLeftS1,pot);
  analogWrite(BackMotorLeftS2,LOW);
  analogWrite(BackMotorRightS1,LOW);
  analogWrite(BackMotorRightS2,0);
  }
  else{
    Serial.println("Me mantengo derecho");
    analogWrite(FrontMotorLeftS1,pot);
    analogWrite(FrontMotorLeftS2,LOW);
    analogWrite(FrontMotorRightS1,LOW);
    analogWrite(FrontMotorRightS2,pot);
    analogWrite(BackMotorLeftS1,pot);
    analogWrite(BackMotorLeftS2,LOW);
    analogWrite(BackMotorRightS1,LOW);
    analogWrite(BackMotorRightS2,pot);
    //Moverse hacia el frente
  }
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
  moveStay();
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
  moveStay();
  double dOrientacionAct=dGetDirect();
  while(dOrientacionAct>180 && dOrientacionAct<175){
      movPers(-30, 30, -30, 30);
      dOrientacionAct = dGetDirect();
  }
}
void turnRight(int dOrientacionGrados){
  Serial.println("Entre a la vuelta derecha");
  double pot=200;
  moveStay();
  double dOrientacionAct=dGetDirect();
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
  double dOrientacionF=dOrientacionAct-dOrientacionGrados;
  if(dOrientacionF>=0){
    while(dOrientacionAct>dOrientacionF){
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
int IrValues(){
  double a[50];
    //El Enfrente
  double dValores=0;
  for(int iX=0; iX<5; iX++){
      dValores+=pulseIn(24,LOW,25000);
      }
  dValores=dValores/5;
  a[0]=dValores;
  //*************************
  //EL Atras
  dValores=0;
  for(int iX=0; iX<5; iX++){
      dValores+=pulseIn(25,LOW,25000);
    }
    dValores=dValores/5;
    a[1]=dValores;
  //El Derecha
  dValores=0;
  for(int iX=0; iX<5; iX++){
      dValores+=pulseIn(30,LOW,25000);
    }
    dValores=dValores/5;
    a[2]=dValores;
    //EL izquierda
    dValores=0;
    for(int iX=0; iX<5; iX++){
        dValores+=pulseIn(31,LOW,25000);
      }
      dValores=dValores/5;
      a[3]=dValores;
      double dMayor=0;
      int iTSOP=0;
      for(int iY=0;iY<4; iY++){
        if(a[iY]>dMayor){
          Serial.print(iY);
          dMayor=a[iY];
          iTSOP=iY;
        }
      }
      if(dMayor<100){
        iTSOP= -1;
      }

    Serial.print("La posicion de la pelota es en: ");
    Serial.println(iTSOP);
    Serial.print("Con la intensidad prom de:");
    Serial.println(dMayor);
//Reiniciar aqui
    return iTSOP;
}
//---------------------------------------------------------------------------------------------------------------
double dGetDirect(){
  sensors_event_t event;
  bno.getEvent(&event);
  double dOrientacionAct=event.orientation.x;
  if(dOrientacionAct<180){
    dOrientacionAct=dOrientacionAct+180;
  }
  else{
    dOrientacionAct=dOrientacionAct-180;
  }
  return dOrientacionAct;
}
void loop(){
  Serial.println(dGetDirect());
  int iBall=-1; //Posición de
  iBall= IrValues();
  Serial.println(iBall);
  /*
           ///// 1 2 3 \\\\\\
           //// 12    4 \\\\\
        /////// 10    6 \\\\\
           ///// 9 8 7 \\\\\
*/
double dgrados=180;
  switch (iBall){
    case 0:
    //La bola se encuentra enfrente
      movPers(50,55,45, 55);
    //  goNorth();
      delay(500);
      break;
    case 1:
    //La bola se encuent atras.
    //1ero la espiral del movimiento hacia la derecha.
      dgrados=dGetDirect();
      if(dgrados>=345 && dgrados<=15){
        movPers(-50,-45,-45,-50);
        delay(800);
        movPers(50,45,45,50);
        delay(400);
      }
      else{
        movPers(-65,65,65,-65);
        delay(500);
        goNorth();
        movPers(-65,-65,-65,-65);
        delay(500);
        movPers(65,-65,-65,65);
        delay(500);
        goNorth();
      }
      break;
    case 2:
    //La bola se encuentra en la derecha.
    dgrados=dGetDirect();
    if(dgrados>=255 && dgrados<=285){
      movPers(50,-45,-45,50);
      delay(800);
      movPers(-50,45,45,-50);
      delay(400);
    }
    else{
      movPers(-50,-50,-50,-50);
      delay(500);
      movPers(50,-50,-50,50);
      delay(500);
    }
      break;
    case 3:
    //La bola se encuentra a la izquierda
    dgrados=dGetDirect();
    if(dgrados>=75 && dgrados<=105){
      movPers(-50,45,45,-50);
      delay(800);
      movPers(50,-45,-45,50);
      delay(400);
    }
    else{
      movPers(-50,-50,-50,-50);
      delay(500);
      movPers(-50,50,50,-50);
      delay(500);
    }
      break;
    case -1:
      movPers(-30,30,-30,30);
      //digitalWrite(12, LOW);
      break;

      //digitalWrite(12, HIGH);



  }
}
