#include <Pixy.h>
#include <Wire.h>
#include <SPI.h>

//Definiendo motores
#define FrontMotorLeftS1  10
#define FrontMotorLeftS2  9
#define FrontMotorRightS1 7
#define FrontMotorRightS2 8
#define BackMotorLeftS1  5
#define BackMotorLeftS2  6
#define BackMotorRightS1 12
#define BackMotorRightS2 11
#define RodilloS1       35
#define RodilloS2       36
//Definiendo Colores
#define COLORIZQ        2
#define COLORDER        3
#define COLORAMBOS      18
//Definiendo ULTRASONICOS
#define TRIGGERL        15
#define ECHOL           16
#define TRIGGERR        17
#define ECHOR           19
#define push            35
#define VULTRAL         24
#define VULTRAR         25
//
#define CMPS11_ADDRESS 0x60
#define ANGLE_8  1
#define AREACONST       20000
double dNorti=180;
//Variables Potencia
double potDer=0;
double potIzq=0;
//Variables "Vision"
int blocks=1;
int cordx=0;
int cordy=0;
unsigned long area=0;
int dist=0;
int altura=0;
int anchura=0;
bool ultPos=0;

Pixy pixy;

unsigned char angle8;
unsigned int angle16;
int iAngle=0;
int inewAngle=0;
int iComplemento=0;
int iNorth=0;
double dNorte;
int ultGyr=-1;
int dGetCMPS11(){
 Wire.beginTransmission(CMPS11_ADDRESS);
  Wire.write(ANGLE_8);
  Wire.endTransmission();


  Wire.requestFrom(CMPS11_ADDRESS, 1);

  while(Wire.available() < 1);

  angle8 = Wire.read();
  angle16=map(angle8,0,255,0,360);
  Serial.print(" NORTE   angle 8: ");
  Serial.println(angle8, DEC);
  delay(100);
  iNorth=angle16;
  iComplemento=360-iNorth;
}
void setup(void) {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  dNorte=dGetCMPS11();
  Serial.println("El norte esta aqui amiguito: ");
  Serial.println(dNorte);
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
  pinMode(push, INPUT);
  //ULTRASONICOS
  pinMode(ECHOL, INPUT);
  pinMode(ECHOR, INPUT);
  pinMode(TRIGGERL, OUTPUT);
  pinMode(TRIGGERR, OUTPUT);
  pinMode(VULTRAL,OUTPUT);
  pinMode(VULTRAR ,OUTPUT);
  delay(200);
  attachInterrupt(digitalPinToInterrupt(COLORIZQ) , moveRight, LOW);
 attachInterrupt(digitalPinToInterrupt(COLORDER),moveLeft , LOW);
 digitalWrite(VULTRAL, HIGH);
 digitalWrite(VULTRAR, HIGH);
 pixy.init();
   info();
 movePers(90,90,90,90);
 delay(500);
 moveStay();
  }
double dGetDirect(){
  Wire.beginTransmission(CMPS11_ADDRESS);
  Wire.write(ANGLE_8);
  Wire.endTransmission();
  Wire.requestFrom(CMPS11_ADDRESS, 1);
  while(Wire.available() < 1);
    angle8 = Wire.read();
    angle16=map(angle8,0,255,0,360);
    iAngle=angle16;
    inewAngle=iAngle+iComplemento;
   if(inewAngle>=360){
      inewAngle=inewAngle-360;
    }
  double dOrientacionAct=inewAngle;
  delay(100);
  if(dOrientacionAct<180){
    dOrientacionAct=dOrientacionAct+180;
  }
  else if(dOrientacionAct>=180){
    dOrientacionAct=dOrientacionAct-180;
  }
  return dOrientacionAct;
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

 void moveFrenos(){
  digitalWrite(FrontMotorLeftS1,HIGH);
  digitalWrite(FrontMotorLeftS2,HIGH);
  digitalWrite(FrontMotorRightS1,HIGH);
  digitalWrite(FrontMotorRightS2,HIGH);
  digitalWrite(BackMotorLeftS1,HIGH);
  digitalWrite(BackMotorLeftS2,HIGH);
  digitalWrite(BackMotorRightS1,HIGH);
  digitalWrite(BackMotorRightS2,HIGH);
 }

void movePers(int p1,int p2, int p3, int p4){
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
//                       SPIN ORIGINAL
void spinBallNor(){
  dNorti=dGetDirect();
  if(dNorti<180){
    while(dNorti<176 && area>AREACONST){
      dNorti=dGetDirect();
    int iPot;
    //iPot=map(dGetDirect(),90,180,40,15);
    movePers(0,30,50,-50);
    }
  }
  else if(dNorti>180){
     while(dNorti>184 && area>AREACONST){
       dNorti=dGetDirect();
       int iPot;
       //iPot=map(dGetDirect(),270,180,40,15);
       //iPot= constrain(iPot, 15, 40);
       //movePers(0,0,-iPot,iPot);
       movePers(30,0,-50,50);
        }
      }
      moveStay();
    }

    void spinNorth(){
      Serial.println("Estoy Spineando North");
      double dNorti=dGetDirect();
      if(dNorti<170){
        while(dNorti<175){
          dNorti=dGetDirect();
          Serial.println(dNorti);
          movePers(20,-20,20,-20);
          delay(1);
          }
          //moveFrenos();
          //movePers(80,80,80,80);
          //delay(100);
          moveStay();
      }
      else if(dNorti>190){
        while(dNorti>185){
          dNorti=dGetDirect();
          Serial.println(dNorti);
          movePers(-20,20,-20,20);
          delay(1);
        }
        //moveFrenos();
        //movePers(80,80,80,80);
        //delay(100);
        moveStay();
      }
    moveStay();
    }

//ULTRASONICOS
int Distancia(char cUlt){
   int iDuracion=0, iDistan=0;
   if(cUlt=='r'){
     for(int iX=0; iX<10;iX++){
       digitalWrite(TRIGGERR,HIGH);
       delay(10);
       digitalWrite(TRIGGERR, LOW);
       iDuracion = pulseIn(ECHOR,HIGH,100000);
       iDistan += iDuracion * 10 / 292/ 2;
     }
  }
  else if (cUlt=='l'){
    for(int iX=0; iX<10;iX++){
      digitalWrite(TRIGGERL,HIGH);
      delay(10);
      digitalWrite(TRIGGERL, LOW);
      iDuracion = pulseIn(ECHOL,HIGH,100000);
      iDistan += iDuracion * 10 / 292/ 2;
    }
  }
  iDistan = iDistan/10;
  return iDistan;
}
void centrarse(){
   if((dGetDirect>=170 && dGetDirect<=190) && (digitalRead(push)==1)){
     int distanciaDer, distanciaIzq;
     distanciaDer = Distancia('r');
     distanciaIzq = Distancia('l');
     if(distanciaDer+distanciaIzq>10 && distanciaDer+distanciaIzq<300){
       if(distanciaDer<70){
         movePers(-80,80,80,-80);
         delay(1);
       }
       else if(distanciaIzq<70){
         movePers(80,-80,-80,80);
         delay(1);
       }
     }
     else if((distanciaDer<30 || distanciaIzq<30)&&(distanciaDer>0 && distanciaIzq>0)){
     int ix=0;
       while(ix<800){
         movePers(85,85,85,85);
         delay(1);
         ix++;
      }
    }
  }
}
//

void info() {
  blocks= pixy.getBlocks();
  cordx= pixy.blocks[0].x;  //0-319
  cordy= pixy.blocks[0].y;  //0-199
  altura= pixy.blocks[0].height;
  anchura=pixy.blocks[0].width;
  area= (altura)*(anchura);
  dist=map(area,81,63481,182,1);
}
void checar(){
  static int i = 0;
  int j;
  uint16_t blocks;
  blocks = pixy.getBlocks();
  if (blocks==1)
  {
    i++;
    if (i==50)
    {
      info();
      Serial.print(blocks);
      Serial.print("          ");
      Serial.print(cordx);
      Serial.print("     ");
      Serial.print(cordy);
      Serial.print("     ");
      Serial.print(altura);
      Serial.print("     ");
      Serial.print(anchura);
      Serial.print("     ");
      Serial.print(area);
      Serial.print("     ");
      Serial.println(dist);
      i=0;
    }
  }
}

void goBall(){
  if(dGetDirect()>100 && dGetDirect()<250){
    potDer=map(cordx,0,150,40,80);
    potIzq=map(cordx,319,170,40,80);
    potDer=constrain(potIzq,40,80);
    potIzq=constrain(potDer,40,80);
    movePers(potIzq,potDer+10,potIzq,potDer+10);
    delay(1);
  }
  else{
    if(area>AREACONST){
      potDer=20;
      potIzq=20;
      movePers(potIzq,potDer+5,potIzq,potDer=5);
      delay(1);
    }
    else{
      potDer=map(cordx,0,150,20,40);
      potIzq=map(cordx,319,170,20,40);
      potDer=constrain(potIzq,20,40);
      potIzq=constrain(potDer,20,40);
      movePers(potIzq,(potDer+10),potIzq,(potDer+10));
      delay(1);
    }
  }
}

void goNorth(){
  potIzq=map(dGetDirect(),360,180,40,80);
  potDer=map(dGetDirect(),0,180,40,80);
  potIzq=constrain(potIzq,40,80);
  potDer=constrain(potDer,40,80);
  movePers(potIzq,(potDer+10),potIzq,potDer);
  delay(1);
}

void moveRight(){
  Serial.println("Interrupcion detectada del lado izquierdo ");
  moveStay();
  movePers(50, -50, -50, 50);
  for(int x=0; x<35; x++){
    delayMicroseconds(14000);
  }
  moveStay();
}
void moveLeft(){
  Serial.println("Interrupcion detectada del lado derecho ");
  moveStay();
  movePers(-50,50,50,-50);
  for(int x=0; x<35; x++){
    delayMicroseconds(14000);
  }
  moveStay();
}
void moveBack(){
  Serial.println("Interrupcion detectada de ambos lados ");
  moveStay();
  movePers(-40, -40, -40, -40);
  for(int x=0; x<35; x++){
    delayMicroseconds(15000);
  }
  moveStay();
}
//************************************************************************************************************************************************************
/*void loop(){
  info();
  Serial.println(area);
  delay(200);
}
*/
/*void loop(){
  int iDisR, iDisL;
  Serial.print("Angulo: ");
  Serial.println(dGetDirect());
  iDisR = Distancia('r');
  Serial.print("Distancia Derecha : ");
  Serial.println(iDisR);
  Serial.println("Distancia Izquierda: ");
  iDisL = Distancia('l');
  Serial.println(iDisL);
  spinBallNor();
}
*/
/*
void loop(){
  //movePers(50,50,50,50);
  //moveStay();
  //delay(2500);
  //spinBallNor();
  //spinNorth();
  Serial.println("Dame tiempo");
  delay(1000);
  Serial.println("Calibra Ya");
   delay(2000);
  Serial.println("Corre");
   delay(2000);
  Serial.println("Ya casi ");
   delay(2000);
  Serial.println("Ya!");
   delay(2000);
  bno.begin();
    delay(1000);
  bno.setExtCrystalUse(true);
  while(true){
  Serial.println(dGetDirect());
  displayCalStatus();
  }
}
*/
/*
void loop(){
  int iDer, iZQ, iAm;
  iDer = digitalRead(COLORDER);
  iZQ = digitalRead(COLORIZQ);
  iAm = digitalRead(COLORAMBOS);
  Serial.print("Der ");
  Serial.println(iDer);
  Serial.print("Izq ");
  Serial.println(iZQ);
  delay(300);


}
*/

//              ALGORITMO ORIGINAL

void loop(){
   info();
   if(blocks==1){
     moveStay();
    Serial.println("Si veo");
    info();
    if(area<AREACONST){
      Serial.println("Si esta lejos");
      while(area<AREACONST && blocks>0){
        Serial.println("Si quiero ir a la bola");
        info();
        goBall();
      }
    }
    if(area>AREACONST){
      /*if(ultGyr==1){
         movePers(-30,30,-30,30);
         delay(80);
       }
       else if (ultGyr==0){
          movePers(30,-30,30,-30);
          delay(80);
       }
       */
        Serial.println(dGetDirect());
         dNorti=dGetDirect();
        if(dNorti<160 || dNorti>190){
          Serial.println("ya no esta lejos");
          spinBallNor();
          spinNorth();
        //  spinNorth();
          //if(dGetDirect()>170 && dGetDirect()<190){
          //centrarse();
          //}
        }
        else if(dGetDirect()>=160 && dGetDirect()<=190){
          while(blocks!=0){
            info();
            //centrarse();
            goNorth();
            Serial.println("VOY AL NORTEE  ");
          }
        }
      }
     if(cordx<159){
      ultPos=true;  //Izq
     }
     else{
      ultPos=false;
     }

  }
  else{
    Serial.println("No veo la pelota pero la voy a buscar");
    if(ultPos==true){
      movePers(30,-30,30,-30);
      delay(1);
      ultGyr=1;
    }
    else{
      movePers(-30,30,-30,30);
      delay(1);
      ultGyr=0;
    }
  }
}
