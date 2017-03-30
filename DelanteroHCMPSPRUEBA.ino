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
#define COLORIZQ        3
#define COLORDER        2
#define COLORAMBOS      18
//Definiendo ULTRASONICOS
#define TRIGGERL        15
#define ECHOL           16
#define TRIGGERR        99
#define ECHOR           98
#define push            35
#define VULTRAL         24
#define VULTRAR         25
//
#define CMPS11_ADDRESS 0x60  
#define ANGLE_8  1


//Variables Potencia
double potDer=0;
double potIzq=0;
//Variables "Vision"
int blocks=0;
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
  delay(200);
 // attachInterrupt(digitalPinToInterrupt(COLORAMBOS), moveBack, LOW);
 // attachInterrupt(digitalPinToInterrupt(COLORIZQ) , moveRight, LOW);
 // attachInterrupt(digitalPinToInterrupt(COLORDER),moveLeft , LOW);
 digitalWrite(VULTRAL, HIGH);
 digitalWrite(VULTRAR, HIGH);


  }
double dGetDirect(){    
  Wire.beginTransmission(CMPS11_ADDRESS);  
  Wire.write(ANGLE_8);                    
  Wire.endTransmission();
  Wire.requestFrom(CMPS11_ADDRESS, 1);       
  while(Wire.available() < 1);            
    angle8 = Wire.read();               
    angle16=map(angle8,0,255,0,360);
    Serial.print("    MAPEADO angle 16: ");        
    Serial.println(angle16, DEC);  
    iAngle=angle16; 
    inewAngle=iAngle+iComplemento;
    if(inewAngle>=360){
      inewAngle=inewAngle-360;
    }
    Serial.print("Nuevo angulo=   ");
    Serial.println(inewAngle);
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
  double dNorti=dGetDirect();
  if(dNorti<180){
    while(dNorti<175){
    dNorti=dGetDirect();
    movePers(0,57,80,-75);
    }
  }
   else if(dNorti>180){
     while(dNorti>185){
      dNorti=dGetDirect();
      movePers(57,0,-80,80);
      }
    }
  //  moveStay();
}

/*
void spinBallNor(){
  double dNorti=dGetDirect();
  if(dNorti<180){
    while(dNorti<175){
    dNorti=dGetDirect();
    movePers(0,57,80,-75);
    delay(1);
    }
   // moveFrenos();
   // delay(10);
    moveStay();
  }
  else if(dNorti>180){
    while(dNorti>185){
      dNorti=dGetDirect();
      movePers(57,0,-80,80);
      delay(1);
    }
   // moveFrenos();
   // delay(10);
    moveStay();
  }
}*/

void spinNorth(){
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
}

//ULTRASONICOS
int Distancia(char cUlt){
   int iDuracion=0, iDistan=0;
   if(cUlt=='r'){
     for(int iX=0; iX<10;iX++){
       digitalWrite(TRIGGERR,HIGH);
       delay(10);
       digitalWrite(TRIGGERR, LOW);
       iDuracion = pulseIn(ECHOR,HIGH);
       iDistan += iDuracion * 10 / 292/ 2;
     }
  }
  else if (cUlt=='l'){
    for(int iX=0; iX<10;iX++){
      digitalWrite(TRIGGERL,HIGH);
      delay(10);
      digitalWrite(TRIGGERL, LOW);
      iDuracion = pulseIn(ECHOL,HIGH);
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
  potIzq=map(cordx,0,150,40,80);
  potDer=map(cordx,319,170,40,80);
  potIzq=constrain(potIzq,40,80);
  potDer=constrain(potDer,40,80);
  movePers(potIzq,potDer,potIzq,potDer);
  delay(1);
}

void goNorth(){
  potIzq=map(dGetDirect(),360,180,40,80);
  potDer=map(dGetDirect(),0,180,40,80);
  potIzq=constrain(potIzq,40,80);
  potDer=constrain(potDer,40,80);
  movePers(potIzq,potDer,potIzq,potDer);
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

void loop(){
 Serial.println(dGetDirect());
 delay(1000);

}

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
/*void loop(){
  int iDer, iZQ, iAm;
  iDer = digitalRead(COLORDER);
  iZQ = digitalRead(COLORIZQ);
  iAm = digitalRead(COLORAMBOS);
  Serial.print("Der ");
  Serial.println(iDer);
  Serial.print("Izq ");
  Serial.println(iZQ);
  delay(300);

  movePers(50,50,70,70);
  delay(2000);
  moveStay();

}
/*
//              ALGORITMO ORIGINAL

void loop(){
   info();
   if(blocks==1){
    moveStay();
    Serial.println("Si veo");
    if(area<30000){
      Serial.println("Si esta lejos");
      while(area<30000 && blocks>0){
        Serial.println("Si quiero ir a la bola");
        info();
        goBall();
      }
    }
    if(area>30000){
        Serial.println(dGetDirect());
        if(dGetDirect()<175 || dGetDirect()>190){
          Serial.println("ya no esta lejos");
          Serial.println(dGetDirect());
          Serial.print("El area es: ");
          Serial.println(area);
          Serial.println(altura);
          Serial.println(anchura);
          spinBallNor();
          spinNorth();
          if(dGetDirect()>170 && dGetDirect()<190){
          centrarse();
          }
        }
        else  if(dGetDirect()>170 && dGetDirect()<190){
          while(blocks!=0){
            info();
            centrarse();
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
    if(ultPos==true){
    movePers(-40,40,-40,80);
    delay(1);
    }
    else{
    movePers(40,-40,40,-80);
    delay(1);
    }
  }
}*/
