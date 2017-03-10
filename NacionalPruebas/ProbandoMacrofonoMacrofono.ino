#include <Pixy.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//Definiendo motores
#define FrontMotorLeftS1 6
#define FrontMotorLeftS2 5
#define FrontMotorRightS1 12
#define FrontMotorRightS2 11
#define BackMotorLeftS1 7
#define BackMotorLeftS2 8
#define BackMotorRightS1 9
#define BackMotorRightS2 10
#define RodilloS1       13
#define RodilloS2       12
//Inicialiando el IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
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

Pixy pixy;


void setup(void) {
  pixy.init();
  Serial.begin(9600);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  double dNorte=dGetDirect();
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

    delay(8000);
  
  }

double dGetDirect(){
  sensors_event_t event;
  bno.getEvent(&event);
  double dOrientacionAct=event.orientation.x;
  delay(100);
  if(dOrientacionAct<=179){
    dOrientacionAct=dOrientacionAct+179.9;
  }
  else if(dOrientacionAct>180){
    dOrientacionAct=dOrientacionAct-179.9;
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

void spinBallNor(){
  double dNorti=dGetDirect();
  if(dNorti<180){
    while(dNorti<175){
    dNorti=dGetDirect();
    movPers(0,57,80,-75);  
    }
    moveStay();
  }
  else if(dNorti>180){
    while(dNorti>185){
      dNorti=dGetDirect();
      movPers(57,0,-80,80);
    }
    moveStay();
  }
}

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
   movPers(potIzq,potDer,potIzq,potDer);
  Serial.print(potIzq);
  Serial.print("  , potDer= ");
  Serial.println(potDer);
  delay(1);  
}
void goNorth(){
  potIzq=map(dGetDirect(),360,180,40,80);
  potDer=map(dGetDirect(),0,180,40,80);  
  potIzq=constrain(potIzq,40,80);
  potDer=constrain(potDer,40,80);
  movPers(potIzq,potDer,potIzq,potDer);
  Serial.print(potIzq);
  Serial.print("  , potDer= ");
  Serial.println(potDer);
  delay(1);  
}


void loop(){
  
   info();
   if(blocks==1){
    moveStay();
    Serial.println("Si veo");
    if(area<30000){
      Serial.println("Si esta lejos");
      while(area<30000 && blocks!=0){
        Serial.println("Si quiero ir a la bola");
        info();
        goBall();
      }
    }
    if(area>30000){
        if(dGetDirect()<170 && dGetDirect()>190);
        spinBallNor();
        Serial.println("ya no esta lejos");
        }
        else{
          goNorth();
        }
  }
  else{
    movPers(40,-40,40,-40);
    delay(1);
  }
}

