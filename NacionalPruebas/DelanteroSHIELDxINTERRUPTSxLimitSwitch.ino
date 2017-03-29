#include <Pixy.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
//Paul Vazquez
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//Inicializando el shield
Adafruit_MotorShield AdaShield = Adafruit_MotorShield();
//Definiendo motores
Adafruit_DCMotor *FrontRight = AdaShield.getMotor(1);
Adafruit_DCMotor *FrontLeft = AdaShield.getMotor(3);
Adafruit_DCMotor *BackLeft = AdaShield.getMotor(4);
Adafruit_DCMotor *BackRight = AdaShield.getMotor(2);
#define RodilloS1       13
#define RodilloS2       12
#define TRIGGER1         15
#define ECHO1            16
#define TRIGGER2         17
#define ECHO2            18
#define colorLeft         2
#define colorRight       18
#define CONSTRIGHT      48
#define CONSTLEFT       55
#define push            35
//Inicialiando el IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Inicializando el shield
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

double dGetDirect(){
  sensors_event_t event;
  bno.getEvent(&event);
  double dOrientacionAct=event.orientation.x;
  delay(100);
  if(dOrientacionAct<180){
    dOrientacionAct=dOrientacionAct+180;
  }
  else if(dOrientacionAct>=180){
    dOrientacionAct=dOrientacionAct-180;
  }
return dOrientacionAct;
}


void setup() {
  pixy.init();
  Serial.begin(9600);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  double dNorte=dGetDirect();
  Serial.println("El norte esta aqui amiguito: ");
  Serial.println(dNorte);
  AdaShield.begin();
  pinMode(RodilloS1, OUTPUT);
  pinMode(RodilloS2, OUTPUT);
  pinMode(colorLeft,INPUT);
  pinMode(colorRight,INPUT);
  pinMode(push,INPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIGGER1, OUTPUT);
  pinMode(TRIGGER2, OUTPUT);
  delay(1000);

  }

  void checkColor(){
    if(digitalRead(colorLeft)==1 && digitalRead(colorRight)==1){
      movPers(-80,-80,-80,-80);
      delay(800);
    }
    if(digitalRead(colorLeft)==1){
      movPers(80,-80,-80,80);
      delay(800);
    }
    if(digitalRead(colorRight)==1){
      movPers(-80,80,80,-80);
      delay(800);
    }
  }



void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

void moveStay(){
  FrontLeft->run(RELEASE);
  FrontRight->run(RELEASE);
  BackRight->run(RELEASE);
  BackLeft->run(RELEASE);
  delay(100);
 }


void movPers(int p1,int p2, int p3, int p4){
    int fL=p1;
    int fR=p2;
    int bL=p3;
    int bR=p4;

    //FrontLeft--------------------------------
    fL=map(fL,0,100,0,255);
    FrontLeft->setSpeed(fL);
    if(fL>0){
      FrontLeft->run(FORWARD);
    }
    else{
      FrontLeft->run(BACKWARD);

    }
    //FrontRight--------------------------------
    fR=map(fR,0,100,0,255);
    FrontRight->setSpeed(fR);
    if(fR>0){
      FrontRight->run(FORWARD);
    }
    else{
      FrontRight->run(BACKWARD);

    }
    //BackLeft--------------------------------
    bL=map(bL,0,100,0,255);
    BackLeft->setSpeed(bL);
    if(bL>0){
      BackLeft->run(FORWARD);
    }
    else{
      BackLeft->run(BACKWARD);
    }
    //BackRight--------------------------------
    bR=map(bR,0,100,0,255);
    BackRight->setSpeed(bR);
    if(bR>0){
      BackRight->run(FORWARD);
    }
    else{
      BackRight->run(BACKWARD);

    }
 }
//                       SPIN ORIGINAL
void spinBallNor(){
  double dNorti=dGetDirect();
  if(dNorti<180){
    while(dNorti<175){
    dNorti=dGetDirect();
    movPers(0,57,80,-75);
    checkColor();
    }
  }
   else if(dNorti>180){
     while(dNorti>185){
     dNorti=dGetDirect();
     movPers(57,0,-80,80);
     checkColor();
     }
    }
    moveStay();
}

/*
void spinBallNor(){
  double dNorti=dGetDirect();
  if(dNorti<180){
    while(dNorti<175){
    dNorti=dGetDirect();
    movPers(0,57,80,-75);
    delay(1);
    }
   // moveFrenos();
   // delay(10);
    moveStay();
  }
  else if(dNorti>180){
    while(dNorti>185){
      dNorti=dGetDirect();
      movPers(57,0,-80,80);
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
    movPers(20,-20,20,-20);
    delay(1);
    checkColor();
    }
    //moveFrenos();
    //movPers(80,80,80,80);
    //delay(100);
    moveStay();
  }
  else if(dNorti>190){
    while(dNorti>185){
      dNorti=dGetDirect();
      Serial.println(dNorti);
      movPers(-20,20,-20,20);
      delay(1);
      checkColor();
    }
    //moveFrenos();
    //movPers(80,80,80,80);
    //delay(100);
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
  delay(1);
}

void goNorth(){
  potIzq=map(dGetDirect(),360,180,40,80);
  potDer=map(dGetDirect(),0,180,40,80);
  potIzq=constrain(potIzq,40,80);
  potDer=constrain(potDer,40,80);
  movPers(potIzq,potDer,potIzq,potDer);
  delay(1);
}

/*
void loop(){


  Serial.println(dGetDirect());
  displayCalStatus();


}
*/


//              ALGORITMO ORIGINAL
void loop(){

   info();
   checkColor();
   if(blocks==1){
    moveStay();
    Serial.println("Si veo");
    if(area<30000){
      Serial.println("Si esta lejos");
      while(area<30000 && blocks>0){
        Serial.println("Si quiero ir a la bola");
        info();
        goBall();
        checkColor();
      }
    }
    if(area>30000){
        Serial.println(dGetDirect());
        if(dGetDirect()<175 || dGetDirect()>190){
          while(digitalRead(push)==1){
            Serial.println("ya no esta lejos");
            Serial.println(dGetDirect());
            Serial.print("El area es: ");
            Serial.println(area);
            Serial.println(altura);
            Serial.println(anchura);
            spinBallNor();
            spinNorth();
          }
        }
        else  if(dGetDirect()>170 && dGetDirect()<190){
          while(digitalRead(push)==1){
            info();
            goNorth();
            checkColor();
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
      movPers(-40,40,-40,40);
      delay(1);
    }
    else{
      movPers(40,-40,40,-40);
      delay(1);
    }
    checkColor();
  }
}
