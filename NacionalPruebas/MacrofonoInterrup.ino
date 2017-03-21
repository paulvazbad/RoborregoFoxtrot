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
#define COLORIZQ        18
#define COLORDER        19

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
bool ultPos=0;

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

/*attachInterrupt(digitalPinToInterrupt(COLORIZQ) , moveRight, LOW);

attachInterrupt(digitalPinToInterrupt(COLORDER),moveLeft , LOW);
*/
  }

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
//                       SPIN ORIGINAL
void spinBallNor(){
  double dNorti=dGetDirect();
  if(dNorti<180){
    while(dNorti<175){
    dNorti=dGetDirect();
    movPers(0,57,80,-75);
    }
  }
   else if(dNorti>180){
     while(dNorti>185){
      dNorti=dGetDirect();
      movPers(57,0,-80,80);
      }
    }
    moveStay();
}
moveRight(){
  moveStay();
  movPers(-40,40,40,-40);
}
moveLeft(){
  moveStay();
  movPers(40, -40, -40, 40);
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
    }
    moveFrenos();
    //movPers(80,80,80,80);
    delay(100);
    moveStay();
  }
  else if(dNorti>190){
    while(dNorti>185){
      dNorti=dGetDirect();
      Serial.println(dNorti);
      movPers(-20,20,-20,20);
      delay(1);
    }
    moveFrenos();
    //movPers(80,80,80,80);
    delay(100);
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
  //movPers(50,50,50,50);
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
bool ultPosFun(){
  if(cordx<159){
    ultPos=true;  //Izq
  }
  else{
   ultPos=false;
  }
}
//              ALGORITMO ORIGINAL
void loop(){
   info();
   //La ve
   if(blocks==1){
    moveStay();
    Serial.println("Si veo");
    //Lejos
    if(area<30000){
      Serial.println("Si esta lejos");
      while(area<30000 && blocks>0){
        Serial.println("Si quiero ir a la bola");
        info();
        goBall();
      }
    }
    //Cerca
    if(area>30000){
        Serial.println("ya no esta lejos");
        Serial.println(dGetDirect());
        if(dGetDirect()<175 || dGetDirect()>190){
          Serial.println(dGetDirect());
          Serial.print("El area es: ");
          Serial.println(area);
          Serial.println(altura);
          Serial.println(anchura);
          spinBallNor();
        }
        else  if(dGetDirect()>170 && dGetDirect()<190){
          while(blocks!=0){
            info();
            goNorth();
            Serial.println("VOY AL NORTEE  ");
          }
        }
      }
      ultPosFun();
  }
  //No la ve
  else{
    if(ultPos==true){
      movPers(-40,40,-40,40);
      delay(1);
    }
    else{
      movPers(40,-40,40,-40);
      delay(1);
    }
  }
}
