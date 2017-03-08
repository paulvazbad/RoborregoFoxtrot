//Definiendo motores
#define FrontMotorLeftS1 5
#define FrontMotorLeftS2 4
#define FrontMotorRightS1 11
#define FrontMotorRightS2 10
#define BackMotorLeftS1 6
#define BackMotorLeftS2 7
#define BackMotorRightS1 9
#define BackMotorRightS2 8
#define RodilloS1       13
#define RodilloS2       12

void setup(void) {
  Serial.begin(9600);
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
  analogWrite(RodilloS1,LOW);
  analogWrite(RodilloS2,LOW);
}

void shoot(int pot){
  analogWrite(RodilloS1,pot);
  analogWrite(RodilloS2,LOW);
}
void  suck(int pot) {
  analogWrite(RodilloS1,LOW);
  analogWrite(RodilloS2,pot);
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
void loop(){
  moveStay();
  delay(500);
  movPers(0,0,50,0);
  //moveForward(150,5000);
  delay(3000);
  moveStay();/*
  delay(500);
  movPers(-50,-50,-50,-50);
  //moveForward(150,5000);
  delay(3000);
  moveStay();
  delay(500);
  movPers(50,-50,-50,50);
  //moveForward(150,5000);
  delay(3000);
  moveStay();
  delay(500);
  movPers(-50,50,50,-50);
  //moveForward(150,5000);
  delay(3000);*/
  
}


