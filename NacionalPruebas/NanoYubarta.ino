#define izq 10
#define der 11
#define intIzq 4
#define intDer 5
#define intBack 6

int iLeftIr = 0;
int iRightIr = 0;

void setup() {
  //Comunicación Cereal
  Serial.begin(9600);
  //Inputs sensores de color
  pinMode(izq,  INPUT);
  pinMode(der, INPUT);
  //Interrupts Mega
  pinMode(intIzq,OUTPUT);
  pinMode(intDer,OUTPUT);
  //HIGH
  digitalWrite(intIzq,HIGH);
  digitalWrite(intDer,HIGH);
}

int prom(int a[], int largo) {
    for(int i=0; i<(largo-1); i++) {
        for(int o=0; o<(largo-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    int t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
   int Promedio=(a[3]+a[4]+a[5]+a[6]+a[7])/5; 
   return Promedio;
}

void values(){  
  int LeftIrPulse[11];
  int RightIrPulse[11];
  int ix=0;
  
  for(ix=0; ix<10; ix++){
    LeftIrPulse[ix]=pulseIn(izq, LOW);
  }
  for(ix=0; ix<10; ix++){
    RightIrPulse[ix]=pulseIn(der, LOW);
  }  
  iLeftIr=prom(LeftIrPulse,11);
  iRightIr=prom(RightIrPulse,11); 
  }
  void printValues(){
    Serial.print(" Left= ");
    Serial.print(iLeftIr);
    Serial.print(" Right= ");
    Serial.println(iRightIr);
  }
  void interrupt(){
    if(iLeftIr<550&&iRightIr<400){
     Serial.println("BACK");
     digitalWrite(intBack,LOW);
     delay(100);
     digitalWrite(intBack,HIGH);     
    }    
    if(iLeftIr<550){
     Serial.println("IZQ");
     digitalWrite(intIzq,LOW);
     delay(100);
     digitalWrite(intIzq,HIGH);
    }
    if(iRightIr<400){      
     Serial.println("DER");
     digitalWrite(intDer,LOW);
     delay(100);
     digitalWrite(intDer,HIGH);
    }
  }

void loop() {
  values();
  printValues();
  interrupt(); 
  
}
