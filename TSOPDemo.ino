#include <Arduino.h>

int pinTSOP=8, iNoEncontrado=0;
bool bSenal;
void setup(){
  pinMode(pinTSOP, INPUT);
  pinMode(10,OUTPUT);
  Serial.begin(9600);
  digitalWrite(10, 1);

}

void loop(){
  digitalWrite(10, 1);
  bSenal = digitalRead(pinTSOP);
  int bPulso;
  bPulso = pulseIn(pinTSOP,LOW);
  Serial.println("Valor: ");
  Serial.println(bPulso);

  if(bPulso>50&&bPulso<800){
    Serial.println("Encontre la pelota");
    iNoEncontrado=0;
  }
  else{
    iNoEncontrado++;
    Serial.println("No esta la pelota");
    if(iNoEncontrado>20){
      Serial.println(iNoEncontrado);
      digitalWrite(10, 0);
      iNoEncontrado=0;
      Serial.println("Reiniciando sensor");
      delay(3000);

    }
  }



  delay(200);
}
