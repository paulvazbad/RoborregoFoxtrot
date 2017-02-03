void setup() {
  pinMode(15,INPUT);
  Serial.begin(9600); 

}

void loop() {
  int time = pulseIn(15,LOW);  
  if(time){
    Serial.println("Existe la pelota en mi direccion");
    
    }// put your main code here, to run repeatedly:

}
