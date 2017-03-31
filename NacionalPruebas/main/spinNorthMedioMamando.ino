void spinNorth(){
  dNorti=dGetDirect();
  if(dNorti<175){
    while(dNorti<180){
      dNorti=dGetDirect();
      Serial.println(dNorti);
      int iPot;
      iPot=map(dGetDirect(),90,180,30,0);
      iPot= constrain(iPot, 10, 20);
      movePers(15,-15,15,-15);
      delay(1);
      }
      //moveFrenos();
      //movePers(80,80,80,80);
      //delay(100);
      moveStay();
  }
  else if(dNorti>185){
    while(dNorti>180){
      dNorti=dGetDirect();
      Serial.println(dNorti);
      int iPot;
      iPot=map(dGetDirect(),270,180,30,0);
      iPot= constrain(iPot, 5, 20);
      movePers(-15,15,-15,15);
      delay(1);
    }
    //moveFrenos();
    //movePers(80,80,80,80);
    //delay(100);
    moveStay();
  }
}
