#include <Servo.h>
//VARIANTE USANDO UN POTENCIOMETRO PARA DETERMINAR EL ANGULO

Servo miServo;

int angulo=0;
int analogo;
boolean stringComplete = false; 

void setup() 
{
  miServo.attach(9);
  Serial.begin(115200);
  miServo.write(angulo); 
}

void loop() 
{
   if (stringComplete) 
   {
      miServo.write(angulo);  
      Serial.print("Angulo: "); 
      Serial.println(angulo);
      stringComplete = false;
  }
}

void serialEvent() {
  if (Serial.available()) {
      angulo=Serial.read(); 
      if (angulo>180) angulo=180;
      stringComplete = true;
    
  }
}
