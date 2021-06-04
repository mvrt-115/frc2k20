#include <Servo.h>

Servo servo;
int potentioPin = 0; //analog pin that connects to the potentiometer
int val = 0;
void setup() 
{
  servo.attach(9); //attach servo on pin 9 on arduino 
}

void loop() 
{
  val = analogRead(potentioPin);
  val = map(val, 0, 1023, 0, 180);
  servo.write(val);
  Serial.println(val);
  delay(15);
}
