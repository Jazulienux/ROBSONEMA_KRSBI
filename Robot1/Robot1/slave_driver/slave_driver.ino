#include <Wire.h>
#include "driver.h"
#include "gerakan.h"
#define chA   2
#define chB   3

int enc=0;
byte stat=0;

void receiveEvent(){
  stat=((byte)Wire.read());
  if(stat==1){
    mov = ((byte)Wire.read());
    pwmVal = 0;
    pwmVal |= ((int)(Wire.read() << 8));
    pwmVal |= ((int)(Wire.read()));
  }
  else if(stat==2){
    enc=0;  
  }
}

void requestEvent(){
  Wire.write((byte)(enc>>8));  
  Wire.write((byte)(enc)); 
}

void encoder(){
  if(digitalRead(chB)==LOW){
    enc--;
  }
  else {
    enc++;
  }
  //Serial.println(enc);
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  attachInterrupt(0,encoder,RISING);
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(chA, INPUT_PULLUP);
  pinMode(chB, INPUT_PULLUP);
  setup_pwm16();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(pinen)==HIGH){
    run_program();   
    //Serial.print((char)(mov));Serial.print("\t");Serial.println(pwmVal);
  }
  else {
    motor(0,0);
    //Serial.print("S");Serial.print("\t");Serial.println("0");
  }
}
