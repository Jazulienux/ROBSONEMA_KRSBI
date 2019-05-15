#ifndef driver_cpp
#define driver_cpp

#include <Arduino.h>
#include "driver.h"

volatile byte mov = 'S', dir=0;
volatile int pwmVal = 0;
volatile byte pinen = 11;

void setup_pwm16(){
  //pinMode(9,OUTPUT);
  //pinMode(10,OUTPUT);
  DDRB |=_BV(PB1)|_BV(PB2);
  pinMode(pinen,INPUT_PULLUP);
  TCCR1A = (1 << COM1A1) | (0 << COM1A0 ) | (1 << COM1B1) | (0<<COM1B0)
            | (1 << WGM11) | (0 << WGM10);
  //TCCR1A = 0b10100010;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11)
            | (1 << CS10);
  ICR1 = 0x3ff;//pwm
}

void analogWrite10(unsigned char pin, int pwm){
  switch(pin){
      case 9 : OCR1B = pwm;break;
      case 10: OCR1A = pwm;break;
  }
}

void motor(byte dirs, int pwm){
  if(pwm<0)pwm=abs(pwm);
  else if(pwm>1023)pwm=1023;
  if(dirs==1){//forward
    analogWrite10(9, pwm);//pwm
    analogWrite10(10, 0); // low
    //Serial.println("Forward");
  }
  else if(dirs==2){//backward
    analogWrite10(9, 0);//pwm
    analogWrite10(10, pwm); // low
    //Serial.println("backward");
  }
  else {
    analogWrite10(9, 0);//pwm
    analogWrite10(10, 0); // low
    //Serial.println("Stop");
  }
}

void print_data(int var, int val){
  var = val;
  Serial.println(var); 
}

#endif

