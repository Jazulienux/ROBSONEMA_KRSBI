#ifndef timer_cpp
#define timer_cpp

#include <Arduino.h>
#include <Wire.h>
#include "timer.h"
#include "i2cdriver.h"
#include "roshandler.h"
#define prescaler 256
#define f_cpu     16000000

int start=0;
volatile bool update_enc = false;

ISR(TIMER1_OVF_vect){
  if(enc_lock==false){
    update_enc = true;
  }
  TCNT1 = start;
}

void init_timer(int milsec){
  start = 65535 - (milsec*(f_cpu/1000)/prescaler);
  //start = 0xBDB;
  TCCR1A = 0;
  TCCR1B = (1<<CS12) | (0<<CS11) | (0<<CS10);
  TCNT1 = start;
  TIMSK1 |= (1<<TOIE1);
  Wire.begin();
  sei();
  enc_lock=true;
}

#endif
