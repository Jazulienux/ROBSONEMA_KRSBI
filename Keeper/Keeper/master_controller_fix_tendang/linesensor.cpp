#ifndef linesensor_cpp
#define linesensor_cpp

#include <Arduino.h>
#include "linesensor.h"
#include "interface.h"
#include "memory.h"

/*
 kiri depan = A2
 kiri belakang = A0
 kanan depan = A7
 kanan belakang = A6
 */

volatile byte pinsensor[4]={A0, A2, A6, A7}, sens[4], sendata=0;
volatile int sensitive[4], input_check[4];

void scansensor(){
  int in=0;
  sendata=0;
  for(unsigned char i=0; i <4;i++){
    in = analogRead(pinsensor[i]);
    if(in>sensitive[i]){
      sens[i]=1;
      switch(i){
        case 0: sendata+=8;break;//kiri belakang
        case 1: sendata+=4;break;//kiri depan
        case 2: sendata+=1;break;//kanan belakang
        case 3: sendata+=2;break;//kanan depan
      }
    }
    else sens[i]=0;
  }
  scansensor_screen();
}

void kalibrasi(){
  int high[4], low[4], in;
  clear_screen();
  for(unsigned char h = 0; h <4; h++){
    high[h]=0;low[h]=1023;  
  }
  while(1){
    kalibrasi_screen();
    for(unsigned char i=0; i<4;i++){
      in = analogRead(pinsensor[i]);
      if(in>high[i])high[i]=in;
      if(in<low[i])low[i]=in;
    }
    if(!tb_back){
      while(!tb_back){}
      for(unsigned char h = 0;h<4;h++){
        sensitive[h]=((high[h]-low[h])/2)+low[h];
        saveToEeprom(6);
      }
      clear_screen();
      delay(500);
      break;  
    }  
  }
}

void cek_sensor(){
  clear_screen();
  while(1){
    for(unsigned char i=0; i<4;i++){
      input_check[i] = analogRead(pinsensor[i]);  
    }
    check_screen();
    if(!tb_back){
      while(!tb_back){};
      break;  
    }  
  }
  clear_screen();  
}

#endif
