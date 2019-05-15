#ifndef memory_cpp
#define memory_cpp

#include <Arduino.h>
#include "memory.h"
#include "controlpid.h"
#include "interface.h"
#include "linesensor.h"

#include <EEPROM.h>

byte ekp[2]={0,2}, eki[2]={4,6}, ekd[2]={8,10}, edelay_rate[4]={12,13, 39, 40}, espoint[2]={14,16}, eylim[2]={18,20};
byte espd_dep[2]={22,24}, ethrotle[5]={26,28,30,32,34}, edribble_spd[5]={36,37,38,59,60};
byte eadditional[4]={42,44,46,48}, esensitive[4]={50,52,54,56}, epos=58;

void readFromEeprom(){
  for(unsigned char i=0; i<2;i++){
    motor[i].kp = float(readEeprom16bit(ekp[i],ekp[i]+1))/10;
    motor[i].ki = float(readEeprom16bit(eki[i],eki[i]+1))/10;
    motor[i].kd = float(readEeprom16bit(ekd[i],ekd[i]+1))/10;
    spoint[i]=readEeprom16bit(espoint[i],espoint[i]+1);
    ylim[i]=readEeprom16bit(eylim[i],eylim[i]+1);
    spd_dep[i]=readEeprom16bit(espd_dep[i],espd_dep[i]+1);
  }
  for(unsigned char i=0; i<5;i++){
    dribble_spd[i]=EEPROM.read(edribble_spd[i]);
  }
  for(unsigned char i=0;i<4;i++){
    delay_rate[i] = EEPROM.read(edelay_rate[i]);  
    additional[i] = readEeprom16bit(eadditional[i],eadditional[i]+1);  
    sensitive[i] = readEeprom16bit(esensitive[i],esensitive[i]+1);  
  }
  for(unsigned char i=0; i<5;i++){
    throtle[i] = readEeprom16bit(ethrotle[i],ethrotle[i]+1);  
  }
  pos = EEPROM.read(epos);
}

void saveToEeprom(byte type){
  if(type==1){
    for(unsigned char i=0; i < 2; i++){
      writeEeprom16bit(ekp[i], ekp[i]+1, motor[i].kp*10);
      writeEeprom16bit(eki[i], eki[i]+1, motor[i].ki*10);
      writeEeprom16bit(ekd[i], ekd[i]+1, motor[i].kd*10); 
    }
  }
  else if(type==2){
    for(unsigned char i=0;i<4;i++){
      EEPROM.write(edelay_rate[i],delay_rate[i]);
    }
  }
  else if(type==3){
    for(unsigned char i=0; i < 2; i++){
      writeEeprom16bit(espoint[i], espoint[i]+1, spoint[i]);
      writeEeprom16bit(eylim[i], eylim[i]+1, ylim[i]);  
    }
  }
  else if(type==4){
    for(unsigned char i=0;i<2;i++){
      writeEeprom16bit(espd_dep[i], espd_dep[i]+1, spd_dep[i]);
    }
    for(unsigned char i=0;i<5;i++){
      EEPROM.write(edribble_spd[i], dribble_spd[i]);  
    }
    for(unsigned char i=0;i<5;i++){
      writeEeprom16bit(ethrotle[i], ethrotle[i]+1, throtle[i]);
    }
  }
  else if(type==5){
    for(unsigned char i=0;i<4;i++){
      writeEeprom16bit(eadditional[i],eadditional[i]+1,additional[i]);
    } 
  }
  else if(type==6){
    for(unsigned char i=0;i<4;i++){
      writeEeprom16bit(esensitive[i],esensitive[i]+1,sensitive[i]);
    }  
  }
  else if(type==7)EEPROM.write(epos,pos);
}

void writeEeprom16bit(int adr1, int adr2, int value){
  byte lowbyte = (value & 0xff);
  byte highbyte = ((value >> 8) & 0xff);
  EEPROM.write(adr1,lowbyte);
  EEPROM.write(adr2,highbyte);
}

int readEeprom16bit(int adr1, int adr2){
  int lowbyte = EEPROM.read(adr1);
  int highbyte = EEPROM.read(adr2);
  int data = ((highbyte<<8)&0xffff) + (lowbyte&0xff);   
  return data; 
}

#endif
