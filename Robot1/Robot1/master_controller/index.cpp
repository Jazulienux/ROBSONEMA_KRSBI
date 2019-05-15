#ifndef index_cpp
#define index_cpp

#include <Arduino.h>
#include "index.h"
#include "interface.h"
#include "i2cdriver.h"
#include "linesensor.h"
#include "controlpid.h"
#include "timer.h"
#include "roshandler.h"

volatile bool strt=true;
volatile int adds=0;
bool mvv=false;
volatile int index_lock=0;
byte zzz = 0;

void start_positioning(){}

void run_program2(){
  odom_screen();
  scansensor();
  if(tendang == 2)transmitDataDribbler('F', 100, 0);
  else transmitDataDribbler('B',dribble_spd[2],0);
  if(!tb_atas){}//reset positioning update
  if(sens[0]==0){
    if(((odom_x > 0.3 && odom_x<1.75) || (odom_x > 4.25 && odom_x < 5.7)) && (odom_y > 1.5 && odom_y < 7.75)){
      u_odom_x = odom_x;
      u_odom_y = 4.5;
      u_odom_z = odom_z;
      update_position();
    }  
  }
  if(sens[1]==0){
    if(odom_x > 4.25 && ((odom_y>1.25 && odom_y <= 3.25) || (odom_y >= 5.75 && odom_y < 7.75))){
      u_odom_x = 6.0;
      u_odom_y = odom_y;
      u_odom_z = odom_z;
      update_position();  
    }
  }
  if(sens[3]==0){
    if(odom_x < 1.75 && ((odom_y>1.25 && odom_y <= 3.25) || (odom_y >= 5.75 && odom_y < 7.75))){
      u_odom_x = 0;
      u_odom_y = odom_y;
      u_odom_z = odom_z;
      update_position();  
    }
  }
  if(sens[3]==0 && sens[1]==0){
    if(odom_x<1.75 || odom_x>4.25)
    u_odom_x = odom_x;
    u_odom_y = odom_y;
    u_odom_z = 0;
    update_position();  
  }
  if(!tb_enter){
    while(!tb_enter){}
    clear_screen();
    enc_lock=true;
    index_lokalisasi = 0;
    odom_x = 0;
    odom_y = 0;
    odom_z = 0;
    transmitData(0, 0, 0);
    transmitDataDribbler('S',0,0);
    motor_disable();
    reset_encoder(2);  
    reset_encoder(3);
    reset_encoder(4);
    mode=0;
  }  
  if(index_lokalisasi==2)update_x();
  else if(index_lokalisasi==3)transmitData(0, 0, 0);
  else {
    transmitData(pwm[0], pwm[1], pwm[2]);
    index_lock = 0;
  }
  motor_enable(); 
}

void update_z(){
  switch(sendata){
    case 0b1011:er = 1;break;
    case 0b1101:er = -1;break;
    case 0b1111:
      if(zzz == 0){
        if(odom_y< 4.5)er = 1;
        else if(odom_y>=4.5)er = -1;
        zzz = 1;
      }
      if(er<0)transmitData(200,0,-200);
      else if(er>=0)transmitData(-200,0,200);
      break;
    case 0b0110:
      transmitData(0,0,0);
      u_odom_x=odom_x;u_odom_y=4.5;u_odom_z=0;
      update_position();
      break;
    case 0b1110:
      if(er<0)transmitData(200,0, 0);
      else if(er>=0)transmitData(-200,0,0);
      break;
    case 0b0111:
      if(er<0)transmitData(0,0,-200);
      else if(er>=0)transmitData(0,0,200);  
      break;
    case 0b0101:transmitData(0,0,-200);er = -1;break;
    case 0b0011:transmitData(0,0, 200);er = 1;break;
    case 0b1100:transmitData(200,0,0);er = -1;break;
    case 0b1010:transmitData(-200,0,0);er = 1;break;
  }
}

void update_x(){
  if(index_lock == 0)transmitData(-178,300,-178);  
  if(sens[1]==0){
    transmitData(0, 0, 0);
    delay(250);
    index_complete = 1;
    publish_acc();
    index_complete = 0;
    index_lock =1;
  }
}

void run_program1(){
  scansensor();
  if(!tb_enter){
    while(!tb_enter){}
    clear_screen();
    enc_lock=true;
    transmitData(0, 0, 0);
    transmitDataDribbler('S',0,0);
    motor_disable();
    reset_encoder(2);  
    reset_encoder(3);
    reset_encoder(4);
    mode=0;
  }  
  if(index_lock==0)transmitData(-175,300,-175);
  if(sens[1]==0){
    transmitData(0, 0, 0);
    index_complete = 1;
    publish_acc();
    index_complete = 0;
    index_lock =1;
  } 
  motor_enable();
}
#endif
