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
volatile int lastsign=0;

void start_positioning(){
  byte clc=1;
  motor_enable();
  while(1){
    scansensor();
    switch(sendata){  
      case 0b0001: motorDrive('F',0,0,200);er=1;break;
      case 0b0111: motorDrive('B',0,0,200);er=-1;break;
      case 0b1011: motorDrive('A',150,150,150);er=1;break;

      case 0b1010: motorDrive('B',0,0,200);er=1;break;
      case 0b0101: motorDrive('B',200,0,0);er=1;break;
    
      case 0b1001:motorDrive('S',0,0,0);er=1;clc=0;break;
      case 0b0110:motorDrive('B',150,0,150);er=-1;break;
      case 0b0000:motorDrive('B',100,0,100);er=0;break;
          
      case 0b1101: motorDrive('D',150,150,150);er=1;break;
      case 0b1110: motorDrive('B',200,0,0);er=-1;break;
      case 0b1000: motorDrive('F',200,0,0);er=1;break;

      case 0b1111:
        if(er>0){
          motorDrive('F',250,0,250);break;
        }
        if(er<0){
          motorDrive('B',250,0,250);break;
        }
        if(er==0){
          motorDrive('F',250,0,250);break;
        }     
      }
     if(clc==0){
        motorDrive('S',0,0,0);break;
     }
     ros_routine();
  }
  motor_disable();
  delay(250);
  motorDrive('Q',0,350,350);
  while(pr_kiri1){
    motor_enable();
  }
  motorDrive('S',0,0,0);
  motor_disable();
  delay(300);
  motorDrive('L',170,300,170);    
  while(pr_kanan1){
    motor_enable();
    ros_routine();
  }
  motorDrive('S',0,0,0);
  motor_disable();
  delay(250);
  motorDrive('Z',300,300,0);
  while(1){
    scansensor();
    motor_enable();
    if(sens[0]==0||sens[2]==0)break;
    ros_routine();
  }
  motorDrive('S',0,0,0);
  motor_disable();
  motorDrive('B',350,0,350);
  while(1){
    motor_enable();      
    if(!pr_kiri1&&!pr_kanan1)break;
  }
  clc=1;
  while(1){
    scansensor();
    switch(sendata){  
      case 0b0001: motorDrive('F',0,0,200);er=1;break;
      case 0b0010: motorDrive('B',0,0,200);er=-1;break;
      case 0b0111: motorDrive('A',150,150,200);er=-1;break;
      case 0b1011: motorDrive('F',0,0,200);er=1;break;

      case 0b1010: motorDrive('B',0,0,200);er=1;break;
      case 0b0101: motorDrive('B',200,0,0);er=1;break;
    
      case 0b1001: motorDrive('S',0,0,0);er=1;clc=0;break;
      case 0b0110: motorDrive('B',150,0,150);er=-1;break;
      case 0b0000: motorDrive('B',100,0,100);er=0;break;

      case 0b1101: motorDrive('F',200,0,0);er=1;break;
      case 0b1110: motorDrive('D',150,150,150);er=-1;break;
      case 0b0100: motorDrive('B',200,0,0);er=-1;break;
      case 0b1000: motorDrive('F',200,0,0);er=1;break;
      
      case 0b1111:
          if(er>0){
            motorDrive('F',200,0,200);break;
          }
          if(er<0){
            motorDrive('B',250,0,250);break;
          }
          if(er==0){
            motorDrive('F',200,0,200);break;
          }
      }
      if(clc==0){
        motorDrive('S',0,0,0);
        motor_disable();
        break;
      }
      motor_enable();
      ros_routine();
  }
  er=0;
  delay(200);
  while(1){
    scansensor();
    switch(sendata){
        case 0b0000:
        case 0b0100:
        case 0b1000:
        case 0b1100:
            adds=0;er=0;break;
            
        case 0b0001:
        case 0b0101:
        case 0b1001:
        case 0b1101:
            adds=-20;er=1;break;

        case 0b0010:
        case 0b0110:
        case 0b1010:
        case 0b1110:
            adds=30;er=-1;break;

        case 0b0011:
        case 0b0111:
        case 0b1011:
        case 0b1111:
            if(er>0){
              adds=-20;break;
            }
            else{
              adds=40;break;
            }
    }
    motorDrive('R',165+adds,300-adds,165+adds);
    motor_enable();  
    if(!pr_kanan){
      u_odom_x = 3.5;
      u_odom_y = 0;
      u_odom_z = 0;
      update_position();
      ros_routine();
      break;  
    }
    ros_routine();
  }
  motorDrive('S',0,0,0);
  transmitDataDribbler('B',dribble_spd[1],0);//B
  motor_disable();  
  strt = false;
  er = 0;
  ros_routine();
  delay(300);
}

void start_positioning1(){
  byte clc=1;
  motor_enable();
  while(1){
    scansensor();
    switch(sendata){  
      case 0b0001: motorDrive('F',0,0,200);er=1;break;
      case 0b0111: motorDrive('B',0,0,200);er=-1;break;
      case 0b1011: motorDrive('A',150,150,150);er=1;break;

      case 0b1010: motorDrive('B',0,0,200);er=1;break;
      case 0b0101: motorDrive('B',200,0,0);er=1;break;
    
      case 0b1001:motorDrive('S',0,0,0);er=1;clc=0;break;
      case 0b0110:motorDrive('B',150,0,150);er=-1;break;
      case 0b0000:motorDrive('B',100,0,100);er=0;break;
          
      case 0b1101: motorDrive('D',150,150,150);er=1;break;
      case 0b1110: motorDrive('B',200,0,0);er=-1;break;
      case 0b1000: motorDrive('F',200,0,0);er=1;break;

      case 0b1111:
        if(er>0){
          motorDrive('F',250,0,250);break;
        }
        if(er<0){
          motorDrive('B',250,0,250);break;
        }
        if(er==0){
          motorDrive('F',250,0,250);break;
        }     
      }
     if(clc==0){
        motorDrive('S',0,0,0);break;
     }
     positioning_screen(3);
     ros_routine();  
  } 
  motor_disable();
  delay(250);
  motorDrive('E',350,350,0);
  while(pr_kanan1){
    motor_enable();
    positioning_screen(1);
    ros_routine();
  }
  motorDrive('S',0,0,0);
  motor_disable();
  delay(300);
  motorDrive('R',150,300,150);    
  while(pr_kiri1){
    positioning_screen(2);
    motor_enable();
    ros_routine();
  }
  motorDrive('S',0,0,0);
  motor_disable();
  delay(250);
  motorDrive('C',0,350,350);
  while(1){
    positioning_screen(0);
    scansensor();
    motor_enable();
    if(sens[0]==0||sens[2]==0)break;
    ros_routine();
  }
  motorDrive('S',0,0,0);
  motor_disable();
  motorDrive('B',350,0,350);
  while(1){
    positioning_screen(4);
    motor_enable();      
    if(!pr_kiri1&&!pr_kanan1)break;
    ros_routine();
  }
  clc=1;
  while(1){
    scansensor();
    switch(sendata){  
      case 0b0001: motorDrive('F',0,0,200);er=1;break;
      case 0b0010: motorDrive('B',0,0,200);er=-1;break;
      case 0b0111: motorDrive('A',150,150,200);er=-1;break;
      case 0b1011: motorDrive('F',0,0,200);er=1;break;

      case 0b1010: motorDrive('B',0,0,200);er=1;break;
      case 0b0101: motorDrive('B',200,0,0);er=1;break;
    
      case 0b1001: motorDrive('S',0,0,0);er=1;clc=0;break;
      case 0b0110: motorDrive('B',150,0,150);er=-1;break;
      case 0b0000: motorDrive('B',100,0,100);er=0;break;

      case 0b1101: motorDrive('F',200,0,0);er=1;break;
      case 0b1110: motorDrive('D',150,150,150);er=-1;break;
      case 0b0100: motorDrive('B',200,0,0);er=-1;break;
      case 0b1000: motorDrive('F',200,0,0);er=1;break;
      
      case 0b1111:
          if(er>0){
            motorDrive('F',200,0,200);break;
          }
          if(er<0){
            motorDrive('B',250,0,250);break;
          }
          if(er==0){
            motorDrive('F',200,0,200);break;
          }
      }
      if(clc==0){
        motorDrive('S',0,0,0);
        motor_disable();
        break;
      }
      motor_enable();
      positioning_screen(0);
      ros_routine();
  }
  er=0;
  delay(200);
  clear_screen();
  while(1){
    scansensor();
    switch(sendata){
        case 0b0000:
        case 0b0100:
        case 0b1000:
        case 0b1100:
            adds=0;er=0;break;
            
        case 0b0001:
        case 0b0101:
        case 0b1001:
        case 0b1101:
            adds=-20;er=1;break;

        case 0b0010:
        case 0b0110:
        case 0b1010:
        case 0b1110:
            adds=30;er=-1;break;

        case 0b0011:
        case 0b0111:
        case 0b1011:
        case 0b1111:
            if(er>0){
              adds=-20;break;
            }
            else{
              adds=40;break;
            }
    }
    motorDrive('R', 165+adds, 300-adds, 165+adds);
    motor_enable();  
    if(!pr_kanan){
      u_odom_x = 3.5;
      u_odom_y = 0;
      u_odom_z = 0;
      update_position();
      ros_routine();
      break;  
    }
    ros_routine();
  }
  motorDrive('S',0,0,0);
  u_odom_x=3.5;u_odom_y=0;u_odom_z=0;
  update_position();
  transmitDataDribbler('B',dribble_spd[1],0);//B
  motor_disable();  
  strt = false;
  er = 0;
  ros_routine();
  delay(300);
}

void run_program2(){
  odom_screen();
  if(!tb_enter){
    while(!tb_enter){}
    clear_screen();
    enc_lock=true;
    reset_encoder(2);  
    reset_encoder(3);
    reset_encoder(4);
    mode=0;
  }  
}

void mid_kick(){
  if(odom_x > 3.1){
    switch(sendata){
      case 0b0000:
      case 0b0001:
      case 0b0010:
      case 0b0011:
        error1=0;er=0;break;
      
      case 0b0100:
      case 0b0101:
      case 0b0110:
      case 0b0111:
        error1=1;er=-1;break;
      
      case 0b1000:
      case 0b1001:
      case 0b1010:
      case 0b1011:
        error1=-1;er=1;break;
      
      case 0b1100:
      case 0b1101:
      case 0b1110:
      case 0b1111:
          if(er>0){
            error1=-2;break;
          }
          else{
            error1=2;break;
          } 
      }
      compute_pid1();
      motorDrive('L', 150+adds, 300-adds, 150+adds);
  }
  else if(odom_x<2.9){
    switch(sendata){
      case 0b0000:
      case 0b0100:
      case 0b1000:
      case 0b1100:
        error1=0;er=0;break;
            
      case 0b0001:
      case 0b0101:
      case 0b1001:
      case 0b1101:
        error1=-1;er=1;break;

      case 0b0010:
      case 0b0110:
      case 0b1010:
      case 0b1110:
        error1=1;er=-1;break;

      case 0b0011:
      case 0b0111:
      case 0b1011:
      case 0b1111:
        if(er>0){
          error1=-2;break;
        }
        else{
          error1=2;break;
        }
      }
      compute_pid1();
      motorDrive('R', 150+adds, 300-adds, 150+adds);
  }
  else{
    switch(sendata){  
      case 0b0000: 
        motorDrive('S',0,0,0);er=0;
        lastsign=0;lastmillis = millis();
        kick_ball(delay_rate[0],0);
        break;
      case 0b0001: motorDrive('F',0,0,200);er=1;break;
      case 0b0010: motorDrive('B',0,0,200);er=-1;break;
      case 0b0011: 
          if(er>0){
            motorDrive('F',0,0,200);break;
          }
          else {
            motorDrive('B',0,0,200);break;
          }
      case 0b0100: motorDrive('B',100,0,0);er=-1;break;
      case 0b0101: motorDrive('F',0,0,200);er=1;break;
      case 0b0110: motorDrive('B',200,0,200);er=-1;break;
      case 0b0111: 
          if(er>0){
            motorDrive('F',0,0,200);break;  
          }
          else {
            motorDrive('B',0,0,200);break;
          }
      case 0b1000: motorDrive('F',100,0,0);er=1;break;
      case 0b1001: motorDrive('F',200,0,200);er=1;break;
      case 0b1010: motorDrive('F',200,0,0);er=1;break;
      case 0b1011: 
         if(er>0){
          motorDrive('F',0,0,200);break;  
         }
         else {
          motorDrive('B',0,0,200);break;
         }
      case 0b1100:
         if(er>0){
          motorDrive('F',200,0,0);break;
         }
         else {
          motorDrive('B',200,0,0);break;
         }
      case 0b1101:
          if(er>0){
            motorDrive('F',200,0,0);break;
          }
          else {
            motorDrive('B',200,0,0);break;
          }
      case 0b1110:
          if(er>0){
            motorDrive('F',200,0,0);break;
          }
          else {
            motorDrive('B',200,0,0);break;  
          }
      case 0b1111:
          if(er>0){
            motorDrive('F',200,0,200);break;
          }
          else {
            motorDrive('B',200,0,200);break;
          }
     }
  }  
}

#endif
