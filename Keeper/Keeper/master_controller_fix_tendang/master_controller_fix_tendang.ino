//keeper robot

#include "i2cdriver.h"
#include "interface.h"
#include "controlpid.h"
#include "memory.h"
#include "roshandler.h"
#include "timer.h"
#include "linesensor.h"
#include "index.h"

ros::NodeHandle nh;

String cmd="s";

void run_program();
void run_program1();

void setup() {
  // put your setup code here, to run once:
  Wire.begin();//I2C--
  initslave('L',KIRI);
  initslave('M',TENGAH);
  initslave('R',KANAN);
  initslave('D',DRIBBLER);
  pinMode(pinen,OUTPUT);
  pinMode(trigger_pin,OUTPUT);
  digitalWrite(pinen,LOW);
  digitalWrite(trigger_pin,LOW);
  lmp=1;
  init_interface();
  readFromEeprom();
  init_ros();
  init_timer(100);
  reset_encoder(2);
  reset_encoder(3);
  reset_encoder(4);
}

void loop(){
  // put your main code here, to run repeatedly:
  if(!tb_kiri){
    while(!tb_kiri){}
    if(++lmp>1)lmp=0;
  }
  if(lmp==1)lmp_on;
  else lmp_off;
  if(mode==0){
    standby_screen();
    if(!pr_bola){
      index_bola = 1;
      publish_bola();
    }
    else if(pr_bola){
      index_bola = 0;  
      publish_bola();
    }
  }
  else if(mode==1){
    if(!pr_bola){
      index_bola = 1;
      publish_bola();
      kick_ball(delay_rate[0],0);
    }
    else if(pr_bola){
      run_program1();
      index_bola = 0;  
      publish_bola();
    }
    if(!pr_kanan){u_odom_x=3.5;u_odom_y=0;u_odom_z=0;update_position();}
    else if(!pr_kiri){u_odom_x=2.5;u_odom_y=0;u_odom_z=0;update_position();}
  }
  else if(mode==2){
    run_program2();
    if(!pr_kanan){u_odom_x=3.5;u_odom_y=0;u_odom_z=0;update_position();}
    else if(!pr_kiri){u_odom_x=2.5;u_odom_y=0;u_odom_z=0;update_position();}
  }
  if(update_enc==true){
    update_encoder();
    publish_encoder();
    reset_encoder(2);
    reset_encoder(3);
    reset_encoder(4);
    update_enc=false; 
  }
  nh.spinOnce();
  delay(10);
}

void run_program1(){
  int s1, s2;
  transmitDataDribbler('B',dribble_spd[1],0);//B
  //sens[1] = kiri depan, sens[3] = kanan depan
  if(strt==true){
    if(st==0)start_positioning(); 
    else start_positioning1(); 
  }
  //============================================================
  scansensor();
  odom_screen();
  //bola center
  //if(action==1){
    if(!pr_kiri2&&!pr_kanan2){
      if((!pr_kiri1)&&(!pr_kanan1)){
        motorDrive('F',200,0,200);
        er = 1;  
      }
      else if((pr_kiri1)&&(!pr_kanan1)){
        motorDrive('F',0,0,200);
        er = 1;
      }
      else if((!pr_kiri1)&&(pr_kanan1)){motorDrive('F',200,0,0);
        er = 1;
      }  
      else if((pr_kiri1)&&(pr_kanan1)){
        compute_camera();
        if(error==0||(cam>=0&&!pr_kanan)||(cam<=0&&!pr_kiri)){
          switch(sendata){  
          case 0b0000: 
            motorDrive('S',0,0,0);er=0;
            lastsign=0;lastmillis = millis();
            if(!pr_kanan){u_odom_x=3.4;u_odom_y=0;u_odom_z=0;update_position();}
            else if(!pr_kiri){u_odom_x=2.6;u_odom_y=0;u_odom_z=0;update_position();}
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
        else if(sign>0){
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
          if(sign != lastsign){
            lastmillis = millis();  
          }
          if(pr_kanan)motorDrive('R', pwm[0]+adds, pwm[1]-adds, pwm[2]+adds);
          else motorDrive('S',0,0,0);
          lastsign = sign;
        }
        else if(sign<0){
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
            if(sign != lastsign){
              lastmillis = millis();  
            }
            if(pr_kiri)motorDrive('L', pwm[0]+adds, pwm[1]-adds, pwm[2]+adds);
            else motorDrive('S',0,0,0);
            lastsign = sign;
          }
        }
      }
      else if((!pr_kiri2) && (!pr_kanan1)){
        motorDrive('Z',200,200,0);
        lastmillis = millis();
        lastsign = 0;
        er = 1;
      }
      else if((!pr_kiri1) && (!pr_kanan2)){
        motorDrive('C',0,200,200);
        lastmillis = millis();
        lastsign = 0;
        er = 1;
      }
      else if((!pr_kiri2) && (pr_kanan2)){
        motorDrive('B',50,0,200);
        lastmillis = millis();
        lastsign = 0;
      }
      else if((pr_kiri2) && (!pr_kanan2)){
        motorDrive('B',200,0,50);
        lastmillis = millis();
        lastsign = 0;
      }
      else if((pr_kiri2)&&(pr_kanan2)){
        motorDrive('B',200,0,200);  
        lastmillis = millis();
        lastsign = 0;
      }
    //}
   /*else if(action == 2){
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
   }*/
   /*else if(action == 0){
    motorDrive('S',0,0,0);
   }*/
  //===================================================================
  motor_enable();
  if(!tb_enter){
    while(!tb_enter){}
    mode=0;lmp=1;
    transmitData('S',0,0,0);
    transmitDataDribbler('S',0,0);
    motor_disable();
    clear_screen();
    lmp_on;
  }
}
