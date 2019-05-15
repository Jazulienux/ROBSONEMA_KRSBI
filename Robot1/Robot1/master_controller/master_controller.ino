//new player robot 1

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
int lastsign=0;

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
  if(!pr_bola){
    ball_detection(1);
    if(tendang == 1)kick_ball(delay_rate[0],0);
    else if(tendang == 3)kick_ball(delay_rate[1],0);
    else if(tendang == 4)kick_ball(delay_rate[1],0);
  }
  else if(pr_bola){
    ball_detection(0);
  }
  if(mode==0)standby_screen();
  else if(mode==1)run_program2();
  else if(mode==2)run_program1();
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
