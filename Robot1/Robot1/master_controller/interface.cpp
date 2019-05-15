#ifndef interface_cpp
#define interface_cpp

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <stdio.h>
#include <string.h>
#include "interface.h"
#include "character.h"
#include "memory.h"
#include "controlpid.h"
#include "i2cdriver.h"
#include "roshandler.h"
#include "linesensor.h"
#include "timer.h"
#include "index.h"

LiquidCrystal lcd(A9, A8, A10, A11, A12, A13);

volatile byte pintombol[6]={52, 50, 48, 46, 44, 42}, mode=0, delay_rate[4]={10, 20, 10, 20}, dribble_spd[5]={15, 100, 150, 100, 100}, lmp=0, pos=1, pinproximity[4]={23,25,27,29};
volatile int spd_dep[2]={300,500}, spd_bel=0, spoint[2]={130,170}, ylim[2]={80, 110}, xyz_camera[3], throtle[5]={250, 250, 250, 250, 250}, spdmax=500, spdmin=300;
volatile int additional[4]={0,0,0,0};
char buff[33];

void init_interface(){
  for(unsigned char i=0; i < 6; i++){
    pinMode(pintombol[i],INPUT_PULLUP);  
  }
  for(unsigned char i=0; i < 4; i++){
    pinMode(pinproximity[i], INPUT_PULLUP);  
  }
  pinMode(lcd_lmp, OUTPUT);
  lcd.createChar(1,enter);
  lcd.createChar(2,back);
  lcd.createChar(3,cursL);
  lcd.createChar(4,cursU);
  lcd.createChar(5,cursD);
  lcd.createChar(0,cursR);
  lcd.begin(20,4);
  lcd.clear();
  lmp_on;
}

void clear_screen(){
  lcd.clear();  
}

void odom_screen1(){
  lcd.setCursor(0,0);
  lcd.print(sendata);
  lcd.setCursor(1,1);
  lcd.print("x: ");lcd.print(odom_x);
  lcd.setCursor(1,2);
  lcd.print("y: ");lcd.print(odom_y);
  lcd.setCursor(1,3);
  lcd.print("z: ");lcd.print(odom_z);
}

void odom_screen(){
  lcd.setCursor(0,0);
  lcd.print(index_lokalisasi);
  lcd.setCursor(1,1);
  lcd.print("x: ");lcd.print(odom_x);
  lcd.setCursor(1,2);
  lcd.print("y: ");lcd.print(odom_y);
  lcd.setCursor(1,3);
  lcd.print("z: ");lcd.print(odom_z);
}

void standby_screen(){
  lcd.setCursor(4,0);lcd.print("ROBSONEMA-PL");
  lcd.setCursor(8,1);
  if(pos==1)lcd.print("(Fw)");
  else if(pos==2)lcd.print("(Df)");
  else lcd.print("(Gk)");
  lcd.setCursor(0,3);lcd.write(byte(3));lcd.print("LMPS");
  lcd.setCursor(7,3);lcd.write(byte(2));lcd.print("START");
  lcd.setCursor(15,3);lcd.print("MENU");lcd.write(byte(0));
  if(!tb_atas){
    while(!tb_enter){};
    clear_screen();
    reset_encoder(2);
    reset_encoder(3);
    reset_encoder(4);
    enc_lock=false;
    mode=2;
  }
  if(!tb_kanan){
    while(!tb_kanan){};
    menu_screen();
  }
  if(!tb_bawah){
    while(!tb_bawah){};
    clear_screen();
    menu_pos();  
  }
  if(!tb_enter){
    while(!tb_enter){};
    clear_screen();
    reset_encoder(2);
    reset_encoder(3);
    reset_encoder(4);
    u_odom_x = 6.0;
    u_odom_y = 0;
    u_odom_z = 0;
    update_position();
    enc_lock=false;
    mode=1;  
  }
  if(!tb_back){
    while(!tb_back){}
    clear_screen();
    while(1){
      lcd.setCursor(0,1);
      lcd.print("Stand - By");
      ros_routine();
      if(index_action > 0 )break;
      delay(10);
    }
    while(1){
      scansensor();
      transmitData(-200,0,200);
      motor_enable();
      if(sens[0]==0 || sens[3]==0){
        transmitData(0,0,0);
        delay(200);
        break;
      }
      ros_routine();
    }
    clear_screen();
    reset_encoder(2);
    reset_encoder(3);
    reset_encoder(4);
    u_odom_x = 6.0;
    u_odom_y = 0;
    u_odom_z = 0;
    update_position();
    enc_lock=false;
    mode=1;  
  }
  if(!tb_atas){
    while(!tb_atas){}
    clear_screen();
    reset_encoder(2);
    reset_encoder(3);
    reset_encoder(4);
    u_odom_x = 6.0;
    u_odom_y = 0;
    u_odom_z = 0;
    update_position();
    enc_lock=false;
    mode=2;
  }
}

void menu_pos(){
  clear_screen();  
  while(1){
    if(!tb_atas)if(--pos<1)pos=1;
    if(!tb_bawah)if(++pos>3)pos=3;
    lcd.setCursor(0,0);lcd.print(" == SET POSISI == ");  
    lcd.setCursor(0,1);lcd.print(" Fw ");
    lcd.setCursor(0,2);lcd.print(" Df ");
    lcd.setCursor(0,3);lcd.print(" Gk ");
    lcd.setCursor(0,pos);
    if(!tb_back){
      while(!tb_back){};
      saveToEeprom(7);
      lmp_off;clear_screen();
      delay(500);lmp_on;
      break;
    }
    lcd.write(byte(0));
    delay(200);
  }
}

void scansensor_screen(){
  lcd.setCursor(9,0);
  if(sens[0]==0)lcd.write(95);
  else lcd.write(0xff); 
  lcd.setCursor(19,1);
  if(sens[1]==0)lcd.write(95);
  else lcd.write(0xff); 
  lcd.setCursor(9,3);
  if(sens[2]==0)lcd.write(95);
  else lcd.write(0xff); 
  lcd.setCursor(0,1);
  if(sens[3]==0)lcd.write(95);
  else lcd.write(0xff); 
}

void kalibrasi_screen(){
  lcd.setCursor(0,0);lcd.print("  Kalibrasi Sensor  ");
  lcd.setCursor(1,3);lcd.write(byte(2));lcd.print("Exit");
}

void manual_screen(){
  lcd.setCursor(0,0);lcd.print("= MANUAL ROS MODE  =");
  lcd.setCursor(0,1);lcd.print("MASTER URI: ");
  lcd.setCursor(4,2);lcd.print("192.168.0.102");
}

void menu_screen(){
  byte x=1, y=1;
  clear_screen();
  while(1){
    lmp_on;
    if(!tb_atas)if(--y<1)y=1;
    if(!tb_bawah)if(++y>3)y=3;
    if(!tb_kanan)x=2;
    if(!tb_kiri)x=1;
    lcd.setCursor(0,0);
    lcd.print("  ======MENU======  ");
    lcd.setCursor(0,1);lcd.print("  PID       CAMERA ");
    lcd.setCursor(0,2);lcd.print("  SENSOR    SPEED  ");
    lcd.setCursor(0,3);lcd.print("  KICKER    MOTOR  ");
    if(x==1)lcd.setCursor(x,y);
    else if(x==2)lcd.setCursor(x+9,y);
    if(x==1&&y==1){
      if(!tb_enter){
        while(!tb_enter){};
        menu_pid();clear_screen();
      }  
    }
    else if(x==1&&y==2){
      if(!tb_enter){
        while(!tb_enter){};
        menu_sensor();clear_screen();
      }
    }
    else if(x==1&&y==3){
      if(!tb_enter){
        while(!tb_enter){};
        menu_kicker();clear_screen();
      }
    }
    else if(x==2&&y==1){
      if(!tb_enter){
        while(!tb_enter){};
        menu_camera();clear_screen();
      }  
    }
    else if(x==2&&y==2){
      if(!tb_enter){
        while(!tb_enter){};
        menu_speed();clear_screen();
      }
    }
    else if(x==2&&y==3){
      if(!tb_enter){
        while(!tb_enter){};
        menu_motor();clear_screen();
      }
    }
    lcd.write(byte(0));
    if(!tb_back){
      while(!tb_back){}
      clear_screen();lmp_off;
      delay(500);lmp_on;
      break;
    }
    delay(200);
  }
}

void running_screen(){
  lcd.setCursor(0,0);
  lcd.print("run program");
  sprintf(buff,"Kiri  :%04d  ",enc[0]);lcd.setCursor(0,1);lcd.print(buff);
  sprintf(buff,"Tengah:%04d  ",enc[1]);lcd.setCursor(0,2);lcd.print(buff);
  sprintf(buff,"Kanan :%04d  ",enc[2]);lcd.setCursor(0,3);lcd.print(buff);  
}

void check_screen(){
    sprintf(buff,"A0:%04d S:%03d",input_check[0], sensitive[0]);lcd.setCursor(0,0);lcd.print(buff); 
    sprintf(buff,"A2:%04d S:%03d",input_check[1], sensitive[1]);lcd.setCursor(0,1);lcd.print(buff);
    sprintf(buff,"A3:%04d S:%03d",input_check[2], sensitive[2]);lcd.setCursor(0,2);lcd.print(buff);
    sprintf(buff,"A4:%04d S:%03d",input_check[3], sensitive[3]);lcd.setCursor(0,3);lcd.print(buff);  
}

void menu_pid(){
  byte x = 1, y = 1, sec=0, lock=0;
  float tambahan = 1;
  clear_screen();  
  while(1){
    lmp_on;
    if(!tb_enter){
      while(!tb_enter){
        sec++;
        delay(200);
        if(sec>10){
          lmp_off;delay(200);lmp_on;
        }
        else if(sec>5){
          if(lock==0){lmp_off;delay(200);lmp_on;lock=1;}
        }
      }
      if(sec>10)tambahan=1;
      else if(sec>5)tambahan=0.1;
      else if(++y>3)y=1;
      sec=0;lock=0;
    }
    if(!tb_kiri)x=1;
    if(!tb_kanan)x=2;
    lcd.setCursor(0,0);lcd.print("Depan:  ");lcd.setCursor(10,0);lcd.print("Belkg: ");lcd.print(tambahan);
    lcd.setCursor(0,1);lcd.print(" Kp:");lcd.print(motor[0].kp,1);lcd.print("  ");lcd.setCursor(10,1);lcd.print(" Kp:");lcd.print(motor[1].kp,1);lcd.print("  ");
    lcd.setCursor(0,2);lcd.print(" Ki:");lcd.print(motor[0].ki,1);lcd.print("  ");lcd.setCursor(10,2);lcd.print(" Ki:");lcd.print(motor[1].ki,1);lcd.print("  ");
    lcd.setCursor(0,3);lcd.print(" Kd:");lcd.print(motor[0].kd,1);lcd.print("  ");lcd.setCursor(10,3);lcd.print(" Kd:");lcd.print(motor[1].kd,1);lcd.print("  ");
    if(x==1)lcd.setCursor(x-1,y);
    else lcd.setCursor(x+8,y);
    if(x==1&&y==1){
      if(!tb_atas)motor[0].kp+=tambahan;
      if(!tb_bawah)motor[0].kp-=tambahan;  
    }
    else if(x==2&&y==1){
      if(!tb_atas)motor[1].kp+=tambahan;
      if(!tb_bawah)motor[1].kp-=tambahan;    
    }
    else if(x==1&&y==2){
      if(!tb_atas)motor[0].ki+=tambahan;
      if(!tb_bawah)motor[0].ki-=tambahan;    
    }
    else if(x==2&&y==2){
      if(!tb_atas)motor[1].ki+=tambahan;
      if(!tb_bawah)motor[1].ki-=tambahan;    
    }
    else if(x==1&&y==3){
      if(!tb_atas)motor[0].kd+=tambahan;
      if(!tb_bawah)motor[0].kd-=tambahan;    
    }
    else if(x==2&&y==3){
      if(!tb_atas)motor[1].kd+=tambahan;
      if(!tb_bawah)motor[1].kd-=tambahan;    
    }
    lcd.write(byte(0));
    if(!tb_back){
      while(!tb_back){}  
      clear_screen();lmp_off;
      saveToEeprom(1);
      delay(500);lmp_on;break;
    }
    delay(150);
  }
}

void menu_sensor(){
  byte x=1, y=1, prx[4];
awal:
  clear_screen();
  while(1){
    if(!tb_atas)if(--y<1)y=1;
    if(!tb_bawah)if(++y>3)y=3;
    lcd.setCursor(0,0);lcd.print(" ===== SENSOR ===== ");
    lcd.setCursor(0,1);lcd.print(" Garis     ");
    lcd.setCursor(0,2);lcd.print(" Proximity ");
    lcd.setCursor(0,3);lcd.print(" Encoder      ");
    lcd.setCursor(0,y);
    if(y==1){
      if(!tb_enter){
        while(!tb_enter){};
        goto garis;  
      }  
    }
    else if(y==2){
      if(!tb_enter){
        while(!tb_enter){};
        goto proximity;  
      }  
    }
    else if(y==3){
      if(!tb_enter){
        while(!tb_enter){};
        goto encoder;  
      }
    }
    if(!tb_back){
      while(!tb_back){}
      goto selesai;  
    }
    lcd.write(byte(0));
    delay(200);
  }
scan:
  clear_screen();
  while(1){
    scansensor();
    lcd.setCursor(2,0);lcd.print("A2");
    lcd.setCursor(2,3);lcd.print("A0");
    lcd.setCursor(16,0);lcd.print("A7");
    lcd.setCursor(16,3);lcd.print("A6");
    if(!tb_back){
      while(!tb_back){}
      goto awal;  
    }
  }
garis:
  y=1;
  clear_screen();
  while(1){
    lcd.setCursor(0,0);lcd.print(" ====== GARIS ===== ");
    lcd.setCursor(0,1);lcd.print(" Cek       ");
    lcd.setCursor(0,2);lcd.print(" Kalibrasi ");
    lcd.setCursor(0,3);lcd.print(" Scan      ");
    lcd.setCursor(0,y);
    if(!tb_atas)if(--y<1)y=1;
    if(!tb_bawah)if(++y>3)y=3;
    if(y==1){
      if(!tb_enter){
        while(!tb_enter){};
        cek_sensor();  
      }  
    }
    else if(y==2){
      if(!tb_enter){
        while(!tb_enter){};
        kalibrasi();  
      }  
    }
    else if(y==3){
      if(!tb_enter){
        while(!tb_enter){};
        goto scan;  
      } 
    }
    if(!tb_back){
      while(!tb_back){}
      goto awal;  
    }
    lcd.write(byte(0));
    delay(200);
  }
proximity:
  y=1;
  for(unsigned char i=0; i < 4; i++){
    prx[i]=0;
  }
  clear_screen();
  while(1){
    lcd.setCursor(0,0);lcd.print(" = CEK PROXIMITY =  ");
    lcd.setCursor(0,1);
    if(prx[0]==0)lcd.print("Kiri : - ");
    else lcd.print("Kiri : Ok");
    lcd.setCursor(10,1);
    if(prx[1]==0)lcd.print("Kanan : - ");
    else lcd.print("Kanan : Ok");
    lcd.setCursor(0,2);
    if(prx[2]==0)lcd.print("Kiri1: - ");
    else lcd.print("Kiri1: Ok");
    lcd.setCursor(10,2);
    if(prx[3]==0)lcd.print("Kanan1: - ");
    else lcd.print("Kanan1: Ok");
    if(update_enc==true){
      update_encoder();
      update_enc==false;  
    }
    if(!tb_back){
      while(!tb_back){};
      clear_screen();
      goto awal;  
    }
  }
encoder:
  clear_screen();
  enc_lock=false;
  while(1){
    sprintf(buff,"Kiri  :%04d  ",enc[0]);lcd.setCursor(0,0);lcd.print(buff);
    sprintf(buff,"Tengah:%04d  ",enc[1]);lcd.setCursor(0,1);lcd.print(buff);
    sprintf(buff,"Kanan :%04d  ",enc[2]);lcd.setCursor(0,2);lcd.print(buff);
    lcd.setCursor(0,3);lcd.write(byte(3));lcd.write(byte(5));lcd.write(byte(0));lcd.print(" To Reset Data ");
    if(!tb_kiri){
      while(!tb_kiri){};
      reset_encoder(2);
    }
    if(!tb_bawah){
      while(!tb_bawah){};
      reset_encoder(3);
    }
    if(!tb_kanan){
      while(!tb_kanan){};
      reset_encoder(4);
    }
    if(!tb_back){
      while(!tb_back){}
      enc_lock=true;
      clear_screen();goto awal;
    }
    if(update_enc==true){
      update_encoder();
      update_enc==false;  
    } 
    ros_routine(); 
  }
selesai:
  clear_screen();
  lmp_off;
  delay(500);
  lmp_on;
}

void menu_kicker(){
  byte y, type, drate;
awal:
  y=1;type=0;
  clear_screen();  
  while(1){
    if(!tb_atas)if(--y<1)y=1;
    if(!tb_bawah)if(++y>2)y=2;
    lcd.setCursor(0,0);lcd.print(" ======KICKER====== ");
    lcd.setCursor(0,1);lcd.print(" LAMBUNG            ");
    lcd.setCursor(0,2);lcd.print(" DATAR              ");
    lcd.setCursor(0,y);
    if(y==1){
      if(!tb_enter){
        while(!tb_enter){}
        goto lambung;  
      }  
    }
    else if(y==2){
      if(!tb_enter){
        while(!tb_enter){}
        goto datar;  
      }
    }
    if(!tb_back){
      while(!tb_back){};
      goto selesai;
    }
    lcd.write(byte(0));
    delay(150);
  }
lambung:
    y=1;type=0;
    clear_screen();
    transmitDataDribbler('S',0,0);
  while(1){
    if(!tb_atas)if(--y<1)y=1;
    if(!tb_bawah)if(++y>3)y=3;
    lcd.setCursor(0,0);lcd.print("Lambung:");
    sprintf(buff," [1]: %03d", delay_rate[0]);  
    lcd.setCursor(0,1);lcd.print(buff);lcd.print(" ms");
    sprintf(buff," [2]: %03d", delay_rate[1]);  
    lcd.setCursor(0,2);lcd.print(buff);lcd.print(" ms");
    lcd.setCursor(0,3);lcd.print(" Test: ");
    if(type==0)lcd.print("[1] ");
    else lcd.print("[2] ");
    lcd.setCursor(0,y);
    if(y==1){
      if(!tb_kanan)delay_rate[0]++;
      if(!tb_kiri)delay_rate[0]--;  
    }
    else if(y==2){
      if(!tb_kanan)delay_rate[1]++;
      if(!tb_kiri)delay_rate[1]--;  
    }
    else if(y==3){
      if(!tb_kanan)type=1;
      if(!tb_kiri)type=0;
      if(type==0)drate=delay_rate[0];
      else drate=delay_rate[1];
      if(!tb_enter){
        while(!tb_enter){}
        kick_ball(drate,0);
        clear_screen();
      }  
    }
    if(!tb_back){
      while(!tb_back){};
      clear_screen();lmp_off;
      delay(250);lmp_on;goto awal;
    }
    lcd.write(byte(0));
    delay(150);  
  }
datar:
  clear_screen();
  y=1;type=0;
  transmitDataDribbler('S',0,1);
  while(1){
    if(!tb_atas)if(--y<1)y=1;
    if(!tb_bawah)if(++y>3)y=3;
    lcd.setCursor(0,0);lcd.print("Datar:");
    sprintf(buff," [1]: %03d", delay_rate[2]);  
    lcd.setCursor(0,1);lcd.print(buff);lcd.print(" ms");
    sprintf(buff," [2]: %03d", delay_rate[3]);  
    lcd.setCursor(0,2);lcd.print(buff);lcd.print(" ms");
    lcd.setCursor(0,3);lcd.print(" Test: ");
    if(type==0)lcd.print("[1] ");
    else lcd.print("[2] ");
    lcd.setCursor(0,y);
    if(y==1){
      if(!tb_kanan)delay_rate[2]++;
      if(!tb_kiri)delay_rate[2]--;  
    }
    else if(y==2){
      if(!tb_kanan)delay_rate[3]++;
      if(!tb_kiri)delay_rate[3]--;  
    }
    else if(y==3){
      if(!tb_kanan)type=1;
      if(!tb_kiri)type=0;
      if(type==0)drate=delay_rate[2];
      else drate=delay_rate[3];
      if(!tb_enter){
        while(!tb_enter){}
        kick_ball(drate,1);
        clear_screen();
      }  
    }
    if(!tb_back){
      while(!tb_back){};
      clear_screen();lmp_off;
      delay(250);lmp_on;goto awal;  
    }
    lcd.write(byte(0));
    delay(150);  
  }
selesai:
  clear_screen();lmp_off;
  saveToEeprom(2);delay(500);
  lmp_on;
}

void menu_camera(){
  byte x=1, y=1;
awal:
  clear_screen();
  while(1){
    if(!tb_enter)if(++y>3)y=1;
    if(!tb_kanan)x=2;
    if(!tb_kiri)x=1;
    lcd.setCursor(0,0);lcd.print(" ======CAMERA====== ");
    sprintf(buff," Xmin:%03d  Xmax:%03d ",spoint[0],spoint[1]);
    lcd.setCursor(0,1);lcd.print(buff);
    sprintf(buff," Ybal:%03d  Ylim:%03d ",ylim[0],ylim[1]);
    lcd.setCursor(0,2);lcd.print(buff); 
    lcd.setCursor(0,3);lcd.print(" Scan Data ");lcd.write(byte(5));
    if(x==1)lcd.setCursor(0,y);
    else lcd.setCursor(10,y);
    if(x==1&&y==1){
      if(!tb_atas)spoint[0]++;
      if(!tb_bawah)spoint[0]--;  
    }
    else if(x==1&&y==2){
      if(!tb_atas)ylim[0]++;
      if(!tb_bawah)ylim[0]--;  
    }
    else if(x==2&&y==1){
      if(!tb_atas)spoint[1]++;
      if(!tb_bawah)spoint[1]--;  
    }
    else if(x==2&&y==2){
      if(!tb_atas)ylim[1]++;
      if(!tb_bawah)ylim[1]--;
    }
    else if((x==1||x==2)&&y==3){
      if(!tb_bawah)goto scan;
      lcd.setCursor(0,3);  
    }
    lcd.write(byte(0));
    if(!tb_back){
      while(!tb_back){};
      goto selesai;
    }
    delay(150);
  }
scan:
  clear_screen();
  while(1){
    lcd.setCursor(0,0);lcd.print("Posisi Bola: ");
    ros_routine();
    if(!tb_back){
      while(!tb_back){};
      goto awal;
    }
    delay(10);
  }
selesai:  
  clear_screen();lmp_off;
  saveToEeprom(3);delay(500);lmp_on;
}

void menu_speed(){
  byte a=1, b=1, ena=0;
  int spwm=1020;
  int rpm[3]={0,0,0};
  float spdss[3]={0,0,0};
  clear_screen();
  while(1){
    if(!tb_enter){
      while(!tb_enter){};
      if(++b>3)b=1;
      a=1;clear_screen();
    }
    if(!tb_kanan){
      if(b==1){if(++a>7)a=7;}
      else if(b==2){if(++a>6)a=6;}
    }
    if(!tb_kiri)if(--a<1)a=1;
    if(b==1){
      sprintf(buff," Smin:%03d  Smax:%03d ",spd_dep[0],spd_dep[1]);
      lcd.setCursor(0,0);lcd.print(buff);
      sprintf(buff,"T-Rate:    [1]:%03d ", throtle[0]);
      lcd.setCursor(0,1);lcd.print(buff);
      sprintf(buff," [2]:%03d   [3]:%03d ",throtle[1],throtle[2]);
      lcd.setCursor(0,2);lcd.print(buff);
      sprintf(buff," [4]:%03d   [5]:%03d ",throtle[3],throtle[4]);
      lcd.setCursor(0,3);lcd.print(buff);
      if(a==1){  
        lcd.setCursor(0,0);
        if(!tb_atas)spd_dep[0]+=5;
        if(!tb_bawah)spd_dep[0]-=5;
      }
      else if(a==2){
        lcd.setCursor(10,0);
        if(!tb_atas)spd_dep[1]+=5;
        if(!tb_bawah)spd_dep[1]-=5;
      }
      else if(a==3){
        lcd.setCursor(10,1);  
        if(!tb_atas)throtle[0]+=5;
        if(!tb_bawah)throtle[0]-=5;
      }
      else if(a==4){
        lcd.setCursor(0,2);  
        if(!tb_atas)throtle[1]+=5;
        if(!tb_bawah)throtle[1]-=5;
      }
      else if(a==5){
        lcd.setCursor(10,2);  
        if(!tb_atas)throtle[2]+=5;
        if(!tb_bawah)throtle[2]-=5;
      }
      else if(a==6){
        lcd.setCursor(0,3);  
        if(!tb_atas)throtle[3]+=5;
        if(!tb_bawah)throtle[3]-=5;
      }
      else if(a==7){
        lcd.setCursor(10,3);  
        if(!tb_atas)throtle[4]+=5;
        if(!tb_bawah)throtle[4]-=5;
      }
      lcd.write(byte(0));
      delay(150);
    }
    else if(b==2){
      lcd.setCursor(0,0);lcd.print("Dribbler Speed  ");
      sprintf(buff," F: %03d  L:%03d", dribble_spd[0],dribble_spd[3]);
      lcd.setCursor(0,1);lcd.print(buff);
      sprintf(buff," s: %03d  R:%03d", dribble_spd[1],dribble_spd[4]);
      lcd.setCursor(0,2);lcd.print(buff);
      sprintf(buff," B: %03d", dribble_spd[2]);
      lcd.setCursor(0,3);lcd.print(buff);
      if(a<4)lcd.setCursor(0,a);
      else {
        if(a==4)lcd.setCursor(8,1);
        else lcd.setCursor(8,2);
      }
      if(a==1){  
        if(!tb_atas)dribble_spd[0]+=5;
        if(!tb_bawah)dribble_spd[0]-=5;
      }
      else if(a==2){
        if(!tb_atas)dribble_spd[1]+=5;
        if(!tb_bawah)dribble_spd[1]-=5;
      }
      else if(a==3){
        if(!tb_atas)dribble_spd[2]+=5;
        if(!tb_bawah)dribble_spd[2]-=5;
      }
      else if(a==4){
        if(!tb_atas)dribble_spd[3]+=5;
        if(!tb_bawah)dribble_spd[3]-=5;
      }
      else if(a==5){
        if(!tb_atas)dribble_spd[4]+=5;
        if(!tb_bawah)dribble_spd[4]-=5;
      }
      lcd.write(byte(0));
      delay(150);
    }
    else if(b==3){
      enc_lock=false;
      sprintf(buff,"pwm : %04d    %d ",spwm,ena);lcd.setCursor(0,0);lcd.print(buff);
      sprintf(buff,"Lrpm: %04d %04d",rpm[0],abs(enc[0]));lcd.setCursor(0,1);lcd.print(buff);
      sprintf(buff,"Mrpm: %04d %04d",rpm[1],abs(enc[1]));lcd.setCursor(0,2);lcd.print(buff);
      sprintf(buff,"Rrpm: %04d %04d",rpm[2],abs(enc[2]));lcd.setCursor(0,3);lcd.print(buff);
      if(!tb_kanan){
        while(!tb_kanan){}
        if(ena==0){
          transmitData(spwm,spwm,spwm);
          motor_enable();  
          ena=1;  
        }
        else {
          transmitData(0,0,0);
          motor_disable();  
          ena=0;  
        }
      }
      if(!tb_atas){
        spwm+=5;
        if(spwm>1023)spwm=1023;
        transmitData(spwm,spwm,spwm);
        delay(150);
      }
      if(!tb_bawah){
        spwm-=5;
        if(spwm<0)spwm=0;
        transmitData(spwm,spwm,spwm);
        delay(150);
      }
      if(update_enc==true){
          update_encoder();
          //compute rpm
          spdss[0]=enc[0]*0.010989*599.99;
          spdss[1]=enc[1]*0.010989*599.99;
          spdss[2]=enc[2]*0.010989*599.99;
          rpm[0]=spdss[0];
          rpm[1]=spdss[1];
          rpm[2]=spdss[2];
          reset_encoder(2);
          reset_encoder(3);
          reset_encoder(4);
          update_enc=false;  
      }
      ros_routine();
    }
    if(!tb_back){
      while(!tb_back){};
      enc_lock=true;transmitData(0,0,0);
      motor_disable();    
      clear_screen();lmp_off;
      saveToEeprom(4);delay(500);
      lmp_on;break;  
    }
  }  
}

void menu_motor(){
  byte x=1, y=1, mot=1, mot1=1, test=0;
  char mov='S', mov1='B';
  String Movem="Stop";
awal:
  y=1;
  clear_screen();
  while(1){
    if(!tb_atas)y=1;
    if(!tb_bawah)y=2;
    lcd.setCursor(0,0);lcd.print("Setting Motor:");
    lcd.setCursor(0,1);lcd.print(" Check     "); 
    lcd.setCursor(0,2);lcd.print(" Kalibrasi ");
    lcd.setCursor(0,y);
    if(y==1){
      if(!tb_enter){
        while(!tb_enter){};
        goto check;
      }
    }
    else {
      if(!tb_enter){
        while(!tb_enter){};
        goto kalibrasi;
      }  
    }
    if(!tb_back){
      while(!tb_back){};
      lmp_on;goto selesai;  
    }
    lcd.write(byte(0));
    delay(150);
  }
check:
  y=1;clear_screen();
  enc_lock=true;
  while(1){
    if(!tb_atas){y=1;test=0;}
    if(!tb_bawah){y=2;test=0;}
    lcd.setCursor(0,0);lcd.print("Gerakan : ");
    lcd.setCursor(0,1);lcd.print(" ");lcd.print(Movem); 
    lcd.setCursor(0,2);lcd.print("Dribbler: ");
    lcd.setCursor(0,3);lcd.print(" ");lcd.print((char)(mov1)); 
    switch(mot){
      case 1: mov = 'S';Movem="Stop  ";transmitData(0,0,0);break;
      case 2: mov = 'F';Movem="Maju  ";transmitData(-150,0,150);break;
      case 3: mov = 'B';Movem="Mundur";transmitData(150,0,-150);break;
      case 4: mov = 'R';Movem="Kanan ";transmitData(-200,400,-200);break;
      case 5: mov = 'L';Movem="Kiri  ";transmitData(200,-400,200);break;
    }
    switch(mot1){
      case 1: mov1='S';break;
      case 2: mov1='s';break;
      case 3: mov1='B';break;
      case 4: mov1='F';break;
      case 5: mov1='L';break;
      case 6: mov1='R';break;
    }
    if(y==1){
      lcd.setCursor(0,1);
      if(!tb_kanan){
        if(++mot>5)mot=5;
      }
      if(!tb_kiri){
        if(--mot<1)mot=1;
      }
      if(!tb_enter){
        while(!tb_enter){};
        x=1;
        if(test==0)test=1; 
        else test=0; 
      }
      //if(x==1){
        //x=2;
      //}
      if(test==1){
        //if(x==1){
          motor_enable();x=2;
        //}
      }
      else {
        //if(x==1){
          motor_disable();x=2;
        //}
      }
      //motor_disable();
      //motor_enable();
    }
    else {
      lcd.setCursor(0,3);
      if(!tb_kanan)if(++mot1>6)mot1=6;
      if(!tb_kiri)if(--mot1<1)mot1=1;
      if(!tb_enter){
        while(!tb_enter){};
        if(test==0)test=1; 
        else test=0; 
      }
      if(test==1){
        if(mot1==1)transmitDataDribbler('S',0,0);
        else if(mot1==2)transmitDataDribbler('B',dribble_spd[1],0);//B
        else if(mot1==3)transmitDataDribbler(mov1,dribble_spd[2],0);//B
        else if(mot1==4)transmitDataDribbler(mov1,dribble_spd[0],0);//F
        else if(mot1==5)transmitDataDribbler(mov1,dribble_spd[3],0);//L
        else if(mot1==6)transmitDataDribbler(mov1,dribble_spd[4],0);//R
      }
      else transmitDataDribbler('S',0,0);
    }
    if(!tb_back){
      while(!tb_back){};
      transmitData(0,0,0);
      transmitDataDribbler('S',0,0);
      motor_disable();
      clear_screen();lmp_off;
      delay(250);lmp_on;goto awal;  
    }
    lcd.write(byte(0));
    lcd.setCursor(19,0);lcd.print(test);
    ros_routine();
    delay(150);
  }  
kalibrasi:
  y=1;clear_screen();
  while(1){
    if(!tb_atas)y=1;
    if(!tb_bawah)y=3;
    if(!tb_enter)if(++x>2)x=1;
    lcd.setCursor(0,0);lcd.print("Maju  : ");
    sprintf(buff," Ki:%03d  Ka:%03d ",additional[0],additional[1]);
    lcd.setCursor(0,1);lcd.print(buff);
    lcd.setCursor(0,2);lcd.print("Mundur : ");
    sprintf(buff," ki:%03d  Ka:%03d ",additional[2],additional[3]);
    lcd.setCursor(0,3);lcd.print(buff);
    if(x==1)lcd.setCursor(0,y);
    else lcd.setCursor(8,y);
    if(y==1&&x==1){
      if(!tb_kanan)additional[0]+=1;
      if(!tb_kiri)additional[0]-=1;
    }
    else if(y==1&&x==2){
      if(!tb_kanan)additional[1]+=1;
      if(!tb_kiri)additional[1]-=1;
    }
    else if(y==3&&x==1){
      if(!tb_kanan)additional[2]+=1;
      if(!tb_kiri)additional[2]-=1;
    }
    else if(y==3&&x==2){
      if(!tb_kanan)additional[3]+=1;
      if(!tb_kiri)additional[3]-=1;
    }    
    if(!tb_back){
      while(!tb_back){};
      clear_screen();lmp_off;saveToEeprom(5);
      delay(250);lmp_on;goto awal;  
    }
    lcd.write(byte(0));
    delay(150);
  }
selesai:
  clear_screen();lmp_off;
  delay(500);lmp_on;
}

void kick_ball(unsigned char del, unsigned char target){//0:lambung, 1:datar
  //transmitDataDribbler('S',0,target);
  //delay(200);
  digitalWrite(trigger_pin,HIGH);
  delay(del);
  digitalWrite(trigger_pin,LOW);
  delay(500);
}

void cari_garis_screen(){
  sprintf(buff,"[%02d]-[%d ] ",sendata, er);
  lcd.setCursor(9,2);lcd.print(buff);
  lcd.setCursor(5,1);
  lcd.print("Cari Garis");  
}

#endif
