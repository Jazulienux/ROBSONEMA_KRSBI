#ifndef controlpid_cpp
#define controlpid_cpp

#include <Arduino.h>
#include "controlpid.h"
#include "interface.h"
#include "roshandler.h"
#include "index.h"
#define ts  0.01

volatile pid motor[2]={{10.5, 0, 20},{0, 0, 0}};
volatile int error=0, error1=0, pwm[3]={0,0,0};
volatile int er=0, cam=0, sign=0;
volatile long lastmillis=0;
volatile byte st=0;
int lasterror=0, zigmaerror=0, deltaerror=0, lasterror1=0, zigmaerror1=0, deltaerror1=0, max_pwm=0;

void compute_pid(){
  int out=0;
  deltaerror = error - lasterror;
  zigmaerror +=error;
  out = (motor[0].kp*error) + (motor[0].ki*zigmaerror)/ts + (motor[0].kd*deltaerror)*ts;//pidbola
  lasterror = error;
  if(millis()-lastmillis>=throtle[2])max_pwm = spd_dep[0];
  else if(millis()-lastmillis>=throtle[1])max_pwm = spd_dep[1];
  else if(millis()-lastmillis>=throtle[0])max_pwm = ((spd_dep[1]-spd_dep[0])/2)+spd_dep[0];
  else max_pwm = spd_dep[0];
  if(out>max_pwm)out = max_pwm;
  pwm[1] = out;
  pwm[0] = 0.5 * out+25;
  pwm[2] = 0.5 * out+25;
}

void compute_pid1(){
  int out=0;
  deltaerror1 = error1 - lasterror1;
  zigmaerror1 += error1;
  out = (motor[1].kp*error1) + (motor[1].ki*zigmaerror1)/ts + (motor[1].kd*deltaerror1)*ts;//pid linefollower
  lasterror1 = error1;
  adds = out;
}

#endif
