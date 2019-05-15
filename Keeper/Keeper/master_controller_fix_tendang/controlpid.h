#ifndef CONTROLPID_H
#define CONTROLPID_H

typedef struct{
  float kp;
  float ki;
  float kd;  
}pid;

extern volatile pid motor[2];
extern volatile int error, error1, er, cam, pwm[3], sign;
extern volatile byte st;
extern volatile long lastmillis;

void compute_pid();
void compute_pid1();
void compute_camera();

#endif
