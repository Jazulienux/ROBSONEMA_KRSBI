#ifndef I2CDRIVER_H
#define I2CDRIVER_H

#include <Wire.h>

#define KIRI      0X02
#define TENGAH    0X03
#define KANAN     0x04
#define DRIBBLER  0x05

extern volatile byte pinen;
extern volatile int enc[3];
extern volatile bool enc_lock;

void initslave(char motor, unsigned char address);
void transmitData(int lpwm, int bpwm, int rpwm);
void transmitDataDribbler(char mov, byte drbVal, byte target);
void motor_enable();
void motor_disable();
void reset_encoder(unsigned char slave_addres);
void update_encoder();

#endif
