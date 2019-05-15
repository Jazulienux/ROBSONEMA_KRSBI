#ifndef DRIVER_H
#define DRIVER_H

extern volatile byte mov, dir;
extern volatile int pwmVal;
extern volatile byte pinen;

void setup_pwm16();
void analogWrite10(unsigned char pin, int pwm);
void motor(byte dirs, int pwm);
void print_data(int var, int val);

#endif
