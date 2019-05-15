#ifndef LINESENSOR_H
#define LINESENSOR_H

extern volatile byte pinsensor[4], sens[4], sendata;
extern volatile int sensitive[4], input_check[4];

void scansensor();
void kalibrasi();
void cek_sensor();

#endif
