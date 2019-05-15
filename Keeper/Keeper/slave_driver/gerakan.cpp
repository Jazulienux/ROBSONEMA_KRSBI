#ifndef gerakan_cpp
#define gerakan_cpp

#include <Arduino.h>
#include "gerakan.h"
#include "driver.h"
//2 = kiri, 3 = belakang, 4 = kanan
volatile byte address = 3;

void run_program(){
  switch(mov){
    case 0: dir=0; break;
    case 1: dir=1; break;
    case 2: dir=2; break;
  }  
  motor(dir, pwmVal);
}

#endif
