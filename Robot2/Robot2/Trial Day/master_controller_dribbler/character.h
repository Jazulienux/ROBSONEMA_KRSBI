#ifndef CHARACTER_H
#define CHARACTER_H

extern volatile byte cursR[8] = {
  0b00000,
  0b11000,
  0b11100,
  0b11110,
  0b11100,
  0b11000,
  0b00000,
  0b00000
};

extern volatile byte cursL[8] = {
  0b00000,
  0b00011,
  0b00111,
  0b01111,
  0b00111,
  0b00011,
  0b00000,
  0b00000
};

extern volatile byte cursU[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b11111,
  0b00000,
  0b00000
};

extern volatile byte cursD[8] = {
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000,
  0b00000
};

extern volatile byte back[8] = {
  0b00000,
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00000
};

extern volatile byte enter[8] = {
  0b10000,
  0b10000,
  0b10100,
  0b10010,
  0b11111,
  0b00010,
  0b00100,
  0b00000
};

#endif
