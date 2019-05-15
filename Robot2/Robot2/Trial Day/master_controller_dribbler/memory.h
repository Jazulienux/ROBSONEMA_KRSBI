#ifndef MEMORY_H
#define MEMORY_H

void readFromEeprom();
void saveToEeprom(byte type);
void writeEeprom16bit(int adr1, int adr2, int value);
int readEeprom16bit(int adr1, int adr2);

#endif
