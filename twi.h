#ifndef TWI_H
#define TWI_H

#include <stdint.h>
    
#ifndef byte
#define byte uint8_t
#endif

void Wire_begin();
void Wire_beginTransmission(uint8_t address, uint8_t R_nW);
int Wire_write(uint8_t address, uint8_t* writeBuffer, uint8_t length, uint8_t type);
uint8_t Wire_read(uint8_t address);
uint8_t Wire_requestFrom(uint8_t slaveAddress, uint8_t length, uint8_t *databuffer);
uint8_t Wire_endTransmission();

#endif