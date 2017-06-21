#ifndef BAROMETER_H
#define BAROMETER_H

#include <stdint.h>
#include <stdlib.h>
/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define MS5611_ADDRESS (0x77) // can be 0x76 if CSB pin is connected to GND
/*=========================================================================*/

#define N_PROM_PARAMS 6

void sendCommand(uint8_t cmd);
uint32_t readnBytes(uint8_t nBytes);

void baro_begin();
uint32_t baro_getRawTemperature();
uint32_t baro_getRawPressure();
int32_t baro_getTemperature();
int32_t baro_getPressure();
void baro_readCalibration();
void baro_getCalibration(uint16_t *C);

#endif