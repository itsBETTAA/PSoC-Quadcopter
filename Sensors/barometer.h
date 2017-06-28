#ifndef BAROMETER_H
#define BAROMETER_H

 #include <stdint.h>
 #include "Serial.h"
    
#define BAROMETER_DEBUG 1
#define SPECIFY_OSR_AT_RUNTIME 0

/*if the SPECIFY_OSR_AT_RUNTIME is 0
Then the default OSR value will be set to one of the following.
The options the user has are:
    MS5611_ULTRA_HIGH_RES
    MS5611_HIGH_RES
    MS5611_STANDARD
    MS5611_LOW_POWER
    MS5611_ULTRA_LOW_POWER

after selecting the perefered one write it down on the following
#define
*/
#define NON_RUNTIME_OSR MS5611_ULTRA_HIGH_RES

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define MS5611_ADDRESS (0x77) // can be 0x76 if CSB pin is connected to GND
/*=========================================================================*/

#define MS5611_CMD_ADC_READ (0x00)
#define MS5611_CMD_RESET (0x1E)
#define MS5611_CMD_CONV_D1 (0x40)
#define MS5611_CMD_CONV_D2 (0x50)
#define MS5611_CMD_READ_PROM (0xA2)

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

typedef enum {
    MS5611_ULTRA_HIGH_RES = 0x08,
    MS5611_HIGH_RES = 0x06,
    MS5611_STANDARD = 0x04,
    MS5611_LOW_POWER = 0x02,
    MS5611_ULTRA_LOW_POWER = 0x00
} ms5611_osr_t;

typedef struct
{
    uint32_t rawTemp;
    uint32_t rawPress;
    double temperature;
    int32_t pressure;

    double absAlt;
    double relAlt;
} barometricData_t;

//if the macro SPECIFY_OSR_AT_RUNTIME is true
#if SPECIFY_OSR_AT_RUNTIME
    //let this format of baro_begin() be used
    void baro_initialize(ms5611_osr_t osr);
    uint8_t baro_begin(ms5611_osr_t osr);
#else
    //else this format 
    void baro_initialize(void);
    uint8_t baro_begin();
#endif

uint32_t baro_readRawTemperature(void);
uint32_t baro_readRawPressure(void);
double baro_readTemperature(void);
int32_t baro_readPressure(void);

double baro_getAbsoluteAltitude(int32_t pressure);
double baro_getRelativeAltitude(int32_t pressure, int32_t initialPressure);
double baro_getSeaLevel(int32_t pressure, int32_t altitude);

void baro_setOversampling(ms5611_osr_t osr);
ms5611_osr_t baro_getOversampling(void);
void baro_setCompensation(uint8_t val);


void baro_update(void);

/*  These functions are provided to allow the user to extract
 *  single pieces of data from the sensor readings safely
 *  instead of just calling the sensor variables directly
*/
uint32_t baro_getRawTemp(void);
uint32_t baro_getRawPress(void);
double baro_getTemp(void);
int32_t baro_getPress(void);
double baro_getAbsAlt(void);
double baro_getRelAlt(void);


#endif