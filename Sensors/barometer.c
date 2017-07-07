/*
*   This file includes the description of the function to read and calculate the
*   Barometric Pressure, Temprature, and Altitude from the MS5611 Barometric Pressure & Temperature Sensor
*   which is made by TE. 
*   We are using the chip on a breakout board that we got from Amazon called GY-63
*/

#include "debug_Serial.h"
#include "debug_Serial.c"
#include "barometer.h"
#include "twi.h"
#include "project.h"
#include "SensorVal.h"
#include "stdlib.h"
#include "math.h"
#include <stdint.h>


static uint16_t fc[6];
static uint8_t ct;
static uint8_t uosr;
static int32_t TEMP2;
static int64_t OFF2, SENS2;
static uint8_t _COMPENSATION = false;

static void baro_sendCommand(uint8_t cmd); //used to send one through I2C
static void baro_reset(void);              //used to send the reset command
static void baro_readPROM(void);           //used to read the PROM

static uint16_t baro_readRegister16(uint8_t reg); //reads and return 2 bytes of data
static uint32_t baro_readRegister24(uint8_t reg); //reads and return 3 bytes of data

static void baro_sendCommand(uint8_t cmd)
{
    //to write, you want to write:  (1) The address you want to write to
    //                              (2) The cmd you wish to write to the register

    uint8_t Write_Buf[1] = {0};
    Write_Buf[0] = (uint8_t)cmd;

    I2C_MasterWriteBuf(MS5611_ADDRESS, (uint8 *)Write_Buf, sizeof(Write_Buf), I2C_MODE_COMPLETE_XFER);
    while ((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) == 0)
    {
    }

    return; //return 1 if everything was successful
}

#if SPECIFY_OSR_AT_RUNTIME

void baro_initialize(ms5611_osr_t osr)
{
    serial_println("Initializing Barometeric Pressure and Temperature sensor:");
    while (!baro_begin(osr))
    {
        serial_println("//=======Could not find a valid Barometric sensor, check wiring!=======//");
    }
    baro_setCompensation(true);

    //read the reference Pressure to enable us to calculate the relative Altitude
    reference_pressure = baro_readPressure();

    //read the initial raw Temperature and Pressure just in case we later want some reference
    initial_raw_temperature = baro_readRawTemperature();
    initial_raw_pressure = baro_readRawPressure();

    //read the real compasated Temperature and Pressure just in case we want it later
    initial_real_temprature = baro_readTemperature();
    initial_real_pressure = baro_readPressure();

    //Calculate and store the initial absolute and relative altitude
    //The initial relative altitude should be less than 10, MAX
    initial_absolute_altitude = baro_getAbsoluteAltitude(reference_pressure);
    
    int32_t check_pressure = baro_readPressure();
    initial_relative_altitude = baro_getRelativeAltitude(check_pressure, reference_pressure);
    
    if (initial_relative_altitude > 10)
    {
        serial_println("There must be an issue with the Altitude calculation or pressure");
        serial_println("check code or restart quadcopter and don't touch.");
    }

//if the macro DEBUG is true (value is not 0)
#if BAROMETER_DEBUG
    Serial_print(" rawTemp = ");
    serial_printInt(initial_raw_temperature); //print an integer
    Serial_print(", realTemp = ");
    serial_printDouble(initial_raw_temperature); //print a double
    Serial_println(" *C");

    Serial_print(" rawPressure = ");
    serial_printInt(rawPressure); //print an integer
    Serial_print(", realPressure = ");
    serial_printInt(realPressure); //print an integer
    Serial_println(" Pa");

    Serial_print(" absoluteAltitude = ");
    serial_printDouble(absoluteAltitude); //print a double
    Serial_print(" m, relativeAltitude = ");
    serial_printDouble(relativeAltitude); //print a double
    Serial_println(" m");
#endif
}
    
uint8_t baro_begin(ms5611_osr_t osr)
{
    Wire_begin();
    baro_reset();
    baro_setOversampling(osr);
    CyDelay(100);
    baro_readPROM();
    return true;
}

#else
    
void baro_initialize(void)
{
    serial_println("Initializing Barometeric Pressure and Temperature sensor:");
    while (!baro_begin())
    {
        serial_println("//=======Could not find a valid Barometric sensor, check wiring!=======//");
    }

    //read the reference Pressure to enable us to calculate the relative Altitude
    reference_pressure = baro_readPressure();

    //read the initial raw Temperature and Pressure just in case we later want some reference
    initial_raw_temperature = baro_readRawTemperature();
    initial_raw_pressure = baro_readRawPressure();

    //read the real compasated Temperature and Pressure just in case we want it later
    initial_real_temprature = baro_readTemperature();
    initial_real_pressure = baro_readPressure();

    //Calculate and store the initial absolute and relative altitude
    //The initial relative altitude should be less than 10, MAX
    initial_absolute_altitude = baro_getAbsoluteAltitude(reference_pressure);
    
    int32_t check_pressure = baro_readPressure();
    initial_relative_altitude = baro_getRelativeAltitude(check_pressure, reference_pressure);
    
    if (initial_relative_altitude > 10)
    {
        serial_println("There must be an issue with the Altitude calculation or pressure");
        serial_println("check code or restart quadcopter and don't touch.");
    }

//if the macro DEBUG is true (value is not 0)
#if BAROMETER_DEBUG
    serial_print(" rawTemp = ");
    serial_printInt(initial_raw_temperature); //print an integer
    serial_print(", realTemp = ");
    serial_printDouble(initial_raw_temperature); //print a double
    serial_println(" *C");

    serial_print(" rawPressure = ");
    serial_printInt(initial_raw_pressure); //print an integer
    serial_print(", realPressure = ");
    serial_printInt(initial_real_pressure); //print an integer
    serial_println(" Pa");

    serial_print(" absoluteAltitude = ");
    serial_printDouble(initial_absolute_altitude); //print a double
    serial_print(" m, relativeAltitude = ");
    serial_printDouble(initial_relative_altitude); //print a double
    serial_println(" m");
#endif
}

uint8_t baro_begin()
{
    //This is like a default setting
    ms5611_osr_t osr = MS5611_STANDARD;
    Wire_begin();
    baro_reset();
    baro_setOversampling(osr);
    CyDelay(100);
    baro_readPROM();
    return true;
}

#endif

// Set oversampling value
void baro_setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
    case MS5611_ULTRA_LOW_POWER:
        ct = 1;
        break;
    case MS5611_LOW_POWER:
        ct = 2;
        break;
    case MS5611_STANDARD:
        ct = 3;
        break;
    case MS5611_HIGH_RES:
        ct = 5;
        break;
    case MS5611_ULTRA_HIGH_RES:
        ct = 10;
        break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t baro_getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void baro_setCompensation(uint8_t val){
    _COMPENSATION = val;
}


static void baro_reset(void)
{
    baro_sendCommand(MS5611_CMD_RESET);
}

static void baro_readPROM(void)
{
    uint8_t offset = 0;
    for (offset = 0; offset < 6; offset++)
    {
        fc[offset] = baro_readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t baro_readRawTemperature(void)
{
    baro_sendCommand(MS5611_CMD_CONV_D2 + uosr);
    CyDelay(ct);
    return baro_readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t baro_readRawPressure(void)
{
    baro_sendCommand(MS5611_CMD_CONV_D2 + uosr);
    CyDelay(ct);
    return baro_readRegister24(MS5611_CMD_ADC_READ);
}

int32_t baro_readPressure(void)
{
    uint32_t D1 = baro_readRawPressure();

    uint32_t D2 = baro_readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (_COMPENSATION)
    {
        int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;

        OFF2 = 0;
        SENS2 = 0;

        if (TEMP < 2000)
        {
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
        }

        if (TEMP < -1500)
        {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }

        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double baro_readTemperature(void)
{
    uint32_t D2 = baro_readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (_COMPENSATION)
    {
        if (TEMP < 2000)
        {
            TEMP2 = (dT * dT) / (2 << 30);
        }
    }

    TEMP = TEMP - TEMP2;

    return ((double)TEMP / 100);
}

// Calculate altitude from Pressure & Sea level pressure
double baro_getAbsoluteAltitude(int32_t pressure)
{
    double seaLevelPressure = 101325;
    return (44330.0 * (1.0 - pow(((double)pressure / (double)seaLevelPressure), 0.1902949)));
}

// Calculate altitude from Pressure & initial pressure
double baro_getRelativeAltitude(int32_t pressure, int32_t initialPressure)
{
    return (44330.0 * (1.0 - pow((double)pressure / (double)initialPressure, 0.1902949)));
}

// Calculate sea level from Pressure given on specific altitude
double baro_getSeaLevel(int32_t pressure, int32_t altitude)
{
    return ((double)pressure / pow((1.0) - ((double)altitude / (44330.0)), (5.255)));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t baro_readRegister16(uint8_t reg)
{
    uint16_t value;
    uint8_t Read_Buf[2] = {0}; //create an array that will store the data read

    // First command sets up the system into the correct read mode (ex:PROM).
    baro_sendCommand(reg);
    //The second part gets the data from the system.
    I2C_MasterReadBuf(MS5611_ADDRESS, (uint8 *)Read_Buf, sizeof(Read_Buf), I2C_MODE_COMPLETE_XFER);
    while ((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT) == 0)
    {
    } //wait till Master status indicates read is complete

    uint8_t vha = Read_Buf[0];
    uint8_t vla = Read_Buf[1];

    value = vha << 8 | vla;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t baro_readRegister24(uint8_t reg)
{
    uint32_t value;
    uint8_t Read_Buf[3] = {0}; //create an array that will store the data read

    // First command sets up the system into the correct read mode (ex:ADC Conversion read).
    baro_sendCommand(reg);
    //The second part gets the data from the system.
    I2C_MasterReadBuf(MS5611_ADDRESS, (uint8 *)Read_Buf, sizeof(Read_Buf), I2C_MODE_COMPLETE_XFER);
    while ((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT) == 0)
    {
    } //wait till Master status indicates read is complete

    uint8_t vxa = Read_Buf[0];
    uint8_t vha = Read_Buf[1];
    uint8_t vla = Read_Buf[2];

    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;
    //serial_printIntln(value);
    return value;
}

void baro_update()
{
    //update the Barometric Pressure and Temperature Sensor values
    baro.rawTemp = baro_readRawTemperature();
    baro.rawPress = baro_readRawPressure();
    baro.temperature = baro_readTemperature();
    baro.pressure = baro_readPressure();

    baro.absAlt = baro_getAbsoluteAltitude(baro.pressure);
    baro.relAlt = baro_getRelativeAltitude((baro.pressure), (reference_pressure));

//if the macro DEBUG is true (value is not 0)
#if BAROMETER_DEBUG
    serial_print(" rawTemp = ");
    serial_printInt(baro_getRawTemp()); //print an integer
    serial_print(", realTemp = ");
    serial_printDouble(baro_getTemp()); //print a double
    serial_print(" *C  |  ");

    serial_print(" rawPressure = ");
    serial_printInt(baro_getRawPress()); //print an integer
    serial_print(", realPressure = ");
    serial_printInt(baro_getPress()); //print an integer
    serial_print(" Pa  |  ");

    serial_print(" absoluteAltitude = ");
    serial_printFloat(baro_getAbsAlt()); //print a double
    serial_print(" m, relativeAltitude = ");
    serial_printFloat(baro_getRelAlt()); //print a double
    serial_println(" m");
    
#endif
}

/*  These functions are provided to allow the user to extract
 *  single pieces of data from the sensor readings safely
 *  instead of just calling the sensor variables directly
*/
uint32_t baro_getRawTemp(void){
    return (baro.rawTemp);
}

uint32_t baro_getRawPress(void){
    return (baro.rawPress);
}

double baro_getTemp(void){
    return (baro.temperature);
}

int32_t baro_getPress(void){
    return (baro.pressure);
}

double baro_getAbsAlt(void){
    return (baro.absAlt);
}

double baro_getRelAlt(void){
    return baro.relAlt;
}


