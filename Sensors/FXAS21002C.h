#ifndef FXAS21002C_H
#define FXAS21002C_H

#include <stdint.h>

#ifndef byte
#define byte uint8_t
#endif

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define FXAS21002C_ADDRESS       (0x21)       // 0100001
    #define FXAS21002C_ID            (0xD7)       // 1101 0111
    #define GYRO_SENSITIVITY_250DPS  (0.0078125F) // Table 35 of datasheet
    #define GYRO_SENSITIVITY_500DPS  (0.015625F)  // ..
    #define GYRO_SENSITIVITY_1000DPS (0.03125F)   // ..
    #define GYRO_SENSITIVITY_2000DPS (0.0625F)    // ..
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                             // DEFAULT    TYPE
      GYRO_REGISTER_STATUS              = 0x00,
      GYRO_REGISTER_OUT_X_MSB           = 0x01,
      GYRO_REGISTER_OUT_X_LSB           = 0x02,
      GYRO_REGISTER_OUT_Y_MSB           = 0x03,
      GYRO_REGISTER_OUT_Y_LSB           = 0x04,
      GYRO_REGISTER_OUT_Z_MSB           = 0x05,
      GYRO_REGISTER_OUT_Z_LSB           = 0x06,
      GYRO_REGISTER_WHO_AM_I            = 0x0C,   // 11010111   r
      GYRO_REGISTER_CTRL_REG0           = 0x0D,   // 00000000   r/w
      GYRO_REGISTER_CTRL_REG1           = 0x13,   // 00000000   r/w
      GYRO_REGISTER_CTRL_REG2           = 0x14,   // 00000000   r/w
    } gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum //double check these values here and up top!!
    {
      GYRO_RANGE_250DPS  = 250,
      GYRO_RANGE_500DPS  = 500,
      GYRO_RANGE_1000DPS = 1000,
      GYRO_RANGE_2000DPS = 2000
    } gyroRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct gyroRawData_s
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } gyroRawData_t;
/*=========================================================================*/

void write8 ( byte reg, byte value );
uint8_t read8 ( byte reg );
uint8_t FXAS21002C_begin (FXAS21002CGyroRange_t rng);
uint8_t FXAS21002C_getData();
void FXAS21002C_readData(FXAS21002CRawData_t * gyro, FXAS21002CRawData_t * RADS);

#endif
