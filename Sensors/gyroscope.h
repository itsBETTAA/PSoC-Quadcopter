#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <stdint.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define FXAS21002C_ADDRESS       (0x21)       // 0100001
    #define FXAS21002C_ID            (0xD7)       // 1101 0111
    #define GYRO_SENSITIVITY_250DPS  (0.0078125F) // Table 35 of datasheet
    #define GYRO_SENSITIVITY_500DPS  (0.015625F)  // ..
    #define GYRO_SENSITIVITY_1000DPS (0.03125F)   // ..
    #define GYRO_SENSITIVITY_2000DPS (0.0625F)    // ..
    /*-----------------------------------------------------------------------
    CONSTANTS
    -----------------------------------------------------------------------*/
    #define SENSORS_DPS_TO_RADS (0.017453293F)      /**< Degrees/s to rad/s multiplier */
    #define true 1
    #define false 0

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
    typedef enum
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

void gyro_write8( uint8_t reg, uint8_t value );
uint8_t gyro_read8 ( uint8_t reg );

uint8_t gyro_begin(gyroRange_t rng);
uint8_t gyro_getData();
void gyro_readData(gyroRawData_t * gyro);

#endif