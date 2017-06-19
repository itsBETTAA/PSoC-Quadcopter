
#ifndef FXOS8700_H
#define FXOS8700_H

#include <stdint.h>
    
#ifndef byte
#define byte uint8_t
#endif

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define FXOS8700_ADDRESS           (0x1F)     // 0011111
    #define FXOS8700_ID                (0xC7)     // 1100 0111
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                             // DEFAULT    TYPE
      FXOS8700_REGISTER_STATUS          = 0x00,
      FXOS8700_REGISTER_OUT_X_MSB       = 0x01,
      FXOS8700_REGISTER_OUT_X_LSB       = 0x02,
      FXOS8700_REGISTER_OUT_Y_MSB       = 0x03,
      FXOS8700_REGISTER_OUT_Y_LSB       = 0x04,
      FXOS8700_REGISTER_OUT_Z_MSB       = 0x05,
      FXOS8700_REGISTER_OUT_Z_LSB       = 0x06,
      FXOS8700_REGISTER_WHO_AM_I        = 0x0D,   // 11000111   r
      FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E,
      FXOS8700_REGISTER_CTRL_REG1       = 0x2A,   // 00000000   r/w
      FXOS8700_REGISTER_CTRL_REG2       = 0x2B,   // 00000000   r/w
      FXOS8700_REGISTER_CTRL_REG3       = 0x2C,   // 00000000   r/w
      FXOS8700_REGISTER_CTRL_REG4       = 0x2D,   // 00000000   r/w
      FXOS8700_REGISTER_CTRL_REG5       = 0x2E,   // 00000000   r/w
      FXOS8700_REGISTER_MSTATUS         = 0x32,
      FXOS8700_REGISTER_MOUT_X_MSB      = 0x33,
      FXOS8700_REGISTER_MOUT_X_LSB      = 0x34,
      FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35,
      FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36,
      FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37,
      FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38,
      FXOS8700_REGISTER_MCTRL_REG1      = 0x5B,   // 00000000   r/w
      FXOS8700_REGISTER_MCTRL_REG2      = 0x5C,   // 00000000   r/w
      FXOS8700_REGISTER_MCTRL_REG3      = 0x5D,   // 00000000   r/w
    }  fxos8700Registers_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      ACCEL_RANGE_2G                    = 0x00,
      ACCEL_RANGE_4G                    = 0x01,
      ACCEL_RANGE_8G                    = 0x02
    } fxos8700AccelRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW ACCEL AND MAG DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } fxos8700RawData_t;
/*=========================================================================*/

void write8 ( byte reg, byte value );
uint8_t  read8( byte reg );
uint8_t FXOS8700_begin (fxos8700AccelRange_t rng);
uint8_t FXOS8700_getData();
void FXOS8700_readData(fxos8700RawData_t * accel, fxos8700RawData_t * mag);



#endif

