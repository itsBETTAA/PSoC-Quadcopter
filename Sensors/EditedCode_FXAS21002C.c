/*
 * This is going to be the remade version of the FXAS21002 library from adafruit
  * to support the PSoC Version of our project
*/
#include "FXAS21002C.h"
#include <twi.h>
#include <project.h>

static FXAS21002CGyroRange_t _range; 

FXAS21002CRawData_t gyro_raw; /* May need to adjust... Supposed to be
reading raw values of last sensor read */ 

#define _write I2C_I2C_WRITE_XFER_MODE 
#define _read I2C_I2C_READ_XFER_MODE 

// Need to correct the below values
#define GYRO_MG_LSB_2G (0.000244F) 
#define GYRO_MG_LSB_4G (0.000488F) 
#define GYR0_MG_LSB_8G (0.000976F) 
#define DPS_TO_RADS (0.0174533F) 

#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */ 
#define SENSORS_GRAVITY_MOON (1.6F)      /**< The moon's gravity in m/s^2 */ 
#define SENSORS_GRAVITY_SUN (275.0F)     /**< The sun's gravity in m/s^2 */ 
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH) 
#define SENSORS_MAGFIELD_EARTH_MAX (60.0F)      /**< Maximum magnetic field on Earth's surface */ 
#define SENSORS_MAGFIELD_EARTH_MIN (30.0F)      /**< Minimum magnetic field on Earth's surface */ 
#define SENSORS_PRESSURE_SEALEVELHPA (1013.25F) /**< Average sea level pressure is 1013.25 hPa */ 
#define SENSORS_DPS_TO_RADS (0.017453293F)      /**< Degrees/s to rad/s multiplier */ 
#define SENSORS_GAUSS_TO_MICROTESLA (100)       /**< Gauss to micro-Tesla multiplier */ 

#define true 1 
#define false 0 



/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void write8(byte reg, byte value)
{
  Wire.beginTransmission(FXAS21002C_ADDRESS);
      uint8_t Write_Buf[2] = {0}
      Write_Buf[0] = (uint8_t)Reg;
      Write_Buf[0] = (uint8_t)value;
  I2C_I2CMasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Write_Buf, 2, I2C_I2C_MODE_COMPLETE_MODE)
  while(I2C_I2CMasterStatus()& I2C_I2C_MSTAT_WR_CMPLT)==0{}
  
  return;
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
 uint8_t read8(uint8_t Reg)
 {

        uint8_t Write_Buf[1];
        Write_Buf[0] = (uint8_t)Reg;

        uint8 Read_Buf[1] = {0};

        I2C_I2CMasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Write_Buf, 1, I2C_I2C_MODE_NO_STOP);
        while(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) == 0){}

        I2C_I2CMasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Read_Buf, 1, I2C_I2C_MODE_REPEAT_START);
        while(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) == 0){}

return Read_Buf[0];
 }
        
/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
uint8_t FXAS21002C_begin(FXAS21002CGyroRange_t rng)
{
  /* Enable I2C */
  Wire.begin();

  /* Set the range the an appropriate value */
  _range = rng;

  /* Clear the raw sensor data */
  gyroraw.x = 0;
  gyroraw.y = 0;
  gyroraw.z = 0;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint32_t id = read8(FXAS21002C_REGISTER_WHO_AM_I);
  // Serial.print("WHO AM I? 0x"); Serial.println(id, HEX);
  if (id != FXAS21002C_ID)
  {
    return false;
  }

  /* Set CTRL_REG1 (0x13)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     6  RESET     Reset device on 1                                   0
     5  ST        Self test enabled on 1                              0
   4:2  DR        Output data rate                                  000
                  000 = 800 Hz
                  001 = 400 Hz
                  010 = 200 Hz
                  011 = 100 Hz
                  100 = 50 Hz
                  101 = 25 Hz
                  110 = 12.5 Hz
                  111 = 12.5 Hz
     1  ACTIVE    Standby(0)/Active(1)                                0
     0  READY     Standby(0)/Ready(1)                                 0

  /* Reset then switch to active mode with 100Hz output */
  write8(FXAS21001C_REGISTER_CTRL_REG1, 0x00);
  write8(FXAS21001C_REGISTER_CTRL_REG1, (1<<6));
  write8(FXAS21001C_REGISTER_CTRL_REG1, 0x0E);
  delay(100); // 60 ms + 1/ODR
  /* ------------------------------------------------------------------ */

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/


  /* Clear the raw data placeholder */
  gyro_raw.x = 0;
  gyro_raw.y = 0;
  gyro_raw.z = 0;

 uint8_t Write_Buf[1];
 Write_Buf[0] = (uint8_t)FXAS21001C_REGISTER_STATUS;

 uint8_t Read_Buf[14] = {0};

 I2C_I2CMasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Write_Buf, 1, I2C_I2C_MODE_NO_STOP);
while(I2C_I2CMasterStatus()& I2C_I2C_MSTAT_WR_CMPLT) == 0{}

I2C_I2CMasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Read_Buf, sizeof(Read_Buf), I2C_I2C_MODE_REPEAT_START);
while(I2C_I2CMasterStatus()& I2C_I2C_MSTAT_RD_CMPLT) == 0{}



    uint8_t xhi = Read_Buf[1];
    uint8_t xlo = Read_Buf[2];
    uint8_t yhi = Read_Buf[3];
    uint8_t ylo = Read_Buf[4];
    uint8_t zhi = Read_Buf[5];
    uint8_t zlo = Read_Buf[6];


  /* Shift values to create properly formed integer */
  gyro.x = (int16_t)((xhi << 8) | xlo);
  gyro.y = (int16_t)((yhi << 8) | ylo);
  gyro.z = (int16_t)((zhi << 8) | zlo);

  /* Assign raw values in case someone needs them */
  gyro_raw.x = gyro.x;
  gyro_raw.y = gyro.y;
  gyro_raw.z = gyro.z;

  /* Compensate values depending on the resolution */
 switch (_range) 
     { 
     case (GYRO_RANGE_2G): 
         gyro.x *= GYRO_MG_LSB_2G * SENSORS_GRAVITY_STANDARD; 
         gyro.y *= GYRO_MG_LSB_2G * SENSORS_GRAVITY_STANDARD; 
         gyro.z *= GYRO_MG_LSB_2G * SENSORS_GRAVITY_STANDARD; 
         //serial_UartPutString("here 5\r\n"); 
         break; 
     case (GYRO_RANGE_4G): 
         gyro.x *= GYRO_MG_LSB_4G * SENSORS_GRAVITY_STANDARD; 
         gyro.y *= GYRO_MG_LSB_4G * SENSORS_GRAVITY_STANDARD; 
         gyro.z *= GYRO_MG_LSB_4G * SENSORS_GRAVITY_STANDARD; 
         break; 
     case (GYRO_RANGE_8G): 
         gyro.x *= GYRO_MG_LSB_8G * SENSORS_GRAVITY_STANDARD; 
         gyro.y *= GYRO_MG_LSB_8G * SENSORS_GRAVITY_STANDARD; 
         gyro.z *= GYRO_MG_LSB_8G * SENSORS_GRAVITY_STANDARD; 
         break; 
     } 
 


  /* Convert values to rad/s */
  RADS.x *= DPS_TO_RADS;
  RADS.y *= DPS_TO_RADS;
  RADS.z *= DPS_TO_RADS;

  return true;
}

void FXAS21002C_readData(FXAS21002CRawData_t *gyro, FXAS21002CRawData_t *RADS) 
 { 
     //serial_UartPutString("here 1\r\n"); 
     FXOS8700_getData(); 
     //serial_UartPutString("here 6\r\n"); 
     gyro->x = gyro.x; 
     gyro->y = gyro.y; 
     gyro->z = gyro.z; 
 
 
     mag->x = RADS.x; 
     mag->y = RADS.y; 
     mag->z = RADS.z; 
 } 
