/*
 * This is going to be the remade version of the FXOS8700 library from adafruit
 * to support the PSoC Version of our project
*/

#include "accel_mag.h" /*include header files*/
#include "twi.h"
#include "project.h"
#include "SensorVal.h"
#include "SensorVal.c"
#include "stdlib.h"
#include <stdint.h>
#include "Serial.h"

static fxos8700AccelRange_t _range; /*will be used to save the sensitivity wanted*/

/*A couple of macros are defined for simplicity in the code*/
#define _write I2C_I2C_WRITE_XFER_MODE
#define _read I2C_I2C_READ_XFER_MODE

#define ACCEL_MG_LSB_2G (0.000244F)
#define ACCEL_MG_LSB_4G (0.000488F)
#define ACCEL_MG_LSB_8G (0.000976F)
#define MAG_UT_LSB (0.1F)

#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON (1.6F)      /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN (275.0F)     /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX (60.0F)      /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN (30.0F)      /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA (1013.25F) /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_GAUSS_TO_MICROTESLA (100)       /**< Gauss to micro-Tesla multiplier */

#define true 1
#define false 0

/**************************************************************************/
/*!
    @brief  Abstract away the I2C reading and writing procedure
*/
/**************************************************************************/


void accel_mag_write8(byte Reg, byte value)
{
    
//to write, you want to write 2 values: (1) The registeraddress you want to write to
//                                      (2) The value you wish to write to the register

    uint8_t Write_Buf[2] = {0};
    Write_Buf[0] = (uint8_t)Reg;
    Write_Buf[1] = (uint8_t)value;
    
    I2C_MasterWriteBuf(FXOS8700_ADDRESS, (uint8 *)Write_Buf, 2, I2C_MODE_COMPLETE_XFER);
    while((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT)==0){}
    
    return;                //return 1 if everything was successful
}

uint8_t accel_mag_read8(uint8_t Reg)
{
    
//to read:  (1)Write the register address you want to read from to the slave device 
//          (2)Use MasterReadBuf to store what is contained in the refister into the read buffer

    uint8_t Write_Buf[1];
    Write_Buf[0] = (uint8_t)Reg;
    
    uint8 Read_Buf[1] = {0};
    
    I2C_MasterWriteBuf(FXOS8700_ADDRESS, (uint8 *)Write_Buf, 1, I2C_MODE_NO_STOP);
    while((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) == 0){} //wait till Master status indicates that write is complete
   
    I2C_MasterReadBuf(FXOS8700_ADDRESS, (uint8 *) Read_Buf, 1, I2C_MODE_REPEAT_START);
    while((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT)==0){}   //wait till Master status indicates read is complete

    return Read_Buf[0];
}

/**************************************************************************/
/*!
    initialize the HW (Hardware)
*/
/**************************************************************************/

uint8_t accel_mag_begin(fxos8700AccelRange_t rng){
    /* Enable I2C */
    Wire_begin();
    /* Set the range the an appropriate value */
    _range = rng;

    /* Clear the raw sensor data */
    accel_raw.x = 0;
    accel_raw.y = 0;
    accel_raw.z = 0;
    mag_raw.x = 0;
    mag_raw.y = 0;
    mag_raw.z = 0;

    /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
    uint32_t id = accel_mag_read8(FXOS8700_REGISTER_WHO_AM_I);
    if (id != FXOS8700_ID)
    {
        serial_println("Incorrect ID, check wiring or code");
        return false;
    }

    /* Set to standby mode (required to make changes to this register) */
    accel_mag_write8(FXOS8700_REGISTER_CTRL_REG1, 0);

    /* Configure the accelerometer */
    switch (_range)
    {
    case (ACCEL_RANGE_2G):
        accel_mag_write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00);
        break;
    case (ACCEL_RANGE_4G):
        accel_mag_write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x01);
        break;
    case (ACCEL_RANGE_8G):
        accel_mag_write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x02);
        break;
    }
    /* High resolution */
    accel_mag_write8(FXOS8700_REGISTER_CTRL_REG2, 0x02);
    /* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
    accel_mag_write8(FXOS8700_REGISTER_CTRL_REG1, 0x15);

    /* Configure the magnetometer */
    /* Hybrid Mode, Over Sampling Rate = 16 */
    accel_mag_write8(FXOS8700_REGISTER_MCTRL_REG1, 0x1F);
    /* Jump to reg 0x33 after reading 0x06 */
    accel_mag_write8(FXOS8700_REGISTER_MCTRL_REG2, 0x20);

    return true;
}

uint8_t accel_mag_update()
{

    /* Clear the raw data placeholder */
    accel_raw.x = 0;
    accel_raw.y = 0;
    accel_raw.z = 0;
    mag_raw.x = 0;
    mag_raw.y = 0;
    mag_raw.z = 0;

    /*Wire.beginTransmission((byte)FXOS8700_ADDRESS);
    Wire.write(FXOS8700_REGISTER_STATUS | 0x80);
    Wire.endTransmission();
    Wire.requestFrom((byte)FXOS8700_ADDRESS, (byte)13);*/
    
    //serial_UartPutString("here 2\r\n");
    
    /*I2C_I2CMasterSendStart((byte)FXOS8700_ADDRESS, _write);
    serial_UartPutString("here 3\r\n");
    I2C_I2CMasterWriteByte(FXOS8700_REGISTER_STATUS | 0x80);
    if (I2C_I2C_MSTR_NO_ERROR != (I2C_I2CMasterWriteByte(FXOS8700_REGISTER_STATUS | 0x80)))
    {
        serial_UartPutString("here 4\r\n");
        return false;
    }
    serial_UartPutString("here 5\r\n");
    I2C_I2CMasterSendStop();*/
    
    uint8_t Write_Buf[1];  //create a buffer that will store the register that we want to start reading at
    Write_Buf[0] = (uint8_t)FXOS8700_REGISTER_STATUS;   
    
    uint8_t Read_Buf[14] = {0}; //create an array that will store the data read
    
    I2C_MasterWriteBuf(FXOS8700_ADDRESS, (uint8 *)Write_Buf, 1, I2C_MODE_NO_STOP);
    while((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) == 0){} //wait till Master status indicates that write is complete
    
    I2C_MasterReadBuf(FXOS8700_ADDRESS, (uint8 *) Read_Buf, sizeof(Read_Buf), I2C_MODE_REPEAT_START);
    while((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT)==0){}   //wait till Master status indicates read is complete
    
    //serial_UartPutString("here 3\r\n");
    //Wire_requestFrom((byte)FXOS8700_ADDRESS, sizeof(Read_Buf), Read_Buf);
    
    //serial_UartPutString("here 4\r\n");
    
    //uint8_t status = Read_Buf[0]; //Status not used
    uint8_t axhi = Read_Buf[1];
    uint8_t axlo = Read_Buf[2];
    uint8_t ayhi = Read_Buf[3];
    uint8_t aylo = Read_Buf[4];
    uint8_t azhi = Read_Buf[5];
    uint8_t azlo = Read_Buf[6];
    uint8_t mxhi = Read_Buf[7];
    uint8_t mxlo = Read_Buf[8];
    uint8_t myhi = Read_Buf[9];
    uint8_t mylo = Read_Buf[10];
    uint8_t mzhi = Read_Buf[11];
    uint8_t mzlo = Read_Buf[12];

    /* Shift values to create properly formed integers */
    /* Note, accel data is 14-bit and left-aligned, so we shift two bit right */
    accel.x = (int16_t)((axhi << 8) | axlo) >> 2;
    accel.y = (int16_t)((ayhi << 8) | aylo) >> 2;
    accel.z = (int16_t)((azhi << 8) | azlo) >> 2;
    mag.x = (int16_t)((mxhi << 8) | mxlo);
    mag.y = (int16_t)((myhi << 8) | mylo);
    mag.z = (int16_t)((mzhi << 8) | mzlo);

    /* Assign raw values in case someone needs them */
    accel_raw.x = accel.x;
    accel_raw.y = accel.y;
    accel_raw.z = accel.z;
    mag_raw.x = mag.x;
    mag_raw.y = mag.y;
    mag_raw.z = mag.z;
    
    /* Convert accel values to m/s^2 */
    switch (_range)
    {
    case (ACCEL_RANGE_2G):
        accel.x *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
        accel.y *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
        accel.z *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
        //serial_UartPutString("here 5\r\n");
        break;
    case (ACCEL_RANGE_4G):
        accel.x *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
        accel.y *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
        accel.z *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
        break;
    case (ACCEL_RANGE_8G):
        accel.x *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
        accel.y *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
        accel.z *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
        break;
    }

    /* Convert mag values to uTesla */
    mag.x *= MAG_UT_LSB;
    mag.y *= MAG_UT_LSB;
    mag.z *= MAG_UT_LSB;

    #if ACCEL_MAG_DEBUG 
        serial_print("accel.x = ");
        serial_printDouble(accel.x);
        serial_print(" accel.y = ");
        serial_printDouble(accel.y);
        serial_print(" accel.z = ");
        serial_printDouble(accel.z);
        serial_print(" m/(s^2)");

        serial_print("   mag.x = ");
        serial_printDouble(mag.x);
        serial_print(" mag.y = ");
        serial_printDouble(mag.y);
        serial_print(" mag.z = ");
        serial_printDouble(mag.z);
        serial_println(" uTesla");

    #endif

    return true;
}

void accel_mag_readData(fxos8700Data_t * _acceleration, fxos8700Data_t * _magnetic)
{
    //serial_UartPutString("here 1\r\n");
    accel_mag_update();
    //serial_UartPutString("here 6\r\n");
     _acceleration->x = accel.x;
     _acceleration->y = accel.y;
     _acceleration->z = accel.z;

    _magnetic->x = mag.x;
    _magnetic->y = mag.y;
    _magnetic->z = mag.z;
}

void accel_readData(fxos8700Data_t * _acceleration){
    accel_mag_update();
    _acceleration->x = accel.x;
    _acceleration->y = accel.y;
    _acceleration->z = accel.z;
}

void mag_readData(fxos8700Data_t * _magnetic){
    accel_mag_update();
    _magnetic->x = mag.x;
    _magnetic->y = mag.y;
    _magnetic->z = mag.z;
}

uint8_t accel_mag_initialize(fxos8700AccelRange_t rng){
    uint8_t ret = 1;
    ret = accel_mag_begin(rng);
    return ret;
}

/*  These functions are provided to allow the user to extract
 *  single pieces of data from the sensor readings safely
 *  instead of just calling the sensor variables directly
*/
int16_t accel_getX(void){
    return (accel.x);
}

int16_t accel_getY(void){
    return (accel.y);
}

int16_t accel_getZ(void){
    return (accel.z);
}

int16_t mag_getX(void){
    return (mag.x);
}

int16_t mag_getY(void){
    return (mag.x);
}

int16_t mag_getZ(void){
    return (mag.x);
}