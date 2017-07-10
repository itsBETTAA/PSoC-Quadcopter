/*
 * This is going to be the remade version of the FXAS21002C library from adafruit
 * to support the PSoC Version of our project
*/

#include "gyroscope.h"
#include "twi.h"
#include "project.h"
#include "SensorVal.h"
#include "stdlib.h"
#include <stdint.h>
#include "debSerial.h"

static gyroRange_t _range;

void gyro_write8(uint8_t reg, uint8_t value)
{

    //to write, you want to write 2 values: (1) The registeraddress you want to write to
    //                                      (2) The value you wish to write to the register

    uint8_t Write_Buf[2] = {0};  //create an array where we will store the things to be written
    Write_Buf[0] = (uint8_t)reg; //save the things to be sent in order in the array
    Write_Buf[1] = (uint8_t)value;

    //Make a complete transfer (I2C_MODE_COMPLETE_XFER) to the gyroscope
    //Before the array is sent, it first sends the address and that it's about to write to it
    //Then, it sends the first array value which is the register value it's trying to write to
    //And, finally once it receives an ACK it sends the second thing which is the value it is trying to write
    I2C_MasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Write_Buf, 2, I2C_MODE_COMPLETE_XFER);
    while ((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) == 0) //Wait till the whole transfer is complete
    {
    }

    return; //return 1 if everything was successful
}

uint8_t gyro_read8(uint8_t reg)
{

    //to read:  (1)Write the register address you want to read from to the slave device
    //          (2)Use MasterReadBuf to store what is contained in the refister into the read buffer

    uint8_t Write_Buf[1];
    Write_Buf[0] = (uint8_t)reg;

    uint8 Read_Buf[1] = {0};

    //Make a transfer with no stop bit (I2C_MODE_NO_STOP) to the gyroscope
    //Before the array is sent, it first sends the address and that it's about to write to it
    //Then, it sends the first array value which is the register value it's trying to point to
    //And, finally once it receives an ACK it sends the second thing which is the value it is trying to write
    I2C_MasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Write_Buf, 1, I2C_MODE_NO_STOP);
    while ((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) == 0)
    {
    } //wait till Master status indicates that write is complete

    //Once the first tranfer was done with a NO STOP BIT it then tells the gyroscope that it is
    //trying to read from it, after sending a repeated start. The Master will no extract as much
    //bytes as we told it to, starting from the value in the register we told it to point to
    I2C_MasterReadBuf(FXAS21002C_ADDRESS, (uint8 *)Read_Buf, 1, I2C_MODE_REPEAT_START);
    while ((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT) == 0)
    {
    } //wait till Master status indicates read is complete

    return Read_Buf[0]; //return the value we just read
}

uint8_t gyro_begin(gyroRange_t rng)
{
  /* Enable I2C */
  Wire_begin();

  /* Set the range to an appropriate value */
  _range = rng;

  /* Clear the raw sensor data */
  gyro_raw.x = 0;
  gyro_raw.y = 0;
  gyro_raw.z = 0;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = gyro_read8(GYRO_REGISTER_WHO_AM_I);
  // Serial.print("WHO AM I? 0x"); Serial.println(id, HEX);
  if (id != FXAS21002C_ID)
  {
    serial_println("Incorrect ID, check wiring or code");
    return false;
  }
  serial_println("Correct ID");

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
     0  READY     Standby(0)/Ready(1)                                 0*/

  /* Reset then switch to active mode with 100Hz output */
  gyro_write8(GYRO_REGISTER_CTRL_REG1, 0x00);  //clear all bit
  gyro_write8(GYRO_REGISTER_CTRL_REG1, (1<<6));  //set the reset bit
  gyro_write8(GYRO_REGISTER_CTRL_REG1, 0x0E);    //
  CyDelay(100); // 60 ms + 1/ODR
  /* ------------------------------------------------------------------ */

  return true;
}

uint8_t gyro_update()
{
  uint8_t readingValid = false;


  /* Clear the raw data placeholder */
  gyro_raw.x = 0;
  gyro_raw.y = 0;
  gyro_raw.z = 0;

  /* Read 7 bytes from the sensor */
  /*Wire.beginTransmission((byte)FXAS21002C_ADDRESS);
  #if ARDUINO >= 100
    Wire.write(GYRO_REGISTER_STATUS | 0x80);
  #else
    Wire.send(GYRO_REGISTER_STATUS | 0x80);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)FXAS21002C_ADDRESS, (byte)7);
  */

    uint8_t Write_Buf[1];  //create a buffer that will store the register that we want to start reading at
    Write_Buf[0] = (uint8_t)GYRO_REGISTER_STATUS;   
    
    uint8_t Read_Buf[7] = {0}; //create an array that will store the data read
    
    I2C_MasterWriteBuf(FXAS21002C_ADDRESS, (uint8 *)Write_Buf, 1, I2C_MODE_NO_STOP);
    while((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) == 0){} //wait till Master status indicates that write is complete
    
    I2C_MasterReadBuf(FXAS21002C_ADDRESS, (uint8 *) Read_Buf, sizeof(Read_Buf), I2C_MODE_REPEAT_START);
    while((I2C_MasterStatus() & I2C_MSTAT_RD_CMPLT)==0){}   //wait till Master status indicates read is complete
    

    uint8_t status = Read_Buf[0];
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

  #if GYROSCOPE_RAW_DEBUG
        serial_print("gyro_raw.x = ");
        serial_printInt(gyro_raw.x);
        serial_print(" || gyro_raw.y = ");
        serial_printInt(gyro_raw.y);
        serial_print("|| gyro_raw.z = ");
        serial_printInt(gyro_raw.z);
        serial_println(" rad/s");

  #endif

  /* Compensate values depending on the resolution */
  switch(_range)
  {
    case GYRO_RANGE_250DPS:
      gyro.x *= GYRO_SENSITIVITY_250DPS;
      gyro.y *= GYRO_SENSITIVITY_250DPS;
      gyro.z *= GYRO_SENSITIVITY_250DPS;
      break;
    case GYRO_RANGE_500DPS:
      gyro.x *= GYRO_SENSITIVITY_500DPS;
      gyro.y *= GYRO_SENSITIVITY_500DPS;
      gyro.z *= GYRO_SENSITIVITY_500DPS;
      break;
    case GYRO_RANGE_1000DPS:
      gyro.x *= GYRO_SENSITIVITY_1000DPS;
      gyro.y *= GYRO_SENSITIVITY_1000DPS;
      gyro.z *= GYRO_SENSITIVITY_1000DPS;
      break;
    case GYRO_RANGE_2000DPS:
      gyro.x *= GYRO_SENSITIVITY_2000DPS;
      gyro.y *= GYRO_SENSITIVITY_2000DPS;
      gyro.z *= GYRO_SENSITIVITY_2000DPS;
      break;
  }

  /* Convert values to rad/s */
  gyro.x *= SENSORS_DPS_TO_RADS;
  gyro.y *= SENSORS_DPS_TO_RADS;
  gyro.z *= SENSORS_DPS_TO_RADS;

  #if GYROSCOPE_DEBUG
        serial_print("gyro.x = ");
        serial_printFloat(gyro.x);
        serial_print(" || gyro.y = ");
        serial_printFloat(gyro.y);
        serial_print("  || gyro.z = ");
        serial_printFloatln(gyro.z);
  #endif
  readingValid = true;
  return readingValid;
}

void gyro_readData(gyroData_t *gyroscope)
{
    //gets the gyroscope data
    gyro_update();
    gyroscope->x = gyro.x;
    gyroscope->y = gyro.y;
    gyroscope->z = gyro.z;

}

uint8_t gyro_initialize(gyroRange_t rng){
    uint8_t ret = 1;
    ret = gyro_begin(rng);
    return ret;
}

/*  These functions are provided to allow the user to extract
 *  single pieces of data from the sensor readings safely
 *  instead of just calling the sensor variables directly
*/
float gyro_getX(void){
  return (gyro.x);
}

float gyro_getY(void){
  return (gyro.y);
}

float gyro_getZ(void){
  return (gyro.z);
}