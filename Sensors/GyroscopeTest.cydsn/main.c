/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <gyroscope.h>
#include <twi.h>
#include <stdlib.h>

gyroRawData_t gyroData;
/*
 * Key words:
 *         -gyroData(.x,.y,.z) for accelerometer data in (m/s^2)
 
*/

void initialize(){
   
   serial_Start();    //initialize the UART
   I2C_Start();       //initializes the I2C

   CyGlobalIntEnable; /* Enable global interrupts. */

   serial_PutString("Starting GYRO initializing\r\n");
   uint8_t err = gyro_begin(GYRO_RANGE_250DPS );
    serial_PutString("//======Done GYRO initializing======//\r\n"); //42
    if(!err){
        serial_PutString("//===GYRO initializing failed===//\r\n");
    }
    else{
        serial_PutString("//===GYRO initialization success===//\r\n");
    }
}

int main(void)
{
    initialize();

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
