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
#include <Sensors/accelerometer.h>
#include <Sensors/twi.h>
#include <stdlib.h>
#include <stdio.h>


#define true 1
#define false 0

fxos8700RawData_t acceleration; /* values from last sensor read in (m/s^2) */
fxos8700RawData_t magnetic;

/*
 * Key words:
 *         -acceleration(.x,.y,.z) for accelerometer data in (m/s^2)
 *         -magnetic    (.x,.y,.z) for magnetometer data in (uTesla)
*/

void initialize(){
   
   serial_Start();    //initialize the UART
   I2C_Start();       //initializes the I2C

   CyGlobalIntEnable; /* Enable global interrupts. */

   serial_PutString("Starting FXOS8700 initializing\r\n");
   uint8_t err = FXOS8700_begin(ACCEL_RANGE_2G);
    serial_PutString("//======Done FXOS8700 initializing======//\r\n"); //42
    if(!err){
        serial_PutString("//===FXOS8700 initializing failed===//\r\n");
    }
    else{
        serial_PutString("//===FXOS8700 initialization success===//\r\n");
    }
}

int main(void)
{
    h = true;
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    initialize();
    
    for(;;)
    {
        char buffer[20];
        /* Place your application code here. */
        FXOS8700_readData(&acceleration, &magnetic);
        serial_PutString("accelerometer.x = ");
        serial_PutString(itoa((acceleration.x),buffer,10));
        serial_PutString("  accelerometer.y = ");
        serial_PutString(itoa((acceleration.y),buffer,10));
        serial_PutString("  accelerometer.z = ");
        serial_PutString(itoa((acceleration.z),buffer,10));
        serial_PutString("\r\n");
        CyDelay(200);
    }
}

/* [] END OF FILE */

