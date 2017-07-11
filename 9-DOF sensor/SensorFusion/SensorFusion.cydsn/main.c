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
#include "project.h"     //includes the project.h which allows us to use the generated API's
#include "accel_mag.h"   //includes the accel_mag.h file which allows us to simply connect to the NXP accel/mag sensor
#include "gyroscope.h"   //includes the gyroscope.h file which allows us to simply connect to the NXP gyroscope sensor
#include "debSerial.h"//includes the debuf_Serial.h file which allows us to use similar serial print functions like in Arduino IDE
#include "Madgwick.h" 

//Includes the standard libraries to use certain functions inside it
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

//=============================================================//
//======Initialization and Update Functions Prototype==========//
//=============================================================//
void quad_init(void);        //initializes all quadcopter 
void quad_sensor_init(void); //initializes all of the quadcopter sensors //called in function quad_init()
void sensors_update(void);   //Updates all of the sensor data (values can be extracted when needed)

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
 quad_init();
    serial_println("Quadcopter Sensors Initialized");
    for(;;)
    
    { //Calling functions below...
       sensors_update();
        CyDelay(1000); 
        accel_mag_initialize(ACCEL_RANGE_2G);
        gyro_initialize(GYRO_RANGE_250DPS); 
    }
}

/* [] END OF FILE */
