/* ========================================
 *
 * Copyright ARROW ELECTRONICS, 2017
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "accel_mag.h"
#include "gyroscope.h"
#include "barometer.h"
#include "Serial.h"
#include "twi.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>


void quad_init(void);        //initializes all quadcopter 
void quad_sensor_init(void); //initializes all of the quadcopter sensors
void sensors_update(void);   //Updates all of the sensor data (values can be extracted when needed)
void imu_update(void);       //Updates IMU roll pitch and yaw values

int main(void)
{
    quad_init();
    serial_println("Quadcopter Sensors Initialized");
    
    for(;;)
    {
        sensors_update();
        CyDelay(1000);
    }
     
}

void quad_init(void){
    Wire_begin();
    serial_begin();
    CyGlobalIntEnable; /* Enable global interrupts. */
    quad_sensor_init();
}

void quad_sensor_init(void){
    accel_mag_initialize(ACCEL_RANGE_2G);
    gyro_initialize(GYRO_RANGE_250DPS);
    baro_begin();
}

void sensors_update(void){
    //serial_PutString("HELLO\r\n");
    accel_mag_update();
    gyro_update();
    baro_update();
}

void imu_update(void){
   /*
    uint16_t roll  = 0;
   uint16_t pitch = 0;
   uint16_t yaw   = 0;
    */
}
/* [] END OF FILE */
