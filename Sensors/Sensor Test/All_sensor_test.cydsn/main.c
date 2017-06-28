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


void quad_init(void);
void quad_sensor_init(void);
void sensor_update(void);
void imu_update(void);

int main(void)
{
    quad_init();
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    serial_println("Started");
    for(;;)
    {
        /* Place your application code here. */
        sensor_update();
        //double i = pow((10.2),(2.7));
        //serial_printDoubleln(i);
        CyDelay(1000);
            //update the Barometric Pressure and Temperature Sensor values
    /*bar.rawTemp = baro_getRawTemperature();
    bar.rawPress = baro_getRawPressure();
    bar.temperature = baro_getTemperature();
    bar.pressure = baro_getPressure();

    bar.absAlt = baro_getAbsoluteAltitude(bar.pressure);
    bar.relAlt = baro_getRelativeAltitude((bar.pressure), (initial_reference_pressure__));

//if the macro DEBUG is true (value is not 0)
    serial_print(" rawTemp = ");
    serial_printInt(bar.rawTemp); //print an integer
    serial_print(", realTemp = ");
    serial_printInt(bar.temperature); //print a double
    serial_print(" *C");

    serial_print(" ||  rawPressure = ");
    serial_printInt(bar.rawPress); //print an integer
    serial_print(", realPressure = ");
    serial_printInt(bar.pressure); //print an integer
    serial_print(" Pa ");

    serial_print(" ||  absoluteAltitude = ");
    serial_printDouble(bar.absAlt); //print a double
    serial_print(" m, relativeAltitude = ");
    serial_printDouble(bar.relAlt); //print a double
    serial_println(" m");
    */
    }
     
}

void quad_init(void){
    Wire_begin();
    serial_begin();
    CyGlobalIntEnable; /* Enable global interrupts. */
    quad_sensor_init();
}

void quad_sensor_init(void){
    //accel_mag_initialize(ACCEL_RANGE_2G);
    //gyro_initialize(GYRO_RANGE_250DPS);
    baro_begin();
}

void sensor_update(void){
    //serial_PutString("HELLO\r\n");
    //accel_mag_update();
    //gyro_update();
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
