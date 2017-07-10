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
#include "project.h"     //includes the project.h which allows us to use the generated API's
#include "accel_mag.h"   //includes the accel_mag.h file which allows us to simply connect to the NXP accel/mag sensor
#include "gyroscope.h"   //includes the gyroscope.h file which allows us to simply connect to the NXP gyroscope sensor
#include "barometer.h"   //includes the barometer.h file which allows us to simply connect to the GY-63 Barometer sensor
#include "gps.h"
#include "debSerial.h"//includes the debuf_Serial.h file which allows us to use similar serial print functions like in Arduino IDE
#include "millis.h"

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

//=============================================================//
//======================Main Function==========================//
//=============================================================//

int main(void)
{
    quad_init();
    serial_println("Quadcopter Sensors Initialized");
    
    for(;;)
    {
        sensors_update();
        CyDelay(100);
    }
    return 0;
}

void quad_init(void){
    CyGlobalIntEnable; /* Enable global interrupts. */
    I2C_Start();       //initializes the I2C block
    serial_begin();    //initializes the debug Serial block
    gpsSerial_Start(); //initializes the gps Serial block
    xbeeSerial_Start();//initializes the xbee Serial block
    
    millis_init();
    quad_sensor_init();//initializes all of the quadcopter sensors
    
}

void quad_sensor_init(void){
    accel_mag_initialize(ACCEL_RANGE_2G); //initializes the accelerometer with a range (sensitivity) of 2G
    gyro_initialize(GYRO_RANGE_250DPS);   //initializes the gyroscope with a range (sensitivity) of 250DPS
    baro_begin();                         //initializes the barometer
    GPS_start();
}

void sensors_update(void){
    accel_mag_update(); //Update the accelerometer and magnotometer data
    gyro_update();      //Update the gyroscope data
    baro_update();      //Update the barometer data
    GPS_update();       //Update the GPS data
}
/* [] END OF FILE */
