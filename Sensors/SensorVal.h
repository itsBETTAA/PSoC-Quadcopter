#ifndef SENSORVAL_H
#define SENSORVAL_H

#include <accel_mag.h>
#include <gyroscope.h>
#include <barometer.h>
/*==================For accelerometer and magnotometer data==============*/

extern fxos8700RawData_t accel_raw; /* Raw values from last sensor read */
extern fxos8700RawData_t mag_raw;   /* Raw values from last sensor read */

extern fxos8700Data_t accel; /* values from last sensor read in (m/s^2) */
extern fxos8700Data_t mag;     /*  values from last sensor read in (uTesla) */

/*==========================For gyroscope data===========================*/

extern gyroRawData_t gyro_raw; /* Raw values from last sensor read */
extern gyroData_t gyro;

/*===========================For barometer data==========================*/

//read the reference Pressure to enable us to calculate the relative Altitude
extern int32_t reference_pressure;

//read the initial raw Temperature and Pressure just in case we later want some reference
extern uint32_t initial_raw_temperature;
extern uint32_t initial_raw_pressure;

//read the real compasated Temperature and Pressure just in case we want it later
extern double initial_real_temprature;
extern int32_t initial_real_pressure;

//Calculate and store the initial absolute and relative altitude
//The initial relative altitude should be less than 10, MAX
extern double initial_absolute_altitude;
extern double initial_relative_altitude;

extern barometricData_t baro;
    
/*======================For IMU Measurment data==========================*/

#endif