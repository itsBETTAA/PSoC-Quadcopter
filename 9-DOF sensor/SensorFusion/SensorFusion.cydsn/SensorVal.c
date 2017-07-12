
/*
*   This will be the file to store all of the sensor values,
*   So if any part of the code wants to extract the value of the sensor 
*   after recently calling its update function, it will only need to 
*   call the variables in this file.
*/

#include <accel_mag.h>
#include <gyroscope.h>
#include <barometer.h>
#include <gps.h>

/*==================For accelerometer and magnotometer data==============*/

fxos8700RawData_t accel_raw; /* Raw values from last sensor read */
fxos8700RawData_t mag_raw;   /* Raw values from last sensor read */

fxos8700Data_t accel; /* values from last sensor read in (m/s^2) */
fxos8700Data_t mag;   /*  values from last sensor read in (uTesla) */

/*==========================For gyroscope data===========================*/

gyroRawData_t gyro_raw; /* Raw values from last sensor read */
gyroData_t gyro;

/*===========================For barometer data==========================*/

//read the reference Pressure to enable us to calculate the relative Altitude
int32_t reference_pressure;

//read the initial raw Temperature and Pressure just in case we later want some reference
uint32_t initial_raw_temperature;
uint32_t initial_raw_pressure;

//read the real compasated Temperature and Pressure just in case we want it later
double initial_real_temprature;
int32_t initial_real_pressure;

//Calculate and store the initial absolute and relative altitude
//The initial relative altitude should be less than 10, MAX
double initial_absolute_altitude;
double initial_relative_altitude;

/* This represents a struct variable
    typedef struct 
    {
        uint32_t rawTemp;
        uint32_t rawPress;
        int32_t temperature;
        int32_t pressure;

        double absAlt;
        double relAlt;
    } barometricRawData_t;
*/
barometricData_t baro;

/*===========================For GPS data==========================*/
/*
typedef struct
{
  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint16_t milliseconds;
  float latitude;
  float longitude;
  int32_t latitude_fixed;
  int32_t longitude_fixed;
  float latitudeDegrees;
  float longitudeDegrees;
  float geoidheight;
  float altitude;
  float speed;
  float angle;
  float magvariation;
  float HDOP;
  char lat;
  char lon;
  char mag;
  uint8_t fix;
  uint8_t fixquality;
  uint8_t satellites;
} gpsData_t;
*/

gpsData_t GPS;