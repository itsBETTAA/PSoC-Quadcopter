//=============================================================================================
// Madgwick.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef Madgwick_h__
#define Madgwick_h__
#include <math.h>

// Function declarations
    int Madgwick(void); //Added "int" to avoid warning msg; hope this causes no issue - Toni
    void Madgwick_begin(float sampleFrequency);
    void Madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void Madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getRoll();
    float getPitch(); 
    float getYaw();
    float getRollRadians();
    float getPitchRadians();
    float getYawRadians();
    void getQuaternion(float *w, float *x, float *y, float *z);

#endif