//=====================================================================================================
// MayhonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MayhonyAHRS_h
#define MayhonyAHRS_h
	
#define Angle(x) (x/M_PI * 180.0)
	
//typedef float MyFloat;
	
typedef float Eulers[3];	

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MayhonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MayhonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void ToEuler(Eulers e);				//	Converts quaternium to euler-angles in radians
void ToAngle(Eulers rad, Eulers deg);	//	Converts radians rad to degrees deg

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
