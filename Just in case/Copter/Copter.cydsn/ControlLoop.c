//******************************************************************************//
//																				//
//		Project	:	Quad Copter													//
//		Device	:	PSoC 4														//
//		Version :	0.0															//
//		Date	:	March 2014													//
//		Author(s)	Bob Marlowe													//
//																				//
//******************************************************************************//

//******************************************************************************//
//********************* Libraries **********************************************//
//******************************************************************************//
#include <GlobalDefs.h>
#include <ControlLoop.h>
#include <stdlib.h>

//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//
	
//******************************************************************************//
//******************** Structures / Enums **************************************//
//******************************************************************************//

//******************************************************************************//
//********************* Data Area **********************************************//
//******************************************************************************//
Control RollControl;			// PID-Controller X-Axis
Control PitchControl;			// PID-Controller Y-Axis
Control YawControl;				// PID-Controller Z-Axis
Control HeightControl;			// PID-Controller Up/Down

//******************************************************************************//
//********************* APIs ***************************************************//
//******************************************************************************//

Control Control_Start(float KP, float KI, float KD, float Ta)
{
Control PID;
	PID = (void *)malloc(sizeof(struct sControl));	// Allocate memory for controller data
	PID->KP 		= KP;							// Proportional factor
	PID->KI 		= KI * Ta;						// Integration factor (pre-calculated)
	PID->KD 		= KD / Ta;						// Derivative factor (pre-calculated)
	PID->ErrSum		= 0.0;							// Error integral
	PID->OldErr		= 0.0;							// Error from stage before
	return PID;										// Return pointer to controller data
}
//******************************************************************************//

float Controller(Control PID, float Wanted, float ShouldBe)
{
float Error;
float Result;
	Error  = ShouldBe - Wanted;						// Comparison
	PID->ErrSum += Error;							// Integration of Error
	Result = PID->KP * Error + PID->KI * PID->ErrSum + PID->KD * (Error - PID->OldErr);	//Controller - equation
	PID->OldErr = Error;							// Remember for next step
	return Result;
}
//******************************************************************************//



