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
//																				//
//	This project is the attempt to control a quad copter with a PSoC4 chip		//
//																				//
//	The underlying hardware can be as small as the CY8CKIT-049-42xx-board 		//
//	which with a weight of less than 10g puts a rather small load to a flying 	//
//	object. The motor-controll uses all four UDBs, the communication to the 	//
//	sensors is made up with I2C.												//
//	The requred MEMS sensors are: 3-axis magnet for the orientation of yaw		//
//	a 3-axis gyro to detect movements and a 2 or 3-axis accellerometer to 		//
//	detect the pitch and roll-angles and a yaw acceleration.					//
//																				//
//	I would like to point out that the project can be developed and debugged 	//
//	with the Pioneer board and later programmed into a KIT-049. Debugging on a	//
//	KIT-049 is possible only with the help of a MiniProg3.						//
//																				//
//******************************************************************************//
//																				//
//	The current design controls 4 motors via the standard 1 to 2ms rc interface	//
//	and uses a so-called multiplex sum-signal as input which is a standard		//
//	for copters, too.															//
//	The targeted update frequency is 50Hz, which could get tough for the		//
//	floating-point math used for the AHRS algorithm AND for the time needed		//
//	to transfer the data from the sensors which should be averaged to reduce	//
//	noise that will get more caused by vibrations from rotating motors.			//
//																				//
//	In the file <Motors.c> you may introduce some changes to controll up to		//
//	eight(!!) motors by using the secondary outputs of the PWMs					//
//	Of course some additional changes have to be made, but it looks as if it	//
//	might be possible to control an Oct-Copter with a single PSoC4!!!			//
//																				//
//	Welcome in the fascinating world of PSoCs!									//
//																				//
//	Bob Marlowe (aka Jörg Meier)												//
//																				//
//******************************************************************************//



//******************************************************************************//
//********************* Libraries **********************************************//
//******************************************************************************//
#include <project.h>
#include <GlobalDefs.h>
#include <DeMPX.h>
#include <Sensors.h>
#include <Motors.h>
#include <Mayhony.h>
#include <ControlLoop.h>
#include <RC_Control.h>

//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//

//******************************************************************************//
//********************* Prototypes **********************************************//
//******************************************************************************//
CY_ISR_PROTO(Handler50Hz);				//	Interrupt for the main-loop

//******************************************************************************//
//********************* Data Area **********************************************//
//******************************************************************************//
uint8 WaitFor50Hz	= TRUE;				//	Indicator for Main clock
Eulers PRY;								//	Pitch, roll and yaw angles in degrees
float	InitialHeading = 0.0;			//	Relative heading is zero when starting;
//******************************************************************************//
//********************* Program Area *******************************************//
//******************************************************************************//
void IndicatorLed_1_Toggle(void)		// For debug purposes, pulse an output pin
{
	IndicatorLED_1_Write(0);
	CyDelay(1);
	IndicatorLED_1_Write(1);
}
//******************************************************************************//

void InitializeHardware(void)
{
	CyGlobalIntEnable;					//	Interrupts may fire
	Timer50Hz_Start();					//	Main clock is running at 50Hz
	Int50Hz_StartEx(Handler50Hz);		//	Main-loop interrupt
	MPX_Start();						//	Start the De-Multiplexing of RC-signal
	IndicatorLed_1_Toggle();			//	Dor debug purposes: Toggle a pin
	Sensors_Start();					//	Start and calibrate the sensors
	
										//	Starting the rc - interfaces (Deadband, Gain, Limit, Channel)
	Roll_Start	(1.0,0.01,10.0,0);		//  Start Roll		rc-channel 0;
	Pitch_Start	(1.0,0.01,10.0,1);		//  Start Pitch		rc-channel 1;
	Yaw_Start	(1.0,0.01,10.0,2);		//  Start Yaw		rc-channel 2;
	Height_Start(10.0,0.1,100.0,3);		//  Start Height	rc-channel 3;
	
										//	Starting of the control-loops (Kp, Ki, Kd, Ta) 
	RollControl		= Control_Start(2.0,0.5,0.0,CycleTime / 1000.0);
	PitchControl	= Control_Start(2.0,0.5,0.0,CycleTime / 1000.0);
	YawControl		= Control_Start(2.0,0.5,0.0,CycleTime / 1000.0);
	HeightControl	= Control_Start(2.0,0.5,0.0,CycleTime / 1000.0);
	
	Motors_Start();						//	get ready
}
//******************************************************************************//

void RC_Shutdown(void)
{
	//	Later, we could try an auto-descent to safely ground the copter
	//	Now we just shutdown the motors
	Motors_Stop();
}
//******************************************************************************//

void GetAttitude(void)
{
Eulers pry;								// Pitch, roll and yaw angled in radians
	MayhonyAHRSupdate(Gyro_X,Gyro_Y,Gyro_Z,Accellmeter_X,Accellmeter_Y,Accellmeter_Z,Magneto_X,Magneto_Y,Magneto_Z);	
	ToEuler(pry);						// Conversion of quaternion to radians
	ToAngle(pry,PRY);					// Conversion to degrees
	PRY[2] -= InitialHeading;
	if(PRY[2] < 0) PRY[2] += 360.0;
}
//******************************************************************************//

void Mixer(void)
{
	// Purpose is to distribute the 3 AHRS values together with
	// the incoming RC for yaw,pitch,roll and up/down
	// to the 4 motor controls of the copter.
static uint8 SignalLossCounter = 0;		// Monitor signal losses 
float R,P,Y,H;							// Roll, Pitch, Yaw and Height	
float Motors[4];						// QUAD Copter
float Max = -10000.0;
float Min =  10000.0;
uint8 ii;				
	if(MPX_SignalLost)					// Do we still have an incoming RC-signal
	{
		SignalLossCounter++;			// NO !!
	}
	else
	{
		SignalLossCounter = 0;			// Yes
	}
/*	
	if(SignalLossCounter > SigTimeout)	// Do we have to react on loss of signal?
	{
		RC_Shutdown();					// Yes, 
		return;
	}
*/										// We are here as the normal case
	R = PRY[ROLL] + RC_GetValue(Roll);	// Get the values of the actual + wanted
	P = PRY[PITCH] + RC_GetValue(Pitch);
	Y = PRY[YAW] + RC_GetValue(Yaw);
	H = RC_GetValue(Height); 
	
	R = Controller(RollControl,PRY[ROLL],R); 	// Calculate the PID-Closed-Loop
	P = Controller(PitchControl,PRY[ROLL],P);
	Y = Controller(YawControl,PRY[ROLL],Y);
Y = 0;	
	for(ii = FirstMot; ii <= LastMot; ii ++)
	{
		Motors[ii] = H;
	}
	
	Motors[MLeft]		-= R + Y ;
	Motors[MRight]		+= R + Y;
	Motors[MFront]		-= P - Y;
	Motors[MRear]		+= P - Y;
	
	for(ii = FirstMot; ii <= LastMot; ii ++)	// Checking for min and max values
	{
		if(Motors[ii] > Max) Max = Motors[ii];
		if(Motors[ii] < Min) Min = Motors[ii];
	}
	

	
	
}
//******************************************************************************//

int main()
{
	InitializeHardware();					//	Startup everything
	InitialHeading = 0.0;					//	
	GetAttitude();							//	Get initial attitude
	InitialHeading = PRY[2];				//	Set the original heading relative to mag north
    while(forever)							//	Main loop 
    {
		IndicatorLED_0_Write(FALSE);		//	Switch LED off to show idle		
		while(WaitFor50Hz) Wait();			//	Wait until 50Hz timer expires
		IndicatorLED_0_Write(TRUE);			//	Switch LED on to show processing
		
		IndicatorLED_2_Write(TRUE);			//	Switch LED on to show processing
		Sensors_GetAll();
		IndicatorLED_2_Write(FALSE);		//	Switch LED off to show processing
											//	This lengthy parameter list calculates the new quaternion
		GetAttitude();
		
		Mixer();							//	Controll motors with RC
		UpdateAllMotors();					//	Actualize the motor settings
		WaitFor50Hz = TRUE;					//	Indicate that the current cycle is through
	}
}
//******************************************************************************//

CY_ISR(Handler50Hz)
{
	Timer50Hz_ClearInterrupt(Timer50Hz_GetInterruptSource());	//	Very important: Reset the interrupt
	WaitFor50Hz = FALSE;					//	Indicate timer occured
}
//******************************************************************************//
