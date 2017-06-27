//******************************************************************************//
//																				//
//		Project	:	Quad Copter													//
//		Device	:	PSoC 4														//
//		Version :	0.0															//
//		Date	:	March 2014													//
//		Author(s)	Bob Marlowe													//
//																				//
//******************************************************************************//
//																				//
//	The interface to the motors is made by providing a speed in percent of 		//
//	max power. It is then re-calculated to a timer-value between				//
//	1ms (motor off) to 2ms(motor full speed) as uses by normal rc-servos.		//
//																				//
//******************************************************************************//
 
//******************************************************************************//
//********************* Libraries **********************************************//
//******************************************************************************//
#include <Motors.h>
#include <project.h>

//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//
#define	MotNumber	4						//Number of Motors_h handled


//******************************************************************************//
//********************* Structures *********************************************//
//******************************************************************************//

//	This structure is used to access the motor-functions with an index
struct sMotFuncs						{
											void (*StartFunc)(void);
											void (*SetFunc)(uint8 Value);
										};

//******************************************************************************//
//********************* Data Area **********************************************//
//******************************************************************************//

//	"const" will put the structure into flash, no sram seeded
const struct sMotFuncs Motor[MotNumber] =	{
											{&MotFront_Start,	&MotFront_WriteCompare1},
											{&MotRear_Start,	&MotRear_WriteCompare1},
											{&MotLeft_Start,	&MotLeft_WriteCompare1},
											{&MotRight_Start,	&MotRight_WriteCompare1},
										};

uint8 CurrMotPercent[MotNumber]	= {0};		// Storage for wanted motor percentages

//******************************************************************************//
//********************* Program Area *******************************************//
//******************************************************************************//
void SetMotor(uint8 MotNo)					// Set motor speed to the desired value 
{
uint16 Count;
	Count = ((uint16)CurrMotPercent[MotNo] * 120)/100 +120;
	Motor[MotNo].SetFunc(Count);
}
//******************************************************************************//

void UpdateAllMotors(void)					// Set all motors to their desired speed
{
enum MotorNumber ii;
	for(ii = MFront; ii <= MRight; ii++) SetMotor(ii);
}
//******************************************************************************//

void SetMotSpeed(enum MotorNumber MotNo, uint8 Percent)
{											// Store the wanted motor-speed in percent
	if(Percent > 100) Percent = 100;
	CurrMotPercent[MotNo] = Percent;
}
//******************************************************************************//

void Motors_Start(void)						// Setup and halt all motors 
{
enum MotorNumber ii;
	for(ii = MFront; ii <= MRight; ii++) 
	{
		SetMotSpeed(ii,0);
		Motor[ii].StartFunc();
		UpdateAllMotors();
	}
}
//******************************************************************************//
void Motors_Stop(void)						// Halt all Motors
{
enum MotorNumber ii;
	for(ii = MFront; ii <= MRight; ii++) 
	{
		SetMotSpeed(ii,0);
		UpdateAllMotors();
	}
}
//******************************************************************************//

