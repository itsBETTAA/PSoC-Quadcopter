//******************************************************************************//
//																				//
//		Project	:	Quad Copter													//
//		Device	:	PSoC 4														//
//		Version :	0.0															//
//		Date	:	March 2014													//
//		Author(s)	Bob Marlowe													//
//																				//
//******************************************************************************//

#ifndef GlobalDefs_h
#define GlobalDefs_h

//******************************************************************************//
//********************* Libraries **********************************************//
//******************************************************************************//
#include <CyLib.h>
#include <StdLib.h>
//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//
#define FALSE		0
#define TRUE		!FALSE
#define forever		1

#define BitMask(BitNo)	(0x01 << BitNo)	
#define IsBitSet(Value,BitNo) (Value & BitMask(BitNo))

#define CycleTime	20								//	Main-loop cycletime is 20 ms = 50Hz
#define RCLossSec	5								//	Seconds until a loss of RC signal is detected
#define	SigTimeout 1000/CycleTime * RCLossSec		//	Calculated timeout for loss of RC-Signal
	
enum AXIS {ROLL, PITCH, YAW};
	
#endif
