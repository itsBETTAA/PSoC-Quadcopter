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
#include <project.h>
#include <DeMPX.h>
#include <GlobalDefs.h>

//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//
#define TicksPer_ms		120							// Clock runs at 120kHz
#define	SyncPulseLength 3	* TicksPer_ms			// Syncpulse is > 3ms
#define	LowPulseLength  1	* TicksPer_ms			// Lowest is 1ms
#define	HighPulseLength	2	* TicksPer_ms			// Longest is 2ms

//******************************************************************************//
//********************* Data Area **********************************************//
//******************************************************************************//
uint16 Channel[NumberOfChannels];					// Decoded channel values from RC
uint8 MPX_SignalLost = TRUE;						// No Signal detected initially

//******************************************************************************//
//********************* Prototypes **********************************************//
//******************************************************************************//
CY_ISR_PROTO(MPXHandler);


//******************************************************************************//
//********************* Program Area *******************************************//
//******************************************************************************//
void MPX_Start(void)								// Start the De-Multiplexer
{
	MPX_SignalLost = TRUE;
	MPXInt_StartEx(MPXHandler);
	MPX_Discriminator_Start();
}
//**********************************************************************
void MPX_Stop(void)									// Stop it
{
	MPXInt_Stop();
	MPX_Discriminator_Stop();
	MPX_SignalLost = TRUE;
}
//**********************************************************************

CY_ISR(MPXHandler)
{
static uint16 ChCount = 0;							// Counter for actual channel
int32 CapValue;
uint32 IntSource = MPX_Discriminator_GetInterruptSource();
	MPX_Discriminator_ClearInterrupt(IntSource);
	if(IntSource & MPX_Discriminator_INTR_MASK_TC)
	{												// We lost Signal!! Indicate that, then exit
		ChCount = -1;
		MPX_SignalLost = TRUE;
		return;
	}
	if(ChCount >= SyncPulseLength)					// Do we read a synchronize pulse
	{
		ChCount = 0;								// Startz with first channel
		return;
	}
	
	CapValue = MPX_Discriminator_ReadCapture();		// Get signal duration
	if(CapValue < LowPulseLength * 8 / 10)			// Distortion, pulse too short
	{
		return;
	}	
	
	if(ChCount >= NumberOfChannels)					// More channels than defined, ignore
	{
		return;
	}
	
	if(ChCount == (NumberOfChannels -1)) 			// When all channels got a value
	{
		MPX_SignalLost = FALSE;							// Signal that we saw a sync-pulse
	}
	
//	Adjust the read value from 2..20ms to 0..999
	CapValue -= LowPulseLength;
	CapValue *= 10;
	if(CapValue < 0) CapValue = 0;
	if(CapValue >= 999) CapValue = 999;
	Channel[ChCount++] = CapValue;
}
//**********************************************************************

