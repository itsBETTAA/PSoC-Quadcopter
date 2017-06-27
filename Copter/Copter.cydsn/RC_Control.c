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
#include <RC_Control.h>

//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//
#define sign(x)		(x>0?1:-1)

//******************************************************************************//
//******************** Structures / Enums **************************************//
//******************************************************************************//
					
//******************************************************************************//
//********************* Data Area **********************************************//
//******************************************************************************//
RC_Channel Roll = NULL;									// Basic channels to control a flying object
RC_Channel Pitch = NULL;
RC_Channel Yaw = NULL;
RC_Channel Height = NULL;


//******************************************************************************//
//********************* APIs ***************************************************//
//******************************************************************************//

void Roll_Start	(float Deadband, float Gain, float Limit, int16 ChIndex)
{
	if(Roll != NULL) free(Roll);
	Roll = RC_Start(Deadband,Gain,Limit,ChIndex);
}
//******************************************************************************//

void Pitch_Start	(float Deadband, float Gain, float Limit, int16 ChIndex)
{
	if(Pitch != NULL) free(Roll);
	Pitch = RC_Start(Deadband,Gain,Limit,ChIndex);
}
//******************************************************************************//

void Yaw_Start	(float Deadband, float Gain, float Limit, int16 ChIndex)
{
	if(Yaw != NULL) free(Roll);
	Yaw = RC_Start(Deadband,Gain,Limit,ChIndex);
}
//******************************************************************************//

void Height_Start	(float Deadband, float Gain, float Limit, int16 ChIndex)
{
	if(Height != NULL) free(Roll);
	Height = RC_Start(Deadband,Gain,Limit,ChIndex);
}
//******************************************************************************//

RC_Channel RC_Start	(float Deadband, float Gain, float Limit, int16 ChIndex)
{
RC_Channel CurrentChannel;
	CurrentChannel = (void *)(sizeof(struct sRC_Channel));
	CurrentChannel->DeadBand = Deadband;
	CurrentChannel->Gain = Gain;
	CurrentChannel->Limit = Limit;
	CurrentChannel->ChannelIndex = ChIndex;
	return CurrentChannel;
}
//******************************************************************************//


float RC_GetValue(RC_Channel RCChannel)
{
float Result = Channel[RCChannel->ChannelIndex];
	if(RCChannel != Height)
	{
		Result = Channel[RCChannel->ChannelIndex] +1 -500;		// Convert to +- 500 units
	}
	Result *= RCChannel->Gain;
	if(abs(Result) <= RCChannel->DeadBand) Result = 0.0;	// Set the deadband
	if(abs(Result) >= RCChannel->Limit) Result = RCChannel->Limit * sign(Result);
	return Result;
}
//******************************************************************************//


