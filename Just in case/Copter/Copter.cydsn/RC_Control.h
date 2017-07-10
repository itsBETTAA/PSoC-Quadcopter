//******************************************************************************//
//																				//
//		Project	:	Quad Copter													//
//		Device	:	PSoC 4														//
//		Version :	0.0															//
//		Date	:	March 2014													//
//		Author(s)	Bob Marlowe													//
//																				//
//******************************************************************************//
#ifndef RC_Control_h
#define RC_Control_h

//******************************************************************************//
//********************* Libraries **********************************************//
//******************************************************************************//
#include <GlobalDefs.h>
#include <DeMPX.h>

//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//

//******************************************************************************//
//******************** Structures / Enums **************************************//
//******************************************************************************//
struct sRC_Channel	{
						float DeadBand;					// No reaction within +-Deadband
						float Gain;						// Gain for this channel
						float Limit;					// +- Maximum value
						int16 ChannelIndex;				// Associated channel number
					};

typedef struct sRC_Channel * RC_Channel;

//******************************************************************************//
//********************* Data Area **********************************************//
//******************************************************************************//
extern RC_Channel Roll;									// Basic channels to control a flying object
extern RC_Channel Pitch;
extern RC_Channel Yaw;
extern RC_Channel Height;

//******************************************************************************//
//********************* APIs ***************************************************//
//******************************************************************************//
void Roll_Start		(float Deadband, float Gain, float Limit, int16 ChIndex);
void Pitch_Start	(float Deadband, float Gain, float Limit, int16 ChIndex);
void Yaw_Start		(float Deadband, float Gain, float Limit, int16 ChIndex);
void Height_Start	(float Deadband, float Gain, float Limit, int16 ChIndex);
RC_Channel RC_Start	(float Deadband, float Gain, float Limit, int16 ChIndex);

float RC_GetValue(RC_Channel RCChannel);

#endif
