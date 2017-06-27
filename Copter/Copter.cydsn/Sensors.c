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
//	The sensor-board is a Pololu MinIMU - 9 breakout board						//
//	http://www.pololu.com/product/1268											//
//	which contains links to both sensors										//
//																				//
//	The sensors can be red with I2C directly, but the program works a bit		//
//	more sophistikated. The Gyro is able to deliver values at a higher			//
//	frequency and store them into a FIFO which then can be red via I2C			//
//	Averaging those values will reduce noise which can be disturbing when the	//
//	props are turning and inject some vibrations.								//
//																				//
//	The accellerometer has not got a fifo. To collect as much values within		//
//	one cycle a trick is used: When waiting for any event to occur a function	//
//	named Wait() is called. This function checks whether a value is ready and	//
//	returns immediately if not. A ready converted value is retrieved and added	//
//	and when the accelleration is retrieved becomes averaged. 					//
//																				//
//	The compass-values are converted at a comparable low frequency and so do	//
//	not need to be averaged.																			//
//																				//
//																				//
//																				//
//******************************************************************************//

//******************************************************************************//
//********************* Libraries **********************************************//
//******************************************************************************//
#include <GlobalDefs.h>
#include <project.h>
#include <CyLib.h>
#include <math.h>
//******************************************************************************//
//********************* Definitions & Macros ***********************************//
//******************************************************************************//
#define I2C_READ		1						// Some I2C constants
#define I2C_WRITE		0

#define Accell_I2C	 		0x19				//	Slave addresses Accellerometer
#define Magnet_I2C	    	0x1e				//	Compass
#define Gyro_I2C		 	0x6b				//	Gyro

#define gravity				9.81				//	g = 9.81 m/s²

//************* L3G4200D Register Definitions **********************************
#define TempRegisterGyro 	0x26
#define	WhoAmIGyro			0x0f
#define	CtlReg1Gyro			0x20
#define	CtlReg2Gyro			0x21
#define	CtlReg3Gyro			0x22
#define	CtlReg4Gyro			0x23
#define	CtlReg5Gyro			0x24
#define AchseX_Gyro 	 	0x28
#define AchseY_Gyro   		0x2A
#define AchseZ_Gyro   		0x2C
#define Status_Reg_Gyro     0x27
#define	FIFO_CtlReg_Gyro	0x2e
#define FIFO_StatReg_Gyro	0x2f
#define DataReadyBit_Gyro	3

#define GyroScaling			(17.0 / 1000 * 4.0 * atan(1.0) / 360.0)	// Factor for scaling to rad/s

#define DataReadyBit		3
#define DataReadyBit_Mag 	0
#define DataOverwritten 	7

//***** LSM303DLHC Register Definitions ********
#define TempRegister 	0x26
#define	WhoAmI			0x0f
#define	CtlReg1			0x20
#define	CtlReg2			0x21
#define	CtlReg3			0x22
#define	CtlReg4			0x23
#define	CtlReg5			0x24
#define	HP				0x26
#define StatusReg       0x27
#define AchseX  	 	0x28
#define AchseY   		0x2A
#define AchseZ   		0x2C

//****** LSM303 Kompass********

#define CRA_REG_M 		0x00
#define CRB_REG_M       0x01
#define MR_REG_M        0x02

#define mag_x			0x03
#define mag_y			0x05
#define mag_z			0x07

#define SR_REG_M			0x09


//******************************************************************************//
//********************* Data Area **********************************************//
//******************************************************************************//
float Gyro_X, Gyro_Y, Gyro_Z;						// Sensor values as floats
float Magneto_X, Magneto_Y, Magneto_Z;
float Accellmeter_X, Accellmeter_Y, Accellmeter_Z;

int32 ResponseX_Gyro;								// Sensor values as int32
int32 ResponseY_Gyro;
int32 ResponseZ_Gyro;

int32 ResponseX_Mag;
int32 ResponseY_Mag;
int32 ResponseZ_Mag;

int32 ResponseX_Acc;
int32 ResponseY_Acc;
int32 ResponseZ_Acc;

int32 G_OffsetX;								// Measured Gyro Offsets
int32 G_OffsetZ;
int32 G_OffsetY;

int32 A_OffsetX;								// Measured Accellerometer Offsets
int32 A_OffsetZ;
int32 A_OffsetY;
uint16 ACCSampleCount;							// Counter for Average
float g_Factor;									// Conversion factor for counts to m/s²

//******************************************************************************//
//********************* Prototypes **********************************************//
//******************************************************************************//
void Magnet_GetAverage(uint16 Number);
void InitializeGyroReadings(void);				
void AddAccTuple(void);
void SetAccOffsets(uint16);
void SetGyroOffsets(uint16);
//******************************************************************************//
//********************* Program Area *******************************************//
//******************************************************************************//

void I2C_PreAmble(uint8 Device, uint8 Register)			// Setup I2C bus for a ReadByte()
{
	I2C_I2CMasterSendStart(Device, I2C_WRITE);
	I2C_I2CMasterWriteByte(Register);
	I2C_I2CMasterSendRestart(Device,I2C_READ);
}
//******************************************************************************//

void WriteGeberByte(uint8 I2C_Device, uint8 Register, uint8 Value)
{														// Write a byte to I2C
	I2C_I2CMasterSendStart(I2C_Device, I2C_WRITE);
	I2C_I2CMasterWriteByte(Register);
	I2C_I2CMasterWriteByte(Value);
	I2C_I2CMasterSendStop();
}
//******************************************************************************//

uint8 ReadGeberByte(uint8 I2C_Device, uint8 Register)	// Read a single Byte from I2C
{
uint8 RetValue;	
	I2C_I2CMasterSendStart(I2C_Device, I2C_WRITE);
	I2C_I2CMasterWriteByte(Register);
	I2C_I2CMasterSendRestart(I2C_Device,I2C_READ);
	RetValue = I2C_I2CMasterReadByte(I2C_I2C_NAK_DATA);
	I2C_I2CMasterSendStop();
	return RetValue;
}
//******************************************************************************//

void Wait(void)											// Retrieve an ACC-value if ready
{
	if(!IsBitSet(ReadGeberByte(Accell_I2C,StatusReg),DataReadyBit)) return;
	
	AddAccTuple();
	
}
//******************************************************************************//

void Accell_Start()										// Initialization 
{
	WriteGeberByte(Accell_I2C,0x20,				0b01110111);	// Reg 0: Power up Beschleunigungs-Geber, 400Hz 
	WriteGeberByte(Accell_I2C,0x23,				0b10000000);	// Reg 4: Enable Block Write
	
	SetAccOffsets(250);											// Calculate offset with 250 values
}
//******************************************************************************//

void Magnet_Start()										// Initialization
{
	WriteGeberByte(Magnet_I2C,0x00,				0b10011000);	// Reg 0: Enable Temperature; 75 Hz, Normal
	WriteGeberByte(Magnet_I2C,0x01,				0b00100000);	// Reg 1: Higest Gain
	WriteGeberByte(Magnet_I2C,0x02,				0b00000000);	// Reg 2: Mode: Continuous Conversion
	
	Magnet_GetAverage(50);
}
//******************************************************************************//

void Gyro_Start(void)									// Initialization
{
	WriteGeberByte(Gyro_I2C,CtlReg1Gyro,		0b11011111);	// Standard 760 Smpls/s, 30Hz LPF
	WriteGeberByte(Gyro_I2C,FIFO_CtlReg_Gyro,	0b01001111);	// Streaming mode,  Watermark set to 15
	WriteGeberByte(Gyro_I2C,CtlReg4Gyro,		0b10010000);	// Block Mode Scale 500°/s
	WriteGeberByte(Gyro_I2C,CtlReg5Gyro,		0b01000000);	// Enable FIFO
	
	SetGyroOffsets(500);										// Calculate offset with 500 measures
}
//******************************************************************************//

void Sensors_Start(void)								// Initialize and ready all sensors
{
uint8 Me;
	I2C_Start();
	Accell_Start();
	Magnet_Start();
//******************************************************************************//
//	For debugging purposes: Me must contain 0x3d when we have the right chip	//
	Me = ReadGeberByte(Magnet_I2C,WhoAmI);
//******************************************************************************//
	Gyro_Start();
}
//******************************************************************************//

uint8 WaitForGyroData(void)								// Wait for at least 1 sample in FIFO
{
uint8 Count;
	while((Count = ReadGeberByte(Gyro_I2C,FIFO_StatReg_Gyro) & 0x1f) == 0)
	{
		Wait();  
	}
	return Count;										// Return number of samples in FIFO
}

//******************************************************************************//
void Gyro_Get3Tuples(uint8 Number)							// Read Values off the FIFO
{
uint8 datalow;
int16 datahigh;
int16 Intermed;
	I2C_PreAmble(Gyro_I2C, AchseX_Gyro | 0x80);			// High bit on signals reading multiple values from FIFO

	while(Number > 0)
	{
		
		Number--;

		datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
		datahigh	= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);	
		Intermed	= (datahigh << 8) | datalow;
		ResponseX_Gyro += (int32)Intermed - G_OffsetX;	

		datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
		datahigh	= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);	
		Intermed	= (datahigh << 8) | datalow;
		ResponseY_Gyro += (int32)Intermed - G_OffsetY;	

		datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
		datahigh	= I2C_I2CMasterReadByte(Number > 0?I2C_I2C_ACK_DATA:I2C_I2C_NAK_DATA);	
		Intermed	= (datahigh << 8) | datalow;
		ResponseZ_Gyro += (int32)Intermed - G_OffsetZ;	
	}
	I2C_I2CMasterSendStop();
}
//******************************************************************************//

void InitializeGyroReadings(void)
{
	G_OffsetX = 0;
	G_OffsetY = 0;
	G_OffsetZ = 0;
	ResponseX_Gyro = 0;
	ResponseY_Gyro = 0;
	ResponseZ_Gyro = 0;
}
//******************************************************************************//

void SetGyroOffsets(uint16 SampleCount)					// Build the average from SampleCount values
{
uint16 SumSamples = 0;
uint16 Actual;
	InitializeGyroReadings();
	while(SumSamples < SampleCount)
	{
		SumSamples += Actual = WaitForGyroData();
		if(Actual)
		{
			Gyro_Get3Tuples(Actual);
		}
		else CyDelay(10);
	}
	SampleCount = SumSamples;
	G_OffsetX = ResponseX_Gyro / SampleCount;			// Now set the offsets accordingly
	G_OffsetY = ResponseY_Gyro / SampleCount;
	G_OffsetZ = ResponseZ_Gyro / SampleCount;
	ResponseX_Gyro = 0;									// and reset the readings
	ResponseY_Gyro = 0;
	ResponseZ_Gyro = 0;
}
//******************************************************************************//

void Gyro_GetValues(void)
{
uint8 nn;	
	nn = WaitForGyroData();								// Wait for at least one data tuple Ready
		
	Gyro_Get3Tuples(nn);										// Read FIFO and integrate

	Gyro_X = (float)ResponseX_Gyro * GyroScaling /  nn;	// Average integrated values
	Gyro_Y = (float)ResponseY_Gyro * GyroScaling /  nn;	// and scale to degrees / second
	Gyro_Z = (float)ResponseZ_Gyro * GyroScaling /  nn;
	
	ResponseX_Gyro = 0;									// Reset the integral of Gyro-values
	ResponseY_Gyro = 0;
	ResponseZ_Gyro = 0;
	
}
//******************************************************************************//

unsigned char MagDataReady(void)						// Return TRUE when magdata availlable
{
	return IsBitSet(ReadGeberByte(Magnet_I2C,SR_REG_M),DataReadyBit_Mag);
}
//******************************************************************************//

void GetMagTuple(void)									// Get the 3 mag-values
{
uint8 datalow;
int16 datahigh;
int16 Intermed;

	I2C_PreAmble(Magnet_I2C, mag_x | 0x80);				// Initialize read of all 3 Axes

	datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
	datahigh	= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);	
	Intermed	= (datahigh << 8) | datalow;
	ResponseX_Mag = Intermed;
	
	datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
	datahigh	= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);	
	Intermed	= (datahigh << 8) | datalow;
	ResponseY_Mag = Intermed;
	
	datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
	datahigh	= I2C_I2CMasterReadByte(I2C_I2C_NAK_DATA);	
	Intermed	= (datahigh << 8) | datalow;
	ResponseZ_Mag = Intermed;
	
	I2C_I2CMasterSendStop();
}
//******************************************************************************//

void Magnet_GetAverage(uint16 Number)	//Magnetfeldsensor
//******************************************************************************//
//	Reading 3 axes magnetometer Number values and averaging them				//
//******************************************************************************//
{
uint16 Count = 0;
	while(Count < Number)
	{
		while(!MagDataReady())
		{
			Wait();							// Warten bis Daten bereit
		}
		GetMagTuple();
		
		Magneto_X += (float)ResponseX_Mag;
		Magneto_Y += (float)ResponseY_Mag;
		Magneto_Z += (float)ResponseZ_Mag;
		
		Count++;
	}
	Magneto_X /= Number;
	Magneto_Y /= Number;
	Magneto_Z /= Number;
}
//******************************************************************************//

void Magnet_GetValues(void)				//Magnetfeldsensor
//******************************************************************************//
//	Reading 3 axes magnetometer.												//
//******************************************************************************//
{
	Magnet_GetAverage(5);
/*	
	while(!MagDataReady())
	{
		Wait();							// Warten bis Daten bereit
	}
	GetMagTuple();
	
	Magneto_X = (float)ResponseX_Mag;
	Magneto_Y = (float)ResponseY_Mag;
	Magneto_Z = (float)ResponseZ_Mag;
*/
}
//******************************************************************************//

void AddAccTuple(void)
{
//******************************************************************************//
//	Reading 3 axes accellerometer.												//
//******************************************************************************//
uint8 datalow;
int16 datahigh;
int16 Intermed;

	I2C_PreAmble(Accell_I2C, AchseX | 0x80);			// Initialize read of Axis. Upper bit allows multiple reads

	datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
	datahigh	= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);	
	Intermed	= (datahigh << 8) | datalow;
	ResponseX_Acc += (int32)Intermed - A_OffsetX;
	
	datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
	datahigh	= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);	
	Intermed	= (datahigh << 8) | datalow;
	ResponseY_Acc += (int32)Intermed - A_OffsetY;
	
	datalow		= I2C_I2CMasterReadByte(I2C_I2C_ACK_DATA);
	datahigh	= I2C_I2CMasterReadByte(I2C_I2C_NAK_DATA);	
	Intermed	= (datahigh << 8) | datalow;
	ResponseZ_Acc += (int32)Intermed;					// No Z- offset, scaled to 9.81 m/s²
	
	I2C_I2CMasterSendStop();
	ACCSampleCount++;
}
//******************************************************************************//

void InitializeAccReadings(void)
{
	A_OffsetX = 0;
	A_OffsetY = 0;
	A_OffsetZ = 0;
	ResponseX_Acc = 0;
	ResponseY_Acc = 0;
	ResponseZ_Acc = 0;
	ACCSampleCount = 0;
}
//******************************************************************************//

void SetAccOffsets(uint16 SamplesWanted)
{
//******************************************************************************//
//	In opposite to the gyro-readings we do not calculate an offset for the		//
//	Z-axis. Instead we assume this value to be exactly 9.81 m/s² and scale		//
//	the X and Y-axes accordingly												//	
//******************************************************************************//
	
	InitializeAccReadings();
	while(ACCSampleCount < SamplesWanted)			// Average at least SamplesWanted values
	{
		Wait();										// Wait() will actually read-off the data
	}
	A_OffsetX = ResponseX_Acc / ACCSampleCount;		// Build averages
	A_OffsetY = ResponseY_Acc / ACCSampleCount;
	A_OffsetZ = ResponseZ_Acc / ACCSampleCount;
	g_Factor = gravity / A_OffsetZ;					
	A_OffsetZ = 0;
	ResponseX_Acc = 0;
	ResponseY_Acc = 0;
	ResponseZ_Acc = 0;
	ACCSampleCount = 0;
}
//******************************************************************************//

void Accell_GetValues(void)
{
	while(!ACCSampleCount) Wait();			// Wait for at least 1 sample ready
	Accellmeter_X = (float)ResponseX_Acc * g_Factor / ACCSampleCount;
	Accellmeter_Y = (float)ResponseY_Acc * g_Factor / ACCSampleCount;
	Accellmeter_Z = (float)ResponseZ_Acc * g_Factor / ACCSampleCount;
	
	ResponseX_Acc = 0;						// Reset the measured averages
	ResponseY_Acc = 0;
	ResponseZ_Acc = 0;
	
	ACCSampleCount = 0;						// Reset the count of measures
}
//******************************************************************************//

void Sensors_GetAll(void)
{
	Accell_GetValues();	
	Magnet_GetValues();
	Gyro_GetValues();
}
//******************************************************************************//
