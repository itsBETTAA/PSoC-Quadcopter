/******************************************************************************
SparkFunBQ27441.cpp
BQ27441 Arduino Library Main Source File
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Implementation of all features of the BQ27441 LiPo Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/

#include "project.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "batteryMonitor.h"
#include <debSerial.h>
#include "BQ27441_Definitions.h"

 //================================================================================================================================================//

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/*****************************************************************************
 ***************Initialization static (private) Functions ********************
 *****************************************************************************/

						 // entering/exiting config

/**
	    Check if the BQ27441-G1A is sealed or not.
		
		@return true if the chip is sealed
	*/
static bool sealed(void);

/**
	    Seal the BQ27441-G1A
		
		@return true on success
	*/
static bool seal(void);

/**
	    UNseal the BQ27441-G1A
		
		@return true on success
	*/
static bool unseal(void);

/**
	    Read the 16-bit opConfig register from extended data
		
		@return opConfig register contents
	*/
static uint16_t opConfig(void);

/**
	    Write the 16-bit opConfig register in extended data
		
		@param New 16-bit value for opConfig
		@return true on success
	*/
static bool writeOpConfig(uint16_t value);

/**
	    Issue a soft-reset to the BQ27441-G1A
		
		@return true on success
	*/
static bool softReset(void);

/**
	    Read a 16-bit command word from the BQ27441-G1A
		
		@param subAddress is the command to be read from
		@return 16-bit value of the command's contents
	*/
static uint16_t readWord(uint16_t subAddress);

/**
	    Read a 16-bit subcommand() from the BQ27441-G1A's control()
		
		@param function is the subcommand of control() to be read
		@return 16-bit value of the subcommand's contents
	*/
static uint16_t readControlWord(uint16_t function);

/**
	    Execute a subcommand() from the BQ27441-G1A's control()
		
		@param function is the subcommand of control() to be executed
		@return true on success
	*/
static bool executeControlWord(uint16_t function);

////////////////////////////
// Extended Data Commands //
////////////////////////////
/**
	    Issue a BlockDataControl() command to enable BlockData access
		
		@return true on success
	*/
static bool blockDataControl(void);

/**
	    Issue a DataClass() command to set the data class to be accessed
		
		@param id is the id number of the class
		@return true on success
	*/
static bool blockDataClass(uint8_t id);

/**
	    Issue a DataBlock() command to set the data block to be accessed
		
		@param offset of the data block
		@return true on success
	*/
static bool blockDataOffset(uint8_t offset);

/**
	    Read the current checksum using BlockDataCheckSum()
		
		@return true on success
	*/
static uint8_t blockDataChecksum(void);

/**
	    Use BlockData() to read a byte from the loaded extended data
		
		@param offset of data block byte to be read
		@return true on success
	*/
static uint8_t readBlockData(uint8_t offset);

/**
	    Use BlockData() to write a byte to an offset of the loaded data
		
		@param offset is the position of the byte to be written
		       data is the value to be written
		@return true on success
	*/
static bool writeBlockData(uint8_t offset, uint8_t data);

/**
	    Read all 32 bytes of the loaded extended data and compute a 
		checksum based on the values.
		
		@return 8-bit checksum value calculated based on loaded data
	*/
static uint8_t computeBlockChecksum(void);

/**
	    Use the BlockDataCheckSum() command to write a checksum value
		
		@param csum is the 8-bit checksum to be written
		@return true on success
	*/
static bool writeBlockChecksum(uint8_t csum);

/**
	    Read a byte from extended data specifying a class ID and position offset
		
		@param classID is the id of the class to be read from
		       offset is the byte position of the byte to be read
		@return 8-bit value of specified data
	*/
static uint8_t readExtendedData(uint8_t classID, uint8_t offset);

/**
	    Write a specified number of bytes to extended data specifying a 
		class ID, position offset.
		
		@param classID is the id of the class to be read from
		       offset is the byte position of the byte to be read
			   data is the data buffer to be written
			   len is the number of bytes to be written
		@return true on success
	*/
static bool writeExtendedData(uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len);

/////////////////////////////////
// I2C Read and Write Routines //
/////////////////////////////////

/**
	    Read a specified number of bytes over I2C at a given subAddress
		
		@param subAddress is the 8-bit address of the data to be read
		       dest is the data buffer to be written to
			   count is the number of bytes to be read
		@return true on success
	*/
static int16_t i2cReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count);

/**
	    Write a specified number of bytes over I2C to a given subAddress
		
		@param subAddress is the 8-bit address of the data to be written to
		       src is the data buffer to be written
			   count is the number of bytes to be written
		@return true on success
	*/
static uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t *src, uint8_t count);

//================================================================================================================================================//

/*****************************************************************************
 ************************** Initialization Functions *************************
 *****************************************************************************/
// Initializes variables


static uint8_t _deviceAddress = BQ72441_I2C_ADDRESS;  // Stores the BQ27441-G1A's I2C address
static bool _sealFlag = false;			 // Global to identify that IC was previously sealed
static bool _userConfigControl = false; // Global to identify that user has control over

// Initializes I2C and verifies communication with the BQ27441.
bool lipo_begin(void)
{
	uint16_t deviceID = 0;

	I2C_Start(); // Initialize I2C master

	deviceID = lipo_deviceType(); // Read deviceType from BQ27441

	if (deviceID == BQ27441_DEVICE_ID)
	{
		return true; // If device ID is valid, return true
	}

	return false; // Otherwise return false
}

// Configures the design capacity of the connected battery.
bool lipo_setCapacity(uint16_t capacity)
{
	// Write to STATE subclass (82) of BQ27441 extended memory.
	// Offset 0x0A (10)
	// Design capacity is a 2-byte piece of data - MSB first
	uint8_t capMSB = capacity >> 8;
	uint8_t capLSB = capacity & 0x00FF;
	uint8_t capacityData[2] = {capMSB, capLSB};
	return writeExtendedData(BQ27441_ID_STATE, 10, capacityData, 2);
}

/*****************************************************************************
 ********************** Battery Characteristics Functions ********************
 *****************************************************************************/

// Reads and returns the battery voltage
uint16_t lipo_voltage(void)
{
	return readWord(BQ27441_COMMAND_VOLTAGE);
}

// Reads and returns the specified current measurement
current_measure current_measure_type = AVG;
void set_lipo_current_arg(current_measure userInput){
	current_measure_type  = userInput;
}

int16_t lipo_current()
{
	int16_t current = 0;
	switch (current_measure_type)
	{
	case AVG:
		current = (int16_t)readWord(BQ27441_COMMAND_AVG_CURRENT);
		break;
	case STBY:
		current = (int16_t)readWord(BQ27441_COMMAND_STDBY_CURRENT);
		break;
	case MAX:
		current = (int16_t)readWord(BQ27441_COMMAND_MAX_CURRENT);
		break;
	}

	return current;
}

// Reads and returns the specified capacity measurement
capacity_measure capacity_measure_type = REMAIN;
void set_lipo_capacity_arg(capacity_measure userInput){
	capacity_measure_type = userInput;
}
uint16_t lipo_capacity()
{
	uint16_t capacity = 0;
	switch (capacity_measure_type)
	{
	case REMAIN:
		return readWord(BQ27441_COMMAND_REM_CAPACITY);
		break;
	case FULL:
		return readWord(BQ27441_COMMAND_FULL_CAPACITY);
		break;
	case AVAIL:
		capacity = readWord(BQ27441_COMMAND_NOM_CAPACITY);
		break;
	case AVAIL_FULL:
		capacity = readWord(BQ27441_COMMAND_AVAIL_CAPACITY);
		break;
	case REMAIN_F:
		capacity = readWord(BQ27441_COMMAND_REM_CAP_FIL);
		break;
	case REMAIN_UF:
		capacity = readWord(BQ27441_COMMAND_REM_CAP_UNFL);
		break;
	case FULL_F:
		capacity = readWord(BQ27441_COMMAND_FULL_CAP_FIL);
		break;
	case FULL_UF:
		capacity = readWord(BQ27441_COMMAND_FULL_CAP_UNFL);
		break;
	case DESIGN:
		capacity = readWord(BQ27441_EXTENDED_CAPACITY);
	}

	return capacity;
}

// Reads and returns measured average power
int16_t lipo_power(void)
{
	return (int16_t)readWord(BQ27441_COMMAND_AVG_POWER);
}

// Reads and returns specified state of charge measurement
soc_measure soc_measure_type = FILTERED;
void set_lipo_soc_arg(soc_measure userInput){
	soc_measure_type = userInput;
}
uint16_t lipo_soc()
{
	uint16_t socRet = 0;
	switch (soc_measure_type)
	{
	case FILTERED:
		socRet = readWord(BQ27441_COMMAND_SOC);
		break;
	case UNFILTERED:
		socRet = readWord(BQ27441_COMMAND_SOC_UNFL);
		break;
	}

	return socRet;
}

// Reads and returns specified state of health measurement
soh_measure soh_measure_type = PERCENT;
void set_lipo_soh_arg(soh_measure userInput){
	soh_measure_type = userInput;
}
uint8_t lipo_soh()
{
	uint16_t sohRaw = readWord(BQ27441_COMMAND_SOH);
	uint8_t sohStatus = sohRaw >> 8;
	uint8_t sohPercent = sohRaw & 0x00FF;

	if (soh_measure_type == PERCENT)
		return sohPercent;
	else
		return sohStatus;
}

// Reads and returns specified temperature measurement
temp_measure temp_measure_type = BATTERY;
void set_lipo_temperature_arg(temp_measure userInput){
	temp_measure_type = userInput;
}
uint16_t lipo_temperature()
{
	uint16_t temp = 0;
	switch (temp_measure_type)
	{
	case BATTERY:
		temp = readWord(BQ27441_COMMAND_TEMP);
		break;
	case INTERNAL_TEMP:
		temp = readWord(BQ27441_COMMAND_INT_TEMP);
		break;
	}
	return temp;
}

/*****************************************************************************
 ************************** GPOUT Control Functions **************************
 *****************************************************************************/
// Get GPOUT polarity setting (active-high or active-low)
bool lipo_GPOUTPolarity(void)
{
	uint16_t opConfigRegister = opConfig();

	return (opConfigRegister & BQ27441_OPCONFIG_GPIOPOL);
}

// Set GPOUT polarity to active-high or active-low
bool lipo_setGPOUTPolarity(bool activeHigh)
{
	uint16_t oldOpConfig = opConfig();

	// Check to see if we need to update opConfig:
	if ((activeHigh && (oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)) ||
		(!activeHigh && !(oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)))
		return true;

	uint16_t newOpConfig = oldOpConfig;
	if (activeHigh)
		newOpConfig |= BQ27441_OPCONFIG_GPIOPOL;
	else
		newOpConfig &= ~(BQ27441_OPCONFIG_GPIOPOL);

	return writeOpConfig(newOpConfig);
}

// Get GPOUT function (BAT_LOW or SOC_INT)
bool lipo_GPOUTFunction(void)
{
	uint16_t opConfigRegister = opConfig();

	return (opConfigRegister & BQ27441_OPCONFIG_BATLOWEN);
}

// Set GPOUT function to BAT_LOW or SOC_INT
bool lipo_setGPOUTFunction(gpout_function function)
{
	uint16_t oldOpConfig = opConfig();

	// Check to see if we need to update opConfig:
	if ((function && (oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)) ||
		(!function && !(oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)))
		return true;

	// Modify BATLOWN_EN bit of opConfig:
	uint16_t newOpConfig = oldOpConfig;
	if (function)
		newOpConfig |= BQ27441_OPCONFIG_BATLOWEN;
	else
		newOpConfig &= ~(BQ27441_OPCONFIG_BATLOWEN);

	// Write new opConfig
	return writeOpConfig(newOpConfig);
}

// Get SOC1_Set Threshold - threshold to set the alert flag
uint8_t lipo_SOC1SetThreshold(void)
{
	return readExtendedData(BQ27441_ID_DISCHARGE, 0);
}

// Get SOC1_Clear Threshold - threshold to clear the alert flag
uint8_t lipo_SOC1ClearThreshold(void)
{
	return readExtendedData(BQ27441_ID_DISCHARGE, 1);
}

// Set the SOC1 set and clear thresholds to a percentage
bool lipo_setSOC1Thresholds(uint8_t set, uint8_t clear)
{
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(BQ27441_ID_DISCHARGE, 0, thresholds, 2);
}

// Get SOCF_Set Threshold - threshold to set the alert flag
uint8_t lipo_SOCFSetThreshold(void)
{
	return readExtendedData(BQ27441_ID_DISCHARGE, 2);
}

// Get SOCF_Clear Threshold - threshold to clear the alert flag
uint8_t lipo_SOCFClearThreshold(void)
{
	return readExtendedData(BQ27441_ID_DISCHARGE, 3);
}

// Set the SOCF set and clear thresholds to a percentage
bool lipo_setSOCFThresholds(uint8_t set, uint8_t clear)
{
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(BQ27441_ID_DISCHARGE, 2, thresholds, 2);
}

// Check if the SOC1 flag is set
bool lipo_socFlag(void)
{
	uint16_t flagState = lipo_flags();

	return flagState & BQ27441_FLAG_SOC1;
}

// Check if the SOCF flag is set
bool lipo_socfFlag(void)
{
	uint16_t flagState = lipo_flags();

	return flagState & BQ27441_FLAG_SOCF;
}

// Get the SOC_INT interval delta
uint8_t lipo_sociDelta(void)
{
	return readExtendedData(BQ27441_ID_STATE, 26);
}

// Set the SOC_INT interval delta to a value between 1 and 100
bool lipo_setSOCIDelta(uint8_t delta)
{
	uint8_t soci = constrain(delta, 0, 100);
	return writeExtendedData(BQ27441_ID_STATE, 26, &soci, 1);
}

// Pulse the GPOUT pin - must be in SOC_INT mode
bool lipo_pulseGPOUT(void)
{
	return executeControlWord(BQ27441_CONTROL_PULSE_SOC_INT);
}

/*****************************************************************************
 *************************** Control Sub-Commands ****************************
 *****************************************************************************/

// Read the device type - should be 0x0421
uint16_t lipo_deviceType(void)
{
	return readControlWord(BQ27441_CONTROL_DEVICE_TYPE);
}

// Enter configuration mode - set userControl if calling from an Arduino sketch
// and you want control over when to exitConfig
bool userControl = true;
void set_lipo_enterConfig_arg(bool userInput){
	userControl = userInput;
}
bool lipo_enterConfig()
{
	if (userControl)
		_userConfigControl = true;

	if (sealed())
	{
		_sealFlag = true;
		unseal(); // Must be unsealed before making changes
	}

	if (executeControlWord(BQ27441_CONTROL_SET_CFGUPDATE))
	{
		int16_t timeout = BQ72441_I2C_TIMEOUT;
		while ((timeout--) && (!(lipo_status() & BQ27441_FLAG_CFGUPMODE)))
			CyDelay(1);

		if (timeout > 0)
			return true;
	}

	return false;
}

// Exit configuration mode with the option to perform a resimulation
bool resim = true;
void set_lipo_exitConfig_arg(bool userInput){
	resim = userInput;
}
bool lipo_exitConfig()
{
	// There are two methods for exiting config mode:
	//    1. Execute the EXIT_CFGUPDATE command
	//    2. Execute the SOFT_RESET command
	// EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
	// measurement, and without resimulating to update unfiltered-SoC and SoC.
	// If a new OCV measurement or resimulation is desired, SOFT_RESET or
	// EXIT_RESIM should be used to exit config mode.
	if (resim)
	{
		if (softReset())
		{
			int16_t timeout = BQ72441_I2C_TIMEOUT;
			while ((timeout--) && ((lipo_flags() & BQ27441_FLAG_CFGUPMODE)))
				CyDelay(1);
			if (timeout > 0)
			{
				if (_sealFlag)
					seal(); // Seal back up if we IC was sealed coming in
				return true;
			}
		}
		return false;
	}
	else
	{
		return executeControlWord(BQ27441_CONTROL_EXIT_CFGUPDATE);
	}
}

// Read the flags() command
uint16_t lipo_flags(void)
{
	return readWord(BQ27441_COMMAND_FLAGS);
}

// Read the CONTROL_STATUS subcommand of control()
uint16_t lipo_status(void)
{
	return readControlWord(BQ27441_CONTROL_STATUS);
}

/***************************** Private Functions *****************************/

// Check if the BQ27441-G1A is sealed or not.
static bool sealed(void)
{
	uint16_t stat = lipo_status();
	return (stat & BQ27441_STATUS_SS);
}

// Seal the BQ27441-G1A
static bool seal(void)
{
	return readControlWord(BQ27441_CONTROL_SEALED);
}

// Unseal the BQ27441-G1A
static bool unseal(void)
{
	// To unseal the BQ27441, write the key to the control
	// command. Then immediately write the same key to control again.
	if (readControlWord(BQ27441_UNSEAL_KEY))
	{
		return readControlWord(BQ27441_UNSEAL_KEY);
	}
	return false;
}

// Read the 16-bit opConfig register from extended data
static uint16_t opConfig(void)
{
	return readWord(BQ27441_EXTENDED_OPCONFIG);
}

// Write the 16-bit opConfig register in extended data
static bool writeOpConfig(uint16_t value)
{
	uint8_t opConfigMSB = value >> 8;
	uint8_t opConfigLSB = value & 0x00FF;
	uint8_t opConfigData[2] = {opConfigMSB, opConfigLSB};

	// OpConfig register location: BQ27441_ID_REGISTERS id, offset 0
	return writeExtendedData(BQ27441_ID_REGISTERS, 0, opConfigData, 2);
}

// Issue a soft-reset to the BQ27441-G1A
static bool softReset(void)
{
	return executeControlWord(BQ27441_CONTROL_SOFT_RESET);
}

// Read a 16-bit command word from the BQ27441-G1A
static uint16_t readWord(uint16_t subAddress)
{
	uint8_t data[2];
	i2cReadBytes(subAddress, data, 2);
	return ((uint16_t)data[1] << 8) | data[0];
}

// Read a 16-bit subcommand() from the BQ27441-G1A's control()
static uint16_t readControlWord(uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	uint8_t data[2] = {0, 0};

	i2cWriteBytes((uint8_t)0, command, 2);

	if (i2cReadBytes((uint8_t)0, data, 2))
	{
		return ((uint16_t)data[1] << 8) | data[0];
	}

	return false;
}

// Execute a subcommand() from the BQ27441-G1A's control()
static bool executeControlWord(uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	//uint8_t data[2] = {0, 0};

	if (i2cWriteBytes((uint8_t)0, command, 2))
		return true;

	return false;
}

/*****************************************************************************
 ************************** Extended Data Commands ***************************
 *****************************************************************************/

// Issue a BlockDataControl() command to enable BlockData access
static bool blockDataControl(void)
{
	uint8_t enableByte = 0x00;
	return i2cWriteBytes(BQ27441_EXTENDED_CONTROL, &enableByte, 1);
}

// Issue a DataClass() command to set the data class to be accessed
static bool blockDataClass(uint8_t id)
{
	return i2cWriteBytes(BQ27441_EXTENDED_DATACLASS, &id, 1);
}

// Issue a DataBlock() command to set the data block to be accessed
static bool blockDataOffset(uint8_t offset)
{
	return i2cWriteBytes(BQ27441_EXTENDED_DATABLOCK, &offset, 1);
}

// Read the current checksum using BlockDataCheckSum()
static uint8_t blockDataChecksum(void)
{
	uint8_t csum;
	i2cReadBytes(BQ27441_EXTENDED_CHECKSUM, &csum, 1);
	return csum;
}

// Use BlockData() to read a byte from the loaded extended data
static uint8_t readBlockData(uint8_t offset)
{
	uint8_t ret;
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	i2cReadBytes(address, &ret, 1);
	return ret;
}

// Use BlockData() to write a byte to an offset of the loaded data
static bool writeBlockData(uint8_t offset, uint8_t data)
{
	uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
	return i2cWriteBytes(address, &data, 1);
}

// Read all 32 bytes of the loaded extended data and compute a
// checksum based on the values.
static uint8_t computeBlockChecksum(void)
{
	uint8_t data[32];
	i2cReadBytes(BQ27441_EXTENDED_BLOCKDATA, data, 32);

	uint8_t csum = 0;
    int i = 0;
	for (i = 0; i < 32; i++)
	{
		csum += data[i];
	}
	csum = 255 - csum;

	return csum;
}

// Use the BlockDataCheckSum() command to write a checksum value
static bool writeBlockChecksum(uint8_t csum)
{
	return i2cWriteBytes(BQ27441_EXTENDED_CHECKSUM, &csum, 1);
}

// Read a byte from extended data specifying a class ID and position offset
static uint8_t readExtendedData(uint8_t classID, uint8_t offset)
{
	uint8_t retData = 0;
	if (!_userConfigControl){
        userControl = false;    
		lipo_enterConfig();
    }
    
	if (!blockDataControl())	  // // enable block data memory control
		return false;			  // Return false if enable fails
	if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
		return false;

	blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)

	computeBlockChecksum(); // Compute checksum going in
	uint8_t oldCsum = blockDataChecksum();
	/*for (int i=0; i<32; i++)
		Serial.print(String(readBlockData(i)) + " ");*/
	retData = readBlockData(offset % 32); // Read from offset (limit to 0-31)

	if (!_userConfigControl)
		lipo_exitConfig();

	return retData;
}

// Write a specified number of bytes to extended data specifying a
// class ID, position offset.
static bool writeExtendedData(uint8_t classID, uint8_t offset, uint8_t *data, uint8_t len)
{
	serial_println("MADE 1");
	if (len > 32)
		return false;

	if (!_userConfigControl){
	    userControl = false;
        lipo_enterConfig();
    }
    serial_println("MADE 2");
	if (!blockDataControl())	  // // enable block data memory control
		return false;			  // Return false if enable fails
	if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
		return false;

	blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)
	computeBlockChecksum();		  // Compute checksum going in
	uint8_t oldCsum = blockDataChecksum();
	serial_println("MADE 3");
	// Write data bytes:
    int i = 0;
	for (i = 0; i < len; i++)
	{
		// Write to offset, mod 32 if offset is greater than 32
		// The blockDataOffset above sets the 32-bit block
		writeBlockData((offset % 32) + i, data[i]);
	}
	serial_println("MADE 4");
	// Write new checksum using BlockDataChecksum (0x60)
	uint8_t newCsum = computeBlockChecksum(); // Compute the new checksum
	writeBlockChecksum(newCsum);
	serial_println("MADE 5");
	if (!_userConfigControl)
		lipo_exitConfig();
	serial_println("MADE 6");
	return true;
}

/*****************************************************************************
 ************************ I2C Read and Write Routines ************************
 *****************************************************************************/

// Read a specified number of bytes over I2C at a given subAddress
static int16_t i2cReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count)
{
	uint8_t _buffer[256] = {0}; //256 because count is an 8 bit number which means that the highest number it can be is 256
	int16_t timeout = BQ72441_I2C_TIMEOUT;
	I2C_I2CMasterSendStart(_deviceAddress, _write); //Wire.beginTransmission(_deviceAddress);
	I2C_I2CMasterWriteByte(subAddress);             //Wire.write(subAddress);
	I2C_I2CMasterSendStop();                        //Wire.endTransmission(true);

	 I2C_I2CMasterReadBuf(_deviceAddress, _buffer, count, I2C_I2C_MODE_COMPLETE_XFER); //Wire.requestFrom(_deviceAddress, count);
	/*
	I2C_I2CMasterGetReadBufSize();
	Returns the number of bytes that has been transferred with the I2C_I2CMasterReadBuf() function.
	*/
	while ((I2C_I2CMasterGetReadBufSize() < count) && timeout--){
		CyDelay(1);
	}
	if (timeout)
	{
        int i = 0;
		for (i = 0; i < count; i++)
		{
			dest[i] = _buffer[i]; //Wire.read();
		}
	}

	return timeout;
}

// Write a specified number of bytes over I2C to a given subAddress
static uint16_t i2cWriteBytes(uint8_t subAddress, uint8_t *src, uint8_t count)
{
	/*
	-Generates Start condition and sends slave address with read/write bit. Disables the I2C interrupt.
	-This function is blocking. It does not return until the Start condition and address byte are sent, 
	 a ACK/NAK is received, or errors have occurred.
	*/
	I2C_I2CMasterSendStart(_deviceAddress, _write); //Wire.beginTransmission(_deviceAddress);

	/*
	-Sends one byte to a slave.
	-This function is blocking and does not return until the byte is transmitted or an error occurs.
	*/
	I2C_I2CMasterWriteByte(subAddress); 			//Wire.write(subAddress);

	/*
	-Sends multiple byte to a slave.
	-The function is blocking and does not return until the byte is transmitted or an error occurs.
	*/
    int i = 0;
    for (i = 0; i < count; i++)
	{
		I2C_I2CMasterWriteByte(src[i]);				//Wire.write(src[i]);
	}

	/*
	-Generates Stop condition on the bus. The NAK is generated before Stop in case of a read transaction.
	-At least one byte has to be read if a Start or ReStart condition with read direction was generated before.
	-This function is blocking and does not return until a Stop condition is generated or error occurred.
	*/
	I2C_I2CMasterSendStop(); 						//Wire.endTransmission(true)
    
	return true;
}