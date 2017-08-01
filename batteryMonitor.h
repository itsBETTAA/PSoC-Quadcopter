/******************************************************************************
SparkFunBQ27441.h
BQ27441 Arduino Library Main Header File
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Definition of the BQ27441 library, which implements all features of the
BQ27441 LiPo Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/

#ifndef batteryMonitor_h
#define batteryMonitor_h

#include "project.h"
#include "BQ27441_Definitions.h"

#define BQ72441_I2C_TIMEOUT 2000

#define _write I2C_I2C_WRITE_XFER_MODE
#define _read I2C_I2C_READ_XFER_MODE

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

typedef uint16_t bool; // set bool just another interchangable word for uint8_t

// Parameters for the current() function, to specify which current to read
typedef enum {
	AVG,  // Average Current (DEFAULT)
	STBY, // Standby Current
	MAX   // Max Current
} current_measure;

// Parameters for the capacity() function, to specify which capacity to read
typedef enum {
	REMAIN,     // Remaining Capacity (DEFAULT)
	FULL,       // Full Capacity
	AVAIL,      // Available Capacity
	AVAIL_FULL, // Full Available Capacity
	REMAIN_F,   // Remaining Capacity Filtered
	REMAIN_UF,  // Remaining Capacity Unfiltered
	FULL_F,     // Full Capacity Filtered
	FULL_UF,    // Full Capacity Unfiltered
	DESIGN      // Design Capacity
} capacity_measure;

// Parameters for the soc() function
typedef enum {
	FILTERED,  // State of Charge Filtered (DEFAULT)
	UNFILTERED // State of Charge Unfiltered
} soc_measure;

// Parameters for the soh() function
typedef enum {
	PERCENT,  // State of Health Percentage (DEFAULT)
	SOH_STAT  // State of Health Status Bits
} soh_measure;

// Parameters for the temperature() function
typedef enum {
	BATTERY,      // Battery Temperature (DEFAULT)
	INTERNAL_TEMP // Internal IC Temperature
} temp_measure;

// Parameters for the setGPOUTFunction() funciton
typedef enum {
	SOC_INT, // Set GPOUT to SOC_INT functionality
	BAT_LOW  // Set GPOUT to BAT_LOW functionality
} gpout_function;


	//////////////////////////////
	// Initialization Functions //
	//////////////////////////////
	
	/**
	    Initializes I2C and verifies communication with the BQ27441.
		Must be called before using any other functions.
		
		@return true if communication was successful.
	*/
	bool lipo_begin(void);
	
	/**
	    Configures the design capacity of the connected battery.
		
		@param capacity of battery (unsigned 16-bit value)
		@return true if capacity successfully set.
	*/
	bool lipo_setCapacity(uint16_t capacity);
	
	/////////////////////////////
	// Battery Characteristics //
	/////////////////////////////
	/**
	    Reads and returns the battery voltage
		
		@return battery voltage in mV
	*/
	uint16_t lipo_voltage(void);
	
	/**
	    Reads and returns the specified current measurement
		
		@param current_measure enum specifying current value to be read
		@return specified current measurement in mA. >0 indicates charging.
	*/
	extern current_measure current_measure_type;
	void set_lipo_current_arg(current_measure userInput);
	int16_t lipo_current(/*current_measure type = AVG*/);//========================================================CHANGE
	
	/**
	    Reads and returns the specified capacity measurement
		
		@param capacity_measure enum specifying capacity value to be read
		@return specified capacity measurement in mAh.
	*/
	extern capacity_measure capacity_measure_type;
	void set_lipo_capacity_arg(capacity_measure userInput);
	uint16_t lipo_capacity(/*capacity_measure type = REMAIN*/);//========================================================CHANGE
	
	/**
	    Reads and returns measured average power
		
		@return average power in mAh. >0 indicates charging.
	*/
	int16_t lipo_power(void);
	
	/**
	    Reads and returns specified state of charge measurement
		
		@param soc_measure enum specifying filtered or unfiltered measurement
		@return specified state of charge measurement in %
	*/
	extern soc_measure soc_measure_type;
	void set_lipo_soc_arg(soc_measure userInput);
	uint16_t lipo_soc(/*soc_measure type = FILTERED*/);//========================================================CHANGE
	
	
	/**
	    Reads and returns specified state of health measurement
		
		@param soh_measure enum specifying filtered or unfiltered measurement
		@return specified state of health measurement in %, or status bits
	*/
	extern soh_measure soh_measure_type;
	void set_lipo_soh_arg(soh_measure userInput);
	uint8_t lipo_soh(/*soh_measure type = PERCENT*/);//========================================================CHANGE
	
	/**
	    Reads and returns specified temperature measurement
		
		@param temp_measure enum specifying internal or battery measurement
		@return specified temperature measurement in degrees C
	*/
	extern temp_measure temp_measure_type;
	void set_lipo_temperature_arg(temp_measure userInput);
	uint16_t lipo_temperature(/*temp_measure type = BATTERY*/);//========================================================CHANGE
	
	////////////////////////////	
	// GPOUT Control Commands //
	////////////////////////////
	/**
	    Get GPOUT polarity setting (active-high or active-low)
		
		@return true if active-high, false if active-low
	*/
	bool lipo_GPOUTPolarity(void);
	
	/**
	    Set GPOUT polarity to active-high or active-low
		
		@param activeHigh is true if active-high, false if active-low
		@return true on success
	*/
	bool lipo_setGPOUTPolarity(bool activeHigh);
	
	/**
	    Get GPOUT function (BAT_LOW or SOC_INT)
		
		@return true if BAT_LOW or false if SOC_INT
	*/
	bool lipo_GPOUTFunction(void);
	
	/**
	    Set GPOUT function to BAT_LOW or SOC_INT
		
		@param function should be either BAT_LOW or SOC_INT
		@return true on success
	*/
	bool lipo_setGPOUTFunction(gpout_function function);
	
	/**
	    Get SOC1_Set Threshold - threshold to set the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t lipo_SOC1SetThreshold(void);
	
	/**
	    Get SOC1_Clear Threshold - threshold to clear the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t lipo_SOC1ClearThreshold(void);
	
	/**
	    Set the SOC1 set and clear thresholds to a percentage
		
		@param set and clear percentages between 0 and 100. clear > set.
		@return true on success
	*/
	bool lipo_setSOC1Thresholds(uint8_t set, uint8_t clear);
	
	/**
	    Get SOCF_Set Threshold - threshold to set the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t lipo_SOCFSetThreshold(void);
	
	/**
	    Get SOCF_Clear Threshold - threshold to clear the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t lipo_SOCFClearThreshold(void);
	
	/**
	    Set the SOCF set and clear thresholds to a percentage
		
		@param set and clear percentages between 0 and 100. clear > set.
		@return true on success
	*/
	bool lipo_setSOCFThresholds(uint8_t set, uint8_t clear);
	
	/**
	    Check if the SOC1 flag is set in flags()
		
		@return true if flag is set
	*/
	bool lipo_socFlag(void);
	
	/**
	    Check if the SOCF flag is set in flags()
		
		@return true if flag is set
	*/
	bool lipo_socfFlag(void);
	
	/**
	    Get the SOC_INT interval delta
		
		@return interval percentage value between 1 and 100
	*/
	uint8_t lipo_sociDelta(void);
	
	/**
	    Set the SOC_INT interval delta to a value between 1 and 100
		
		@param interval percentage value between 1 and 100
		@return true on success
	*/
	bool lipo_setSOCIDelta(uint8_t delta);
	
	/**
	    Pulse the GPOUT pin - must be in SOC_INT mode
		
		@return true on success
	*/
	bool lipo_pulseGPOUT(void);
	
	//////////////////////////
	// Control Sub-commands //
	//////////////////////////
	
	/**
	    Read the device type - should be 0x0421
		
		@return 16-bit value read from DEVICE_TYPE subcommand
	*/
	uint16_t lipo_deviceType(void);
	
	/**
	    Enter configuration mode - set userControl if calling from an Arduino
		sketch and you want control over when to exitConfig.
		
		@param userControl is true if the Arduino sketch is handling entering 
		and exiting config mode (should be false in library calls).
		@return true on success
	*/
	extern bool userControl;
	void set_lipo_enterConfig_arg(bool userInput);
	bool lipo_enterConfig(/*bool userControl = true*/);//========================================================CHANGE
	
	/**
	    Exit configuration mode with the option to perform a resimulation
		
		@param resim is true if resimulation should be performed after exiting
		@return true on success
	*/
	extern bool resim;
	void set_lipo_exitConfig(bool userInput);
	bool lipo_exitConfig(/*bool resim = true*/);//========================================================CHANGE
	
	/**
	    Read the flags() command
		
		@return 16-bit representation of flags() command register
	*/
	uint16_t lipo_flags(void);
	
	/**
	    Read the CONTROL_STATUS subcommand of control()
		
		@return 16-bit representation of CONTROL_STATUS subcommand
	*/
	uint16_t lipo_status(void);
	
#endif
