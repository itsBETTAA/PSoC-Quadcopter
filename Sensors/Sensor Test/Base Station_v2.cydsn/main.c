/* ========================================
 *
 * Copyright ARROW ELECTRONICS, 2017
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

/* ========================================
 *
 * This code will read the neccessary information from Battery Babysitter
 * And display the necessary information on a 3 LEDs: Green, Yellow, Red
 *
 * ========================================
*/
#include "project.h"
#include <batteryMonitor.h>
#include <sev_seg.h>
#include <mySerial.h>

//if in debug mode,
//we will never enter hibernate mode
//all that we will be doing is outputing the battery monitor
//values through the UART line
#define _DEBUG 0

//if in setup_GPOUT mode,
//the battery monitor will be configured to output
//an active low or active high signal when the
//battery monitor detects that the battery level are low or dangerously low
#define setup_GPOUT 1

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 2000; // e.g. 2000mAh battery
const uint8_t SOCI_SET = 15; // Interrupt set threshold at 20%
const uint8_t SOCI_CLR = 20; // Interrupt clear threshold at 25%
const uint8_t SOCF_SET = 5;  // Final threshold set at 5%
const uint8_t SOCF_CLR = 10; // Final threshold clear at 10%

#if _DEBUG
void getBatteryStats();
#endif

//=============================================================//
//=============Interrupt handler Prototype=====================//
//=============================================================//

CY_ISR(read_battery_Handler);
CY_ISR(battery_critical_low_Handler);

//=============================================================//
//========Battery Monitor Setup Function Prototype=============//
//=============================================================//

void setupBQ27441(void);

//=============================================================//
//============LED Toggler Functions Prototype==================//
//=============================================================//

void toggleBoardLED   (int numberOfTimes, int delayBetweenBlinks);
void toggleRedLED     (int numberOfTimes, int delayBetweenBlinks);
void toggleYellowLED  (int numberOfTimes, int delayBetweenBlinks);
void toggleGreenLED   (int numberOfTimes, int delayBetweenBlinks);
void toggleSequenceLED(int numberOfTimes, int delayBetweenSequenceToggle);

//=============================================================//
//==============Interrupt Handler function=====================//
//=============================================================//

CY_ISR(read_battery_Handler){
    
    BOARD_LED_Write(~BOARD_LED_Read());    //TOGGLE THE BOARD LED TO SHOW that we are in an interrupt
    unsigned int percentage = lipo_soc();  // Read state-of-charge (%)
   
    if(percentage >= 25){  //if the state of charge of the battery is greater than 40 percent
        RED_LED_Write(0);  //Keep the green lights on
        YELLOW_LED_Write(0); 
        GREEN_LED_Write(1);
    }
    else if(percentage >= 15){ //else if the state of charge of the battery is greater than 15 percent
        RED_LED_Write(0);      //Keep the Yellow light on
        YELLOW_LED_Write(1);
        GREEN_LED_Write(0);
    }
    else{
        RED_LED_Write(1);    //else, which means that State of Charge is less than 15 percent
        YELLOW_LED_Write(0); //Keep the red light on to notify the user to charge the base station soon
        GREEN_LED_Write(0);
    }
    CyDelay(10000);
    
    RED_LED_Write(0);   //inialize all of the LEDs turned off
    YELLOW_LED_Write(0); 
    GREEN_LED_Write(0);
    
    butt_pin_ClearInterrupt(); //Clear the interrupt so that we are not going to be stuck in the interrupt
} 

CY_ISR(battery_critical_low_Handler){
    // Check which of the flags triggered the interrupt
    if (lipo_socfFlag()) {
        #if _DEBUG
            serial_println("<!-- WARNING: Battery Dangerously low -->");
        #else
            RED_LED_Write(1);    
            YELLOW_LED_Write(0); 
            GREEN_LED_Write(0);
            read_battery_Disable();
        #endif
    }
    else if (lipo_socFlag()){
        #if _DEBUG
            serial_println("<!-- WARNING: Battery Low -->");
        #else
            RED_LED_Write(0);    
            YELLOW_LED_Write(1); 
            GREEN_LED_Write(0);
        #endif
    }
    
    
    GPOUT_ClearInterrupt();
}

//=============================================================//
//=======================Main Function=========================//
//=============================================================//

int main(void)
{
    CyGlobalIntEnable;                          /* Enable global interrupts. */    
    setupBQ27441();                             //Initializes battery monitor
    read_battery_StartEx(read_battery_Handler); //Sets up the interrupt Handler
    lipo_gpout_StartEx(battery_critical_low_Handler);
    
    RED_LED_Write(0);   //inialize all of the LEDs turned off
    YELLOW_LED_Write(0); 
    GREEN_LED_Write(0);
    
    #if _DEBUG         
        serial_Start(); //If the user wishes to debug, then initialize serial communication
    #else
        //if there was a reset, check if it was caused by a hibernate wakeup
        if(CySysPmGetResetReason() == CY_PM_RESET_REASON_WAKEUP_HIB){
            CySysPmUnfreezeIo(); //if so, unfreeze the GPIO PINS
        }
    #endif
    
    for(;;)
    {
        #if _DEBUG  //if the _DEBUG MACRO value is 1
            getBatteryStats(); //then the user is debuging so allow data to be sent through the UART port
        #else
            //if debug is off then allow for hibernate sleep to occur
            CySysPmFreezeIo(); //allows us to retain their state upon wake up
            //Pulls 150 nA
            CySysPmHibernate(); //Puts the device in Hibernate state
                                //This turns off all of the clocks but retains logical state
                                //used for long period of inactivity 
                                //where the device is not needed for for critical tasks and periodic
                                //wake ups are not required
                                //Any exist resets the microcontroller
        #endif
    }
}

//=============================================================//
//========Battery Monitor Initialization Function==============//
//=============================================================//

void setupBQ27441(void)
{
    #if _DEBUG  
        // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
        // connected and communicating.
        serial_println(" Lipo monitor communication being initialized.");
        if (!lipo_begin()) // begin() will return true if communication is successful
        {
	        // If communication fails, print an error message and loop forever.
            serial_println(" Error: Unable to communicate with BQ27441.");
            serial_println("  Check wiring and try again.");
            serial_println("  (Battery must be plugged into Battery Babysitter!)");
            while (1) ;
        }
        serial_println(" Connected to BQ27441!");
  
        // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
        // of your battery.
        serial_println(" Setting Lipo monitor capacity.");
        lipo_setCapacity(BATTERY_CAPACITY);
        serial_println(" Lipo monitor capacity set.");
    #else
        if(!lipo_begin()){  //if the battery monitor does not properly initialize
           while(1){        //Toggle the RED led on indefinetly
                toggleRedLED(3, 500);
                CyDelay(2000);
            }
        }
        toggleYellowLED(3, 1000); //else if all connections goes well, toggle the yellow LED 3 times with a second interval
        CyDelay(1000);            //wait for about 2 seconds to settle down the user after the Yellow LED Blink
        
        lipo_setCapacity(BATTERY_CAPACITY); //set the capacity of the Lipo battery to the specified value
        toggleGreenLED(3,1000);             //if the lipo capacity was set correctly and didn't get stuck in the function
                                            //toggle the green LED 3 times with a seond interval to notify the user that
                                            //Battery Monitor is responsive and fully setup
    #endif
    
    #ifdef setup_GPOUT
        // In this example, we'll manually enter and exit config mode. By controlling
        // config mode manually, you can set the chip up faster -- completing all of
        // the set up in a single config mode sweep.
        lipo_enterConfig(); // To configure the values below, you must be in config mode
        lipo_setCapacity(BATTERY_CAPACITY); // Set the battery capacity
        lipo_setGPOUTPolarity(0); // Set GPOUT to active-high
        lipo_setGPOUTFunction(BAT_LOW); // Set GPOUT to BAT_LOW mode
        lipo_setSOC1Thresholds(SOCI_SET, SOCI_CLR); // Set SOCI set and clear thresholds
        lipo_setSOCFThresholds(SOCF_SET, SOCF_CLR); // Set SOCF set and clear thresholds
        lipo_exitConfig();
    #endif
}

//=============================================================//
//==============Battery Monitor Debug Function=================//
//=============================================================//
//=============Only ran if _DEBUG MACRO is a 1=================//

#if _DEBUG  //if debug is true then define the function if not it would not compile
    void getBatteryStats()
    {

        // Read battery stats from the BQ27441-G1A
        unsigned int soc = lipo_soc();  // Read state-of-charge (%)
        unsigned int volts = lipo_voltage(); // Read battery voltage (mV)

        set_lipo_current_arg(AVG);
        int current = lipo_current(/*AVG*/); // Read average current (mA)

        set_lipo_capacity_arg(FULL_F);
        unsigned int fullCapacity = lipo_capacity(/*FULL*/); // Read full capacity (mAh)

        set_lipo_capacity_arg(REMAIN);
        unsigned int capacity = lipo_capacity(/*REMAIN*/); // Read remaining capacity (mAh)

        int power = lipo_power(); // Read average power draw (mW)
        int health = lipo_soh(); // Read state-of-health (%)


        serial_print("SOC(%) = "); 
        serial_printInt(soc);
        serial_print(" | Volt(mV) = "); 
        serial_printInt(volts);
        serial_print(" | Current(mA) = "); 
        serial_printInt(current);

        serial_print(" | Remaining(mAh) = "); 
        serial_printInt(capacity);
        serial_print(" / full(mAh) = "); 
        serial_printInt(fullCapacity);
  
        serial_print(" | Power(mW) = "); 
        serial_printInt(power);
        serial_print(" | Health = "); 
        serial_printIntln(health);
    }
#endif

//=============================================================//
//=======Function to help with board signallings===============//
//=============================================================//

void toggleBoardLED(int numberOfTimes, int delayBetweenBlinks){
    int i = 0;
    for(i = 0; i<= numberOfTimes; i++){
        BOARD_LED_Write(~BOARD_LED_Read());
        CyDelay(500);
        BOARD_LED_Write(~BOARD_LED_Read());
        CyDelay(delayBetweenBlinks);
    }
}

void toggleRedLED(int numberOfTimes, int delayBetweenBlinks){
    int i = 0;
    RED_LED_Write(0);
    
    for(i = 0; i<= numberOfTimes; i++){
        RED_LED_Write(~RED_LED_Read());
        CyDelay(500);
        RED_LED_Write(~RED_LED_Read());
        CyDelay(delayBetweenBlinks);
    }
}
void toggleYellowLED(int numberOfTimes, int delayBetweenBlinks){
    int i = 0;
    YELLOW_LED_Write(0);
    
    for(i = 0; i<= numberOfTimes; i++){
        YELLOW_LED_Write(~YELLOW_LED_Read());
        CyDelay(500);
        YELLOW_LED_Write(~YELLOW_LED_Read());
        CyDelay(delayBetweenBlinks);
    }
}
void toggleGreenLED(int numberOfTimes, int delayBetweenBlinks){
    int i = 0;
    GREEN_LED_Write(0);
    
    for(i = 0; i<= numberOfTimes; i++){
        GREEN_LED_Write(~GREEN_LED_Read());
        CyDelay(500);
        GREEN_LED_Write(~GREEN_LED_Read());
        CyDelay(delayBetweenBlinks);
    }
}

void toggleSequenceLED(int numberOfTimes, int delayBetweenSequenceToggle){
    int i = 0;
    RED_LED_Write(0);   //turn all of the LEDS off at first
    YELLOW_LED_Write(0);
    GREEN_LED_Write(0);
    
    for(i = 0; i < (numberOfTimes*2); i++){      //Then, for the number of specified times
        RED_LED_Write(~RED_LED_Read());      //toggle each LEDS in a sequence 
        CyDelay(delayBetweenSequenceToggle); //with the given intervals
        YELLOW_LED_Write(~YELLOW_LED_Read());
        CyDelay(delayBetweenSequenceToggle);
        GREEN_LED_Write(~GREEN_LED_Read());
        CyDelay(delayBetweenSequenceToggle);
    }
}

/* [] END OF FILE */
