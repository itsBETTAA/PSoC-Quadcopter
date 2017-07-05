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
 * And display the necessary information on a 4- 7 segment+dot display
 *
 * ========================================
*/
#include "project.h"
#include <batteryMonitor.h>
#include <sev_seg.h>
#include <mySerial.h>

#define _DEBUG 0

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 2000; // e.g. 2000mAh battery

void setupBQ27441(void);
void getBatteryStats();
CY_ISR(read_battery_Handler);

int i = 0;

CY_ISR(read_battery_Handler){
    
    CyDelay(50);
    LED_Write(~LED_Read());
    butt_pin_ClearInterrupt();
}    

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */    
    read_battery_StartEx(read_battery_Handler);
    //setupBQ27441();
    serial_Start();
    
    for(;;)
    {
        
    }
}

void setupBQ27441(void)
{
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo_begin()) // begin() will return true if communication is successful
  {
	// If communication fails, print an error message and loop forever.
    serial_println("Error: Unable to communicate with BQ27441.");
    serial_println("  Check wiring and try again.");
    serial_println("  (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  serial_println("Connected to BQ27441!");
  
  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  lipo_setCapacity(BATTERY_CAPACITY);
}

void getBatteryStats()
{
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = lipo_soc();  // Read state-of-charge (%)
  unsigned int volts = lipo_voltage(); // Read battery voltage (mV)

  set_lipo_current_arg(AVG);
  int current = lipo_current(/*AVG*/); // Read average current (mA)

  set_lipo_capacity_arg(FULL);
  unsigned int fullCapacity = lipo_capacity(/*FULL*/); // Read full capacity (mAh)

  set_lipo_capacity_arg(REMAIN);
  unsigned int capacity = lipo_capacity(/*REMAIN*/); // Read remaining capacity (mAh)

  int power = lipo_power(); // Read average power draw (mW)
  int health = lipo_soh(); // Read state-of-health (%)

#if _DEBUG
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
#endif
}

/* [] END OF FILE */
