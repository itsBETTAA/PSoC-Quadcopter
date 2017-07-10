/*******************************************************************************
* File Name: left_arm_led.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_left_arm_led_ALIASES_H) /* Pins left_arm_led_ALIASES_H */
#define CY_PINS_left_arm_led_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
*              Constants        
***************************************/
#define left_arm_led_0			(left_arm_led__0__PC)
#define left_arm_led_0_INTR	((uint16)((uint16)0x0001u << left_arm_led__0__SHIFT))

#define left_arm_led_INTR_ALL	 ((uint16)(left_arm_led_0_INTR))

#endif /* End Pins left_arm_led_ALIASES_H */


/* [] END OF FILE */
