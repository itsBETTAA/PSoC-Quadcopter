/*******************************************************************************
* File Name: butt_pin.h  
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

#if !defined(CY_PINS_butt_pin_ALIASES_H) /* Pins butt_pin_ALIASES_H */
#define CY_PINS_butt_pin_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define butt_pin_0			(butt_pin__0__PC)
#define butt_pin_0_PS		(butt_pin__0__PS)
#define butt_pin_0_PC		(butt_pin__0__PC)
#define butt_pin_0_DR		(butt_pin__0__DR)
#define butt_pin_0_SHIFT	(butt_pin__0__SHIFT)
#define butt_pin_0_INTR	((uint16)((uint16)0x0003u << (butt_pin__0__SHIFT*2u)))

#define butt_pin_INTR_ALL	 ((uint16)(butt_pin_0_INTR))


#endif /* End Pins butt_pin_ALIASES_H */


/* [] END OF FILE */
