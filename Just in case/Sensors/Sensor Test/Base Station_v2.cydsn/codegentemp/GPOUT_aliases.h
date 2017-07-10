/*******************************************************************************
* File Name: GPOUT.h  
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

#if !defined(CY_PINS_GPOUT_ALIASES_H) /* Pins GPOUT_ALIASES_H */
#define CY_PINS_GPOUT_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define GPOUT_0			(GPOUT__0__PC)
#define GPOUT_0_PS		(GPOUT__0__PS)
#define GPOUT_0_PC		(GPOUT__0__PC)
#define GPOUT_0_DR		(GPOUT__0__DR)
#define GPOUT_0_SHIFT	(GPOUT__0__SHIFT)
#define GPOUT_0_INTR	((uint16)((uint16)0x0003u << (GPOUT__0__SHIFT*2u)))

#define GPOUT_INTR_ALL	 ((uint16)(GPOUT_0_INTR))


#endif /* End Pins GPOUT_ALIASES_H */


/* [] END OF FILE */
