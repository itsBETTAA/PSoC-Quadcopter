/*******************************************************************************
* File Name: serial_tx.h  
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

#if !defined(CY_PINS_serial_tx_ALIASES_H) /* Pins serial_tx_ALIASES_H */
#define CY_PINS_serial_tx_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define serial_tx_0			(serial_tx__0__PC)
#define serial_tx_0_PS		(serial_tx__0__PS)
#define serial_tx_0_PC		(serial_tx__0__PC)
#define serial_tx_0_DR		(serial_tx__0__DR)
#define serial_tx_0_SHIFT	(serial_tx__0__SHIFT)
#define serial_tx_0_INTR	((uint16)((uint16)0x0003u << (serial_tx__0__SHIFT*2u)))

#define serial_tx_INTR_ALL	 ((uint16)(serial_tx_0_INTR))


#endif /* End Pins serial_tx_ALIASES_H */


/* [] END OF FILE */
