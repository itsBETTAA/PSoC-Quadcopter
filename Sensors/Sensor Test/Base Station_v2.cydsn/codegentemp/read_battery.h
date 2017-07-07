/*******************************************************************************
* File Name: read_battery.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_read_battery_H)
#define CY_ISR_read_battery_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void read_battery_Start(void);
void read_battery_StartEx(cyisraddress address);
void read_battery_Stop(void);

CY_ISR_PROTO(read_battery_Interrupt);

void read_battery_SetVector(cyisraddress address);
cyisraddress read_battery_GetVector(void);

void read_battery_SetPriority(uint8 priority);
uint8 read_battery_GetPriority(void);

void read_battery_Enable(void);
uint8 read_battery_GetState(void);
void read_battery_Disable(void);

void read_battery_SetPending(void);
void read_battery_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the read_battery ISR. */
#define read_battery_INTC_VECTOR            ((reg32 *) read_battery__INTC_VECT)

/* Address of the read_battery ISR priority. */
#define read_battery_INTC_PRIOR             ((reg32 *) read_battery__INTC_PRIOR_REG)

/* Priority of the read_battery interrupt. */
#define read_battery_INTC_PRIOR_NUMBER      read_battery__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable read_battery interrupt. */
#define read_battery_INTC_SET_EN            ((reg32 *) read_battery__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the read_battery interrupt. */
#define read_battery_INTC_CLR_EN            ((reg32 *) read_battery__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the read_battery interrupt state to pending. */
#define read_battery_INTC_SET_PD            ((reg32 *) read_battery__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the read_battery interrupt. */
#define read_battery_INTC_CLR_PD            ((reg32 *) read_battery__INTC_CLR_PD_REG)



#endif /* CY_ISR_read_battery_H */


/* [] END OF FILE */
