/*******************************************************************************
* File Name: cisr.h
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
#if !defined(CY_ISR_cisr_H)
#define CY_ISR_cisr_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void cisr_Start(void);
void cisr_StartEx(cyisraddress address);
void cisr_Stop(void);

CY_ISR_PROTO(cisr_Interrupt);

void cisr_SetVector(cyisraddress address);
cyisraddress cisr_GetVector(void);

void cisr_SetPriority(uint8 priority);
uint8 cisr_GetPriority(void);

void cisr_Enable(void);
uint8 cisr_GetState(void);
void cisr_Disable(void);

void cisr_SetPending(void);
void cisr_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the cisr ISR. */
#define cisr_INTC_VECTOR            ((reg32 *) cisr__INTC_VECT)

/* Address of the cisr ISR priority. */
#define cisr_INTC_PRIOR             ((reg32 *) cisr__INTC_PRIOR_REG)

/* Priority of the cisr interrupt. */
#define cisr_INTC_PRIOR_NUMBER      cisr__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable cisr interrupt. */
#define cisr_INTC_SET_EN            ((reg32 *) cisr__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the cisr interrupt. */
#define cisr_INTC_CLR_EN            ((reg32 *) cisr__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the cisr interrupt state to pending. */
#define cisr_INTC_SET_PD            ((reg32 *) cisr__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the cisr interrupt. */
#define cisr_INTC_CLR_PD            ((reg32 *) cisr__INTC_CLR_PD_REG)



#endif /* CY_ISR_cisr_H */


/* [] END OF FILE */
