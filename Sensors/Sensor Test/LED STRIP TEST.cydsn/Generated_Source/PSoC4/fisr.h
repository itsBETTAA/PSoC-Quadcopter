/*******************************************************************************
* File Name: fisr.h
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
#if !defined(CY_ISR_fisr_H)
#define CY_ISR_fisr_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void fisr_Start(void);
void fisr_StartEx(cyisraddress address);
void fisr_Stop(void);

CY_ISR_PROTO(fisr_Interrupt);

void fisr_SetVector(cyisraddress address);
cyisraddress fisr_GetVector(void);

void fisr_SetPriority(uint8 priority);
uint8 fisr_GetPriority(void);

void fisr_Enable(void);
uint8 fisr_GetState(void);
void fisr_Disable(void);

void fisr_SetPending(void);
void fisr_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the fisr ISR. */
#define fisr_INTC_VECTOR            ((reg32 *) fisr__INTC_VECT)

/* Address of the fisr ISR priority. */
#define fisr_INTC_PRIOR             ((reg32 *) fisr__INTC_PRIOR_REG)

/* Priority of the fisr interrupt. */
#define fisr_INTC_PRIOR_NUMBER      fisr__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable fisr interrupt. */
#define fisr_INTC_SET_EN            ((reg32 *) fisr__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the fisr interrupt. */
#define fisr_INTC_CLR_EN            ((reg32 *) fisr__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the fisr interrupt state to pending. */
#define fisr_INTC_SET_PD            ((reg32 *) fisr__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the fisr interrupt. */
#define fisr_INTC_CLR_PD            ((reg32 *) fisr__INTC_CLR_PD_REG)



#endif /* CY_ISR_fisr_H */


/* [] END OF FILE */
