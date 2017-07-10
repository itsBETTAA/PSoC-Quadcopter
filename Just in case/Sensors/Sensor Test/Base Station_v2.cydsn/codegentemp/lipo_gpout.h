/*******************************************************************************
* File Name: lipo_gpout.h
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
#if !defined(CY_ISR_lipo_gpout_H)
#define CY_ISR_lipo_gpout_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void lipo_gpout_Start(void);
void lipo_gpout_StartEx(cyisraddress address);
void lipo_gpout_Stop(void);

CY_ISR_PROTO(lipo_gpout_Interrupt);

void lipo_gpout_SetVector(cyisraddress address);
cyisraddress lipo_gpout_GetVector(void);

void lipo_gpout_SetPriority(uint8 priority);
uint8 lipo_gpout_GetPriority(void);

void lipo_gpout_Enable(void);
uint8 lipo_gpout_GetState(void);
void lipo_gpout_Disable(void);

void lipo_gpout_SetPending(void);
void lipo_gpout_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the lipo_gpout ISR. */
#define lipo_gpout_INTC_VECTOR            ((reg32 *) lipo_gpout__INTC_VECT)

/* Address of the lipo_gpout ISR priority. */
#define lipo_gpout_INTC_PRIOR             ((reg32 *) lipo_gpout__INTC_PRIOR_REG)

/* Priority of the lipo_gpout interrupt. */
#define lipo_gpout_INTC_PRIOR_NUMBER      lipo_gpout__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable lipo_gpout interrupt. */
#define lipo_gpout_INTC_SET_EN            ((reg32 *) lipo_gpout__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the lipo_gpout interrupt. */
#define lipo_gpout_INTC_CLR_EN            ((reg32 *) lipo_gpout__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the lipo_gpout interrupt state to pending. */
#define lipo_gpout_INTC_SET_PD            ((reg32 *) lipo_gpout__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the lipo_gpout interrupt. */
#define lipo_gpout_INTC_CLR_PD            ((reg32 *) lipo_gpout__INTC_CLR_PD_REG)



#endif /* CY_ISR_lipo_gpout_H */


/* [] END OF FILE */
