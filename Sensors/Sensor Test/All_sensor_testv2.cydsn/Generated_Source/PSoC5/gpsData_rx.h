/*******************************************************************************
* File Name: gpsData_rx.h
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
#if !defined(CY_ISR_gpsData_rx_H)
#define CY_ISR_gpsData_rx_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void gpsData_rx_Start(void);
void gpsData_rx_StartEx(cyisraddress address);
void gpsData_rx_Stop(void);

CY_ISR_PROTO(gpsData_rx_Interrupt);

void gpsData_rx_SetVector(cyisraddress address);
cyisraddress gpsData_rx_GetVector(void);

void gpsData_rx_SetPriority(uint8 priority);
uint8 gpsData_rx_GetPriority(void);

void gpsData_rx_Enable(void);
uint8 gpsData_rx_GetState(void);
void gpsData_rx_Disable(void);

void gpsData_rx_SetPending(void);
void gpsData_rx_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the gpsData_rx ISR. */
#define gpsData_rx_INTC_VECTOR            ((reg32 *) gpsData_rx__INTC_VECT)

/* Address of the gpsData_rx ISR priority. */
#define gpsData_rx_INTC_PRIOR             ((reg8 *) gpsData_rx__INTC_PRIOR_REG)

/* Priority of the gpsData_rx interrupt. */
#define gpsData_rx_INTC_PRIOR_NUMBER      gpsData_rx__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable gpsData_rx interrupt. */
#define gpsData_rx_INTC_SET_EN            ((reg32 *) gpsData_rx__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the gpsData_rx interrupt. */
#define gpsData_rx_INTC_CLR_EN            ((reg32 *) gpsData_rx__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the gpsData_rx interrupt state to pending. */
#define gpsData_rx_INTC_SET_PD            ((reg32 *) gpsData_rx__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the gpsData_rx interrupt. */
#define gpsData_rx_INTC_CLR_PD            ((reg32 *) gpsData_rx__INTC_CLR_PD_REG)


#endif /* CY_ISR_gpsData_rx_H */


/* [] END OF FILE */
