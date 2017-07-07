/*******************************************************************************
* File Name: gpsSerial_IntClock.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_gpsSerial_IntClock_H)
#define CY_CLOCK_gpsSerial_IntClock_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
* Conditional Compilation Parameters
***************************************/

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component cy_clock_v2_20 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*        Function Prototypes
***************************************/

void gpsSerial_IntClock_Start(void) ;
void gpsSerial_IntClock_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void gpsSerial_IntClock_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void gpsSerial_IntClock_StandbyPower(uint8 state) ;
void gpsSerial_IntClock_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 gpsSerial_IntClock_GetDividerRegister(void) ;
void gpsSerial_IntClock_SetModeRegister(uint8 modeBitMask) ;
void gpsSerial_IntClock_ClearModeRegister(uint8 modeBitMask) ;
uint8 gpsSerial_IntClock_GetModeRegister(void) ;
void gpsSerial_IntClock_SetSourceRegister(uint8 clkSource) ;
uint8 gpsSerial_IntClock_GetSourceRegister(void) ;
#if defined(gpsSerial_IntClock__CFG3)
void gpsSerial_IntClock_SetPhaseRegister(uint8 clkPhase) ;
uint8 gpsSerial_IntClock_GetPhaseRegister(void) ;
#endif /* defined(gpsSerial_IntClock__CFG3) */

#define gpsSerial_IntClock_Enable()                       gpsSerial_IntClock_Start()
#define gpsSerial_IntClock_Disable()                      gpsSerial_IntClock_Stop()
#define gpsSerial_IntClock_SetDivider(clkDivider)         gpsSerial_IntClock_SetDividerRegister(clkDivider, 1u)
#define gpsSerial_IntClock_SetDividerValue(clkDivider)    gpsSerial_IntClock_SetDividerRegister((clkDivider) - 1u, 1u)
#define gpsSerial_IntClock_SetMode(clkMode)               gpsSerial_IntClock_SetModeRegister(clkMode)
#define gpsSerial_IntClock_SetSource(clkSource)           gpsSerial_IntClock_SetSourceRegister(clkSource)
#if defined(gpsSerial_IntClock__CFG3)
#define gpsSerial_IntClock_SetPhase(clkPhase)             gpsSerial_IntClock_SetPhaseRegister(clkPhase)
#define gpsSerial_IntClock_SetPhaseValue(clkPhase)        gpsSerial_IntClock_SetPhaseRegister((clkPhase) + 1u)
#endif /* defined(gpsSerial_IntClock__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define gpsSerial_IntClock_CLKEN              (* (reg8 *) gpsSerial_IntClock__PM_ACT_CFG)
#define gpsSerial_IntClock_CLKEN_PTR          ((reg8 *) gpsSerial_IntClock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define gpsSerial_IntClock_CLKSTBY            (* (reg8 *) gpsSerial_IntClock__PM_STBY_CFG)
#define gpsSerial_IntClock_CLKSTBY_PTR        ((reg8 *) gpsSerial_IntClock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define gpsSerial_IntClock_DIV_LSB            (* (reg8 *) gpsSerial_IntClock__CFG0)
#define gpsSerial_IntClock_DIV_LSB_PTR        ((reg8 *) gpsSerial_IntClock__CFG0)
#define gpsSerial_IntClock_DIV_PTR            ((reg16 *) gpsSerial_IntClock__CFG0)

/* Clock MSB divider configuration register. */
#define gpsSerial_IntClock_DIV_MSB            (* (reg8 *) gpsSerial_IntClock__CFG1)
#define gpsSerial_IntClock_DIV_MSB_PTR        ((reg8 *) gpsSerial_IntClock__CFG1)

/* Mode and source configuration register */
#define gpsSerial_IntClock_MOD_SRC            (* (reg8 *) gpsSerial_IntClock__CFG2)
#define gpsSerial_IntClock_MOD_SRC_PTR        ((reg8 *) gpsSerial_IntClock__CFG2)

#if defined(gpsSerial_IntClock__CFG3)
/* Analog clock phase configuration register */
#define gpsSerial_IntClock_PHASE              (* (reg8 *) gpsSerial_IntClock__CFG3)
#define gpsSerial_IntClock_PHASE_PTR          ((reg8 *) gpsSerial_IntClock__CFG3)
#endif /* defined(gpsSerial_IntClock__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define gpsSerial_IntClock_CLKEN_MASK         gpsSerial_IntClock__PM_ACT_MSK
#define gpsSerial_IntClock_CLKSTBY_MASK       gpsSerial_IntClock__PM_STBY_MSK

/* CFG2 field masks */
#define gpsSerial_IntClock_SRC_SEL_MSK        gpsSerial_IntClock__CFG2_SRC_SEL_MASK
#define gpsSerial_IntClock_MODE_MASK          (~(gpsSerial_IntClock_SRC_SEL_MSK))

#if defined(gpsSerial_IntClock__CFG3)
/* CFG3 phase mask */
#define gpsSerial_IntClock_PHASE_MASK         gpsSerial_IntClock__CFG3_PHASE_DLY_MASK
#endif /* defined(gpsSerial_IntClock__CFG3) */

#endif /* CY_CLOCK_gpsSerial_IntClock_H */


/* [] END OF FILE */
