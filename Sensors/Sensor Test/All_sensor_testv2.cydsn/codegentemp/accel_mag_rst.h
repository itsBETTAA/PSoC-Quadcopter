/*******************************************************************************
* File Name: accel_mag_rst.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_accel_mag_rst_H) /* Pins accel_mag_rst_H */
#define CY_PINS_accel_mag_rst_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "accel_mag_rst_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 accel_mag_rst__PORT == 15 && ((accel_mag_rst__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    accel_mag_rst_Write(uint8 value);
void    accel_mag_rst_SetDriveMode(uint8 mode);
uint8   accel_mag_rst_ReadDataReg(void);
uint8   accel_mag_rst_Read(void);
void    accel_mag_rst_SetInterruptMode(uint16 position, uint16 mode);
uint8   accel_mag_rst_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the accel_mag_rst_SetDriveMode() function.
     *  @{
     */
        #define accel_mag_rst_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define accel_mag_rst_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define accel_mag_rst_DM_RES_UP          PIN_DM_RES_UP
        #define accel_mag_rst_DM_RES_DWN         PIN_DM_RES_DWN
        #define accel_mag_rst_DM_OD_LO           PIN_DM_OD_LO
        #define accel_mag_rst_DM_OD_HI           PIN_DM_OD_HI
        #define accel_mag_rst_DM_STRONG          PIN_DM_STRONG
        #define accel_mag_rst_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define accel_mag_rst_MASK               accel_mag_rst__MASK
#define accel_mag_rst_SHIFT              accel_mag_rst__SHIFT
#define accel_mag_rst_WIDTH              1u

/* Interrupt constants */
#if defined(accel_mag_rst__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in accel_mag_rst_SetInterruptMode() function.
     *  @{
     */
        #define accel_mag_rst_INTR_NONE      (uint16)(0x0000u)
        #define accel_mag_rst_INTR_RISING    (uint16)(0x0001u)
        #define accel_mag_rst_INTR_FALLING   (uint16)(0x0002u)
        #define accel_mag_rst_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define accel_mag_rst_INTR_MASK      (0x01u) 
#endif /* (accel_mag_rst__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define accel_mag_rst_PS                     (* (reg8 *) accel_mag_rst__PS)
/* Data Register */
#define accel_mag_rst_DR                     (* (reg8 *) accel_mag_rst__DR)
/* Port Number */
#define accel_mag_rst_PRT_NUM                (* (reg8 *) accel_mag_rst__PRT) 
/* Connect to Analog Globals */                                                  
#define accel_mag_rst_AG                     (* (reg8 *) accel_mag_rst__AG)                       
/* Analog MUX bux enable */
#define accel_mag_rst_AMUX                   (* (reg8 *) accel_mag_rst__AMUX) 
/* Bidirectional Enable */                                                        
#define accel_mag_rst_BIE                    (* (reg8 *) accel_mag_rst__BIE)
/* Bit-mask for Aliased Register Access */
#define accel_mag_rst_BIT_MASK               (* (reg8 *) accel_mag_rst__BIT_MASK)
/* Bypass Enable */
#define accel_mag_rst_BYP                    (* (reg8 *) accel_mag_rst__BYP)
/* Port wide control signals */                                                   
#define accel_mag_rst_CTL                    (* (reg8 *) accel_mag_rst__CTL)
/* Drive Modes */
#define accel_mag_rst_DM0                    (* (reg8 *) accel_mag_rst__DM0) 
#define accel_mag_rst_DM1                    (* (reg8 *) accel_mag_rst__DM1)
#define accel_mag_rst_DM2                    (* (reg8 *) accel_mag_rst__DM2) 
/* Input Buffer Disable Override */
#define accel_mag_rst_INP_DIS                (* (reg8 *) accel_mag_rst__INP_DIS)
/* LCD Common or Segment Drive */
#define accel_mag_rst_LCD_COM_SEG            (* (reg8 *) accel_mag_rst__LCD_COM_SEG)
/* Enable Segment LCD */
#define accel_mag_rst_LCD_EN                 (* (reg8 *) accel_mag_rst__LCD_EN)
/* Slew Rate Control */
#define accel_mag_rst_SLW                    (* (reg8 *) accel_mag_rst__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define accel_mag_rst_PRTDSI__CAPS_SEL       (* (reg8 *) accel_mag_rst__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define accel_mag_rst_PRTDSI__DBL_SYNC_IN    (* (reg8 *) accel_mag_rst__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define accel_mag_rst_PRTDSI__OE_SEL0        (* (reg8 *) accel_mag_rst__PRTDSI__OE_SEL0) 
#define accel_mag_rst_PRTDSI__OE_SEL1        (* (reg8 *) accel_mag_rst__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define accel_mag_rst_PRTDSI__OUT_SEL0       (* (reg8 *) accel_mag_rst__PRTDSI__OUT_SEL0) 
#define accel_mag_rst_PRTDSI__OUT_SEL1       (* (reg8 *) accel_mag_rst__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define accel_mag_rst_PRTDSI__SYNC_OUT       (* (reg8 *) accel_mag_rst__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(accel_mag_rst__SIO_CFG)
    #define accel_mag_rst_SIO_HYST_EN        (* (reg8 *) accel_mag_rst__SIO_HYST_EN)
    #define accel_mag_rst_SIO_REG_HIFREQ     (* (reg8 *) accel_mag_rst__SIO_REG_HIFREQ)
    #define accel_mag_rst_SIO_CFG            (* (reg8 *) accel_mag_rst__SIO_CFG)
    #define accel_mag_rst_SIO_DIFF           (* (reg8 *) accel_mag_rst__SIO_DIFF)
#endif /* (accel_mag_rst__SIO_CFG) */

/* Interrupt Registers */
#if defined(accel_mag_rst__INTSTAT)
    #define accel_mag_rst_INTSTAT            (* (reg8 *) accel_mag_rst__INTSTAT)
    #define accel_mag_rst_SNAP               (* (reg8 *) accel_mag_rst__SNAP)
    
	#define accel_mag_rst_0_INTTYPE_REG 		(* (reg8 *) accel_mag_rst__0__INTTYPE)
#endif /* (accel_mag_rst__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_accel_mag_rst_H */


/* [] END OF FILE */
