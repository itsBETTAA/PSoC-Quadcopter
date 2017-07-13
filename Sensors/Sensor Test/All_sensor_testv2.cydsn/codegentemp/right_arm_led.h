/*******************************************************************************
* File Name: right_arm_led.h  
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

#if !defined(CY_PINS_right_arm_led_H) /* Pins right_arm_led_H */
#define CY_PINS_right_arm_led_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "right_arm_led_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 right_arm_led__PORT == 15 && ((right_arm_led__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    right_arm_led_Write(uint8 value);
void    right_arm_led_SetDriveMode(uint8 mode);
uint8   right_arm_led_ReadDataReg(void);
uint8   right_arm_led_Read(void);
void    right_arm_led_SetInterruptMode(uint16 position, uint16 mode);
uint8   right_arm_led_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the right_arm_led_SetDriveMode() function.
     *  @{
     */
        #define right_arm_led_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define right_arm_led_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define right_arm_led_DM_RES_UP          PIN_DM_RES_UP
        #define right_arm_led_DM_RES_DWN         PIN_DM_RES_DWN
        #define right_arm_led_DM_OD_LO           PIN_DM_OD_LO
        #define right_arm_led_DM_OD_HI           PIN_DM_OD_HI
        #define right_arm_led_DM_STRONG          PIN_DM_STRONG
        #define right_arm_led_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define right_arm_led_MASK               right_arm_led__MASK
#define right_arm_led_SHIFT              right_arm_led__SHIFT
#define right_arm_led_WIDTH              1u

/* Interrupt constants */
#if defined(right_arm_led__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in right_arm_led_SetInterruptMode() function.
     *  @{
     */
        #define right_arm_led_INTR_NONE      (uint16)(0x0000u)
        #define right_arm_led_INTR_RISING    (uint16)(0x0001u)
        #define right_arm_led_INTR_FALLING   (uint16)(0x0002u)
        #define right_arm_led_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define right_arm_led_INTR_MASK      (0x01u) 
#endif /* (right_arm_led__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define right_arm_led_PS                     (* (reg8 *) right_arm_led__PS)
/* Data Register */
#define right_arm_led_DR                     (* (reg8 *) right_arm_led__DR)
/* Port Number */
#define right_arm_led_PRT_NUM                (* (reg8 *) right_arm_led__PRT) 
/* Connect to Analog Globals */                                                  
#define right_arm_led_AG                     (* (reg8 *) right_arm_led__AG)                       
/* Analog MUX bux enable */
#define right_arm_led_AMUX                   (* (reg8 *) right_arm_led__AMUX) 
/* Bidirectional Enable */                                                        
#define right_arm_led_BIE                    (* (reg8 *) right_arm_led__BIE)
/* Bit-mask for Aliased Register Access */
#define right_arm_led_BIT_MASK               (* (reg8 *) right_arm_led__BIT_MASK)
/* Bypass Enable */
#define right_arm_led_BYP                    (* (reg8 *) right_arm_led__BYP)
/* Port wide control signals */                                                   
#define right_arm_led_CTL                    (* (reg8 *) right_arm_led__CTL)
/* Drive Modes */
#define right_arm_led_DM0                    (* (reg8 *) right_arm_led__DM0) 
#define right_arm_led_DM1                    (* (reg8 *) right_arm_led__DM1)
#define right_arm_led_DM2                    (* (reg8 *) right_arm_led__DM2) 
/* Input Buffer Disable Override */
#define right_arm_led_INP_DIS                (* (reg8 *) right_arm_led__INP_DIS)
/* LCD Common or Segment Drive */
#define right_arm_led_LCD_COM_SEG            (* (reg8 *) right_arm_led__LCD_COM_SEG)
/* Enable Segment LCD */
#define right_arm_led_LCD_EN                 (* (reg8 *) right_arm_led__LCD_EN)
/* Slew Rate Control */
#define right_arm_led_SLW                    (* (reg8 *) right_arm_led__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define right_arm_led_PRTDSI__CAPS_SEL       (* (reg8 *) right_arm_led__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define right_arm_led_PRTDSI__DBL_SYNC_IN    (* (reg8 *) right_arm_led__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define right_arm_led_PRTDSI__OE_SEL0        (* (reg8 *) right_arm_led__PRTDSI__OE_SEL0) 
#define right_arm_led_PRTDSI__OE_SEL1        (* (reg8 *) right_arm_led__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define right_arm_led_PRTDSI__OUT_SEL0       (* (reg8 *) right_arm_led__PRTDSI__OUT_SEL0) 
#define right_arm_led_PRTDSI__OUT_SEL1       (* (reg8 *) right_arm_led__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define right_arm_led_PRTDSI__SYNC_OUT       (* (reg8 *) right_arm_led__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(right_arm_led__SIO_CFG)
    #define right_arm_led_SIO_HYST_EN        (* (reg8 *) right_arm_led__SIO_HYST_EN)
    #define right_arm_led_SIO_REG_HIFREQ     (* (reg8 *) right_arm_led__SIO_REG_HIFREQ)
    #define right_arm_led_SIO_CFG            (* (reg8 *) right_arm_led__SIO_CFG)
    #define right_arm_led_SIO_DIFF           (* (reg8 *) right_arm_led__SIO_DIFF)
#endif /* (right_arm_led__SIO_CFG) */

/* Interrupt Registers */
#if defined(right_arm_led__INTSTAT)
    #define right_arm_led_INTSTAT            (* (reg8 *) right_arm_led__INTSTAT)
    #define right_arm_led_SNAP               (* (reg8 *) right_arm_led__SNAP)
    
	#define right_arm_led_0_INTTYPE_REG 		(* (reg8 *) right_arm_led__0__INTTYPE)
#endif /* (right_arm_led__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_right_arm_led_H */


/* [] END OF FILE */
