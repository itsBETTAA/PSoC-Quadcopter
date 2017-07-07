/*******************************************************************************
* File Name: GPOUT.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_GPOUT_H) /* Pins GPOUT_H */
#define CY_PINS_GPOUT_H

#include "cytypes.h"
#include "cyfitter.h"
#include "GPOUT_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} GPOUT_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   GPOUT_Read(void);
void    GPOUT_Write(uint8 value);
uint8   GPOUT_ReadDataReg(void);
#if defined(GPOUT__PC) || (CY_PSOC4_4200L) 
    void    GPOUT_SetDriveMode(uint8 mode);
#endif
void    GPOUT_SetInterruptMode(uint16 position, uint16 mode);
uint8   GPOUT_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void GPOUT_Sleep(void); 
void GPOUT_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(GPOUT__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define GPOUT_DRIVE_MODE_BITS        (3)
    #define GPOUT_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - GPOUT_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the GPOUT_SetDriveMode() function.
         *  @{
         */
        #define GPOUT_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define GPOUT_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define GPOUT_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define GPOUT_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define GPOUT_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define GPOUT_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define GPOUT_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define GPOUT_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define GPOUT_MASK               GPOUT__MASK
#define GPOUT_SHIFT              GPOUT__SHIFT
#define GPOUT_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in GPOUT_SetInterruptMode() function.
     *  @{
     */
        #define GPOUT_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define GPOUT_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define GPOUT_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define GPOUT_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(GPOUT__SIO)
    #define GPOUT_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(GPOUT__PC) && (CY_PSOC4_4200L)
    #define GPOUT_USBIO_ENABLE               ((uint32)0x80000000u)
    #define GPOUT_USBIO_DISABLE              ((uint32)(~GPOUT_USBIO_ENABLE))
    #define GPOUT_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define GPOUT_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define GPOUT_USBIO_ENTER_SLEEP          ((uint32)((1u << GPOUT_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << GPOUT_USBIO_SUSPEND_DEL_SHIFT)))
    #define GPOUT_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << GPOUT_USBIO_SUSPEND_SHIFT)))
    #define GPOUT_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << GPOUT_USBIO_SUSPEND_DEL_SHIFT)))
    #define GPOUT_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(GPOUT__PC)
    /* Port Configuration */
    #define GPOUT_PC                 (* (reg32 *) GPOUT__PC)
#endif
/* Pin State */
#define GPOUT_PS                     (* (reg32 *) GPOUT__PS)
/* Data Register */
#define GPOUT_DR                     (* (reg32 *) GPOUT__DR)
/* Input Buffer Disable Override */
#define GPOUT_INP_DIS                (* (reg32 *) GPOUT__PC2)

/* Interrupt configuration Registers */
#define GPOUT_INTCFG                 (* (reg32 *) GPOUT__INTCFG)
#define GPOUT_INTSTAT                (* (reg32 *) GPOUT__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define GPOUT_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(GPOUT__SIO)
    #define GPOUT_SIO_REG            (* (reg32 *) GPOUT__SIO)
#endif /* (GPOUT__SIO_CFG) */

/* USBIO registers */
#if !defined(GPOUT__PC) && (CY_PSOC4_4200L)
    #define GPOUT_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define GPOUT_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define GPOUT_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define GPOUT_DRIVE_MODE_SHIFT       (0x00u)
#define GPOUT_DRIVE_MODE_MASK        (0x07u << GPOUT_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins GPOUT_H */


/* [] END OF FILE */
