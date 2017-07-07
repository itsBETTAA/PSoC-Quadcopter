/***************************************************************************//**
* \file .h
* \version 3.20
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_serial_H)
#define CY_SCB_PVT_serial_H

#include "serial.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define serial_SetI2CExtClkInterruptMode(interruptMask) serial_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define serial_ClearI2CExtClkInterruptSource(interruptMask) serial_CLEAR_INTR_I2C_EC(interruptMask)
#define serial_GetI2CExtClkInterruptSource()                (serial_INTR_I2C_EC_REG)
#define serial_GetI2CExtClkInterruptMode()                  (serial_INTR_I2C_EC_MASK_REG)
#define serial_GetI2CExtClkInterruptSourceMasked()          (serial_INTR_I2C_EC_MASKED_REG)

#if (!serial_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define serial_SetSpiExtClkInterruptMode(interruptMask) \
                                                                serial_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define serial_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                serial_CLEAR_INTR_SPI_EC(interruptMask)
    #define serial_GetExtSpiClkInterruptSource()                 (serial_INTR_SPI_EC_REG)
    #define serial_GetExtSpiClkInterruptMode()                   (serial_INTR_SPI_EC_MASK_REG)
    #define serial_GetExtSpiClkInterruptSourceMasked()           (serial_INTR_SPI_EC_MASKED_REG)
#endif /* (!serial_CY_SCBIP_V1) */

#if(serial_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void serial_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (serial_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_serial_CUSTOM_INTR_HANDLER)
    extern cyisraddress serial_customIntrHandler;
#endif /* !defined (CY_REMOVE_serial_CUSTOM_INTR_HANDLER) */
#endif /* (serial_SCB_IRQ_INTERNAL) */

extern serial_BACKUP_STRUCT serial_backup;

#if(serial_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 serial_scbMode;
    extern uint8 serial_scbEnableWake;
    extern uint8 serial_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 serial_mode;
    extern uint8 serial_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * serial_rxBuffer;
    extern uint8   serial_rxDataBits;
    extern uint32  serial_rxBufferSize;

    extern volatile uint8 * serial_txBuffer;
    extern uint8   serial_txDataBits;
    extern uint32  serial_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 serial_numberOfAddr;
    extern uint8 serial_subAddrSize;
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (! (serial_SCB_MODE_I2C_CONST_CFG || \
        serial_SCB_MODE_EZI2C_CONST_CFG))
    extern uint16 serial_IntrTxMask;
#endif /* (! (serial_SCB_MODE_I2C_CONST_CFG || \
              serial_SCB_MODE_EZI2C_CONST_CFG)) */


/***************************************
*        Conditional Macro
****************************************/

#if(serial_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define serial_SCB_MODE_I2C_RUNTM_CFG     (serial_SCB_MODE_I2C      == serial_scbMode)
    #define serial_SCB_MODE_SPI_RUNTM_CFG     (serial_SCB_MODE_SPI      == serial_scbMode)
    #define serial_SCB_MODE_UART_RUNTM_CFG    (serial_SCB_MODE_UART     == serial_scbMode)
    #define serial_SCB_MODE_EZI2C_RUNTM_CFG   (serial_SCB_MODE_EZI2C    == serial_scbMode)
    #define serial_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (serial_SCB_MODE_UNCONFIG == serial_scbMode)

    /* Defines wakeup enable */
    #define serial_SCB_WAKE_ENABLE_CHECK       (0u != serial_scbEnableWake)
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!serial_CY_SCBIP_V1)
    #define serial_SCB_PINS_NUMBER    (7u)
#else
    #define serial_SCB_PINS_NUMBER    (2u)
#endif /* (!serial_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_serial_H) */


/* [] END OF FILE */
