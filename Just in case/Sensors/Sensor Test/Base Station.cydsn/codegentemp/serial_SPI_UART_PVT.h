/***************************************************************************//**
* \file serial_SPI_UART_PVT.h
* \version 3.20
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component in SPI and UART modes.
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

#if !defined(CY_SCB_SPI_UART_PVT_serial_H)
#define CY_SCB_SPI_UART_PVT_serial_H

#include "serial_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if (serial_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  serial_rxBufferHead;
    extern volatile uint32  serial_rxBufferTail;
    
    /**
    * \addtogroup group_globals
    * @{
    */
    
    /** Sets when internal software receive buffer overflow
     *  was occurred.
    */  
    extern volatile uint8   serial_rxBufferOverflow;
    /** @} globals */
#endif /* (serial_INTERNAL_RX_SW_BUFFER_CONST) */

#if (serial_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  serial_txBufferHead;
    extern volatile uint32  serial_txBufferTail;
#endif /* (serial_INTERNAL_TX_SW_BUFFER_CONST) */

#if (serial_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 serial_rxBufferInternal[serial_INTERNAL_RX_BUFFER_SIZE];
#endif /* (serial_INTERNAL_RX_SW_BUFFER) */

#if (serial_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 serial_txBufferInternal[serial_TX_BUFFER_SIZE];
#endif /* (serial_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

void serial_SpiPostEnable(void);
void serial_SpiStop(void);

#if (serial_SCB_MODE_SPI_CONST_CFG)
    void serial_SpiInit(void);
#endif /* (serial_SCB_MODE_SPI_CONST_CFG) */

#if (serial_SPI_WAKE_ENABLE_CONST)
    void serial_SpiSaveConfig(void);
    void serial_SpiRestoreConfig(void);
#endif /* (serial_SPI_WAKE_ENABLE_CONST) */

void serial_UartPostEnable(void);
void serial_UartStop(void);

#if (serial_SCB_MODE_UART_CONST_CFG)
    void serial_UartInit(void);
#endif /* (serial_SCB_MODE_UART_CONST_CFG) */

#if (serial_UART_WAKE_ENABLE_CONST)
    void serial_UartSaveConfig(void);
    void serial_UartRestoreConfig(void);
#endif /* (serial_UART_WAKE_ENABLE_CONST) */


/***************************************
*         UART API Constants
***************************************/

/* UART RX and TX position to be used in serial_SetPins() */
#define serial_UART_RX_PIN_ENABLE    (serial_UART_RX)
#define serial_UART_TX_PIN_ENABLE    (serial_UART_TX)

/* UART RTS and CTS position to be used in  serial_SetPins() */
#define serial_UART_RTS_PIN_ENABLE    (0x10u)
#define serial_UART_CTS_PIN_ENABLE    (0x20u)


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Interrupt processing */
#define serial_SpiUartEnableIntRx(intSourceMask)  serial_SetRxInterruptMode(intSourceMask)
#define serial_SpiUartEnableIntTx(intSourceMask)  serial_SetTxInterruptMode(intSourceMask)
uint32  serial_SpiUartDisableIntRx(void);
uint32  serial_SpiUartDisableIntTx(void);


#endif /* (CY_SCB_SPI_UART_PVT_serial_H) */


/* [] END OF FILE */
