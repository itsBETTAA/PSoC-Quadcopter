/***************************************************************************//**
* \file serial_BOOT.h
* \version 3.20
*
* \brief
*  This file provides constants and parameter values of the bootloader
*  communication APIs for the SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2014-2016, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_BOOT_serial_H)
#define CY_SCB_BOOT_serial_H

#include "serial_PVT.h"

#if (serial_SCB_MODE_I2C_INC)
    #include "serial_I2C.h"
#endif /* (serial_SCB_MODE_I2C_INC) */

#if (serial_SCB_MODE_EZI2C_INC)
    #include "serial_EZI2C.h"
#endif /* (serial_SCB_MODE_EZI2C_INC) */

#if (serial_SCB_MODE_SPI_INC || serial_SCB_MODE_UART_INC)
    #include "serial_SPI_UART.h"
#endif /* (serial_SCB_MODE_SPI_INC || serial_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define serial_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_serial) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (serial_SCB_MODE_I2C_INC)
    #define serial_I2C_BTLDR_COMM_ENABLED     (serial_BTLDR_COMM_ENABLED && \
                                                            (serial_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             serial_I2C_SLAVE_CONST))
#else
     #define serial_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (serial_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (serial_SCB_MODE_EZI2C_INC)
    #define serial_EZI2C_BTLDR_COMM_ENABLED   (serial_BTLDR_COMM_ENABLED && \
                                                         serial_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define serial_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (serial_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (serial_SCB_MODE_SPI_INC)
    #define serial_SPI_BTLDR_COMM_ENABLED     (serial_BTLDR_COMM_ENABLED && \
                                                            (serial_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             serial_SPI_SLAVE_CONST))
#else
        #define serial_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (serial_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (serial_SCB_MODE_UART_INC)
       #define serial_UART_BTLDR_COMM_ENABLED    (serial_BTLDR_COMM_ENABLED && \
                                                            (serial_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (serial_UART_RX_DIRECTION && \
                                                              serial_UART_TX_DIRECTION)))
#else
     #define serial_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (serial_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define serial_BTLDR_COMM_MODE_ENABLED    (serial_I2C_BTLDR_COMM_ENABLED   || \
                                                     serial_SPI_BTLDR_COMM_ENABLED   || \
                                                     serial_EZI2C_BTLDR_COMM_ENABLED || \
                                                     serial_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void serial_I2CCyBtldrCommStart(void);
    void serial_I2CCyBtldrCommStop (void);
    void serial_I2CCyBtldrCommReset(void);
    cystatus serial_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus serial_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (serial_SCB_MODE_I2C_CONST_CFG)
        #define serial_CyBtldrCommStart   serial_I2CCyBtldrCommStart
        #define serial_CyBtldrCommStop    serial_I2CCyBtldrCommStop
        #define serial_CyBtldrCommReset   serial_I2CCyBtldrCommReset
        #define serial_CyBtldrCommRead    serial_I2CCyBtldrCommRead
        #define serial_CyBtldrCommWrite   serial_I2CCyBtldrCommWrite
    #endif /* (serial_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void serial_EzI2CCyBtldrCommStart(void);
    void serial_EzI2CCyBtldrCommStop (void);
    void serial_EzI2CCyBtldrCommReset(void);
    cystatus serial_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus serial_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (serial_SCB_MODE_EZI2C_CONST_CFG)
        #define serial_CyBtldrCommStart   serial_EzI2CCyBtldrCommStart
        #define serial_CyBtldrCommStop    serial_EzI2CCyBtldrCommStop
        #define serial_CyBtldrCommReset   serial_EzI2CCyBtldrCommReset
        #define serial_CyBtldrCommRead    serial_EzI2CCyBtldrCommRead
        #define serial_CyBtldrCommWrite   serial_EzI2CCyBtldrCommWrite
    #endif /* (serial_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void serial_SpiCyBtldrCommStart(void);
    void serial_SpiCyBtldrCommStop (void);
    void serial_SpiCyBtldrCommReset(void);
    cystatus serial_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus serial_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (serial_SCB_MODE_SPI_CONST_CFG)
        #define serial_CyBtldrCommStart   serial_SpiCyBtldrCommStart
        #define serial_CyBtldrCommStop    serial_SpiCyBtldrCommStop
        #define serial_CyBtldrCommReset   serial_SpiCyBtldrCommReset
        #define serial_CyBtldrCommRead    serial_SpiCyBtldrCommRead
        #define serial_CyBtldrCommWrite   serial_SpiCyBtldrCommWrite
    #endif /* (serial_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void serial_UartCyBtldrCommStart(void);
    void serial_UartCyBtldrCommStop (void);
    void serial_UartCyBtldrCommReset(void);
    cystatus serial_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus serial_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (serial_SCB_MODE_UART_CONST_CFG)
        #define serial_CyBtldrCommStart   serial_UartCyBtldrCommStart
        #define serial_CyBtldrCommStop    serial_UartCyBtldrCommStop
        #define serial_CyBtldrCommReset   serial_UartCyBtldrCommReset
        #define serial_CyBtldrCommRead    serial_UartCyBtldrCommRead
        #define serial_CyBtldrCommWrite   serial_UartCyBtldrCommWrite
    #endif /* (serial_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_UART_BTLDR_COMM_ENABLED) */

/**
* \addtogroup group_bootloader
* @{
*/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_BTLDR_COMM_ENABLED)
    #if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void serial_CyBtldrCommStart(void);
        void serial_CyBtldrCommStop (void);
        void serial_CyBtldrCommReset(void);
        cystatus serial_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus serial_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_serial)
        #define CyBtldrCommStart    serial_CyBtldrCommStart
        #define CyBtldrCommStop     serial_CyBtldrCommStop
        #define CyBtldrCommReset    serial_CyBtldrCommReset
        #define CyBtldrCommWrite    serial_CyBtldrCommWrite
        #define CyBtldrCommRead     serial_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_serial) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (serial_BTLDR_COMM_ENABLED) */

/** @} group_bootloader */

/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define serial_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define serial_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define serial_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define serial_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef serial_SPI_BYTE_TO_BYTE
    #define serial_SPI_BYTE_TO_BYTE   (160u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef serial_UART_BYTE_TO_BYTE
    #define serial_UART_BYTE_TO_BYTE  (8330u)
#endif /* serial_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_serial_H) */


/* [] END OF FILE */
