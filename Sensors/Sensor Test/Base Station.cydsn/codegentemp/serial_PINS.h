/***************************************************************************//**
* \file serial_PINS.h
* \version 3.20
*
* \brief
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PINS_serial_H)
#define CY_SCB_PINS_serial_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define serial_REMOVE_RX_WAKE_SCL_MOSI_PIN  (1u)
#define serial_REMOVE_RX_SCL_MOSI_PIN      (1u)
#define serial_REMOVE_TX_SDA_MISO_PIN      (1u)
#define serial_REMOVE_SCLK_PIN      (1u)
#define serial_REMOVE_SS0_PIN      (1u)
#define serial_REMOVE_SS1_PIN                 (1u)
#define serial_REMOVE_SS2_PIN                 (1u)
#define serial_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define serial_REMOVE_I2C_PINS                (1u)
#define serial_REMOVE_SPI_MASTER_PINS         (1u)
#define serial_REMOVE_SPI_MASTER_SCLK_PIN     (1u)
#define serial_REMOVE_SPI_MASTER_MOSI_PIN     (1u)
#define serial_REMOVE_SPI_MASTER_MISO_PIN     (1u)
#define serial_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define serial_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define serial_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define serial_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define serial_REMOVE_SPI_SLAVE_PINS          (1u)
#define serial_REMOVE_SPI_SLAVE_MOSI_PIN      (1u)
#define serial_REMOVE_SPI_SLAVE_MISO_PIN      (1u)
#define serial_REMOVE_UART_TX_PIN             (0u)
#define serial_REMOVE_UART_RX_TX_PIN          (1u)
#define serial_REMOVE_UART_RX_PIN             (1u)
#define serial_REMOVE_UART_RX_WAKE_PIN        (1u)
#define serial_REMOVE_UART_RTS_PIN            (1u)
#define serial_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define serial_RX_WAKE_SCL_MOSI_PIN (0u == serial_REMOVE_RX_WAKE_SCL_MOSI_PIN)
#define serial_RX_SCL_MOSI_PIN     (0u == serial_REMOVE_RX_SCL_MOSI_PIN)
#define serial_TX_SDA_MISO_PIN     (0u == serial_REMOVE_TX_SDA_MISO_PIN)
#define serial_SCLK_PIN     (0u == serial_REMOVE_SCLK_PIN)
#define serial_SS0_PIN     (0u == serial_REMOVE_SS0_PIN)
#define serial_SS1_PIN                (0u == serial_REMOVE_SS1_PIN)
#define serial_SS2_PIN                (0u == serial_REMOVE_SS2_PIN)
#define serial_SS3_PIN                (0u == serial_REMOVE_SS3_PIN)

/* Mode defined pins */
#define serial_I2C_PINS               (0u == serial_REMOVE_I2C_PINS)
#define serial_SPI_MASTER_PINS        (0u == serial_REMOVE_SPI_MASTER_PINS)
#define serial_SPI_MASTER_SCLK_PIN    (0u == serial_REMOVE_SPI_MASTER_SCLK_PIN)
#define serial_SPI_MASTER_MOSI_PIN    (0u == serial_REMOVE_SPI_MASTER_MOSI_PIN)
#define serial_SPI_MASTER_MISO_PIN    (0u == serial_REMOVE_SPI_MASTER_MISO_PIN)
#define serial_SPI_MASTER_SS0_PIN     (0u == serial_REMOVE_SPI_MASTER_SS0_PIN)
#define serial_SPI_MASTER_SS1_PIN     (0u == serial_REMOVE_SPI_MASTER_SS1_PIN)
#define serial_SPI_MASTER_SS2_PIN     (0u == serial_REMOVE_SPI_MASTER_SS2_PIN)
#define serial_SPI_MASTER_SS3_PIN     (0u == serial_REMOVE_SPI_MASTER_SS3_PIN)
#define serial_SPI_SLAVE_PINS         (0u == serial_REMOVE_SPI_SLAVE_PINS)
#define serial_SPI_SLAVE_MOSI_PIN     (0u == serial_REMOVE_SPI_SLAVE_MOSI_PIN)
#define serial_SPI_SLAVE_MISO_PIN     (0u == serial_REMOVE_SPI_SLAVE_MISO_PIN)
#define serial_UART_TX_PIN            (0u == serial_REMOVE_UART_TX_PIN)
#define serial_UART_RX_TX_PIN         (0u == serial_REMOVE_UART_RX_TX_PIN)
#define serial_UART_RX_PIN            (0u == serial_REMOVE_UART_RX_PIN)
#define serial_UART_RX_WAKE_PIN       (0u == serial_REMOVE_UART_RX_WAKE_PIN)
#define serial_UART_RTS_PIN           (0u == serial_REMOVE_UART_RTS_PIN)
#define serial_UART_CTS_PIN           (0u == serial_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if (serial_RX_WAKE_SCL_MOSI_PIN)
    #include "serial_uart_rx_wake_i2c_scl_spi_mosi.h"
#endif /* (serial_RX_SCL_MOSI) */

#if (serial_RX_SCL_MOSI_PIN)
    #include "serial_uart_rx_i2c_scl_spi_mosi.h"
#endif /* (serial_RX_SCL_MOSI) */

#if (serial_TX_SDA_MISO_PIN)
    #include "serial_uart_tx_i2c_sda_spi_miso.h"
#endif /* (serial_TX_SDA_MISO) */

#if (serial_SCLK_PIN)
    #include "serial_spi_sclk.h"
#endif /* (serial_SCLK) */

#if (serial_SS0_PIN)
    #include "serial_spi_ss0.h"
#endif /* (serial_SS0_PIN) */

#if (serial_SS1_PIN)
    #include "serial_spi_ss1.h"
#endif /* (serial_SS1_PIN) */

#if (serial_SS2_PIN)
    #include "serial_spi_ss2.h"
#endif /* (serial_SS2_PIN) */

#if (serial_SS3_PIN)
    #include "serial_spi_ss3.h"
#endif /* (serial_SS3_PIN) */

#if (serial_I2C_PINS)
    #include "serial_scl.h"
    #include "serial_sda.h"
#endif /* (serial_I2C_PINS) */

#if (serial_SPI_MASTER_PINS)
#if (serial_SPI_MASTER_SCLK_PIN)
    #include "serial_sclk_m.h"
#endif /* (serial_SPI_MASTER_SCLK_PIN) */

#if (serial_SPI_MASTER_MOSI_PIN)
    #include "serial_mosi_m.h"
#endif /* (serial_SPI_MASTER_MOSI_PIN) */

#if (serial_SPI_MASTER_MISO_PIN)
    #include "serial_miso_m.h"
#endif /*(serial_SPI_MASTER_MISO_PIN) */
#endif /* (serial_SPI_MASTER_PINS) */

#if (serial_SPI_SLAVE_PINS)
    #include "serial_sclk_s.h"
    #include "serial_ss_s.h"

#if (serial_SPI_SLAVE_MOSI_PIN)
    #include "serial_mosi_s.h"
#endif /* (serial_SPI_SLAVE_MOSI_PIN) */

#if (serial_SPI_SLAVE_MISO_PIN)
    #include "serial_miso_s.h"
#endif /*(serial_SPI_SLAVE_MISO_PIN) */
#endif /* (serial_SPI_SLAVE_PINS) */

#if (serial_SPI_MASTER_SS0_PIN)
    #include "serial_ss0_m.h"
#endif /* (serial_SPI_MASTER_SS0_PIN) */

#if (serial_SPI_MASTER_SS1_PIN)
    #include "serial_ss1_m.h"
#endif /* (serial_SPI_MASTER_SS1_PIN) */

#if (serial_SPI_MASTER_SS2_PIN)
    #include "serial_ss2_m.h"
#endif /* (serial_SPI_MASTER_SS2_PIN) */

#if (serial_SPI_MASTER_SS3_PIN)
    #include "serial_ss3_m.h"
#endif /* (serial_SPI_MASTER_SS3_PIN) */

#if (serial_UART_TX_PIN)
    #include "serial_tx.h"
#endif /* (serial_UART_TX_PIN) */

#if (serial_UART_RX_TX_PIN)
    #include "serial_rx_tx.h"
#endif /* (serial_UART_RX_TX_PIN) */

#if (serial_UART_RX_PIN)
    #include "serial_rx.h"
#endif /* (serial_UART_RX_PIN) */

#if (serial_UART_RX_WAKE_PIN)
    #include "serial_rx_wake.h"
#endif /* (serial_UART_RX_WAKE_PIN) */

#if (serial_UART_RTS_PIN)
    #include "serial_rts.h"
#endif /* (serial_UART_RTS_PIN) */

#if (serial_UART_CTS_PIN)
    #include "serial_cts.h"
#endif /* (serial_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if (serial_RX_SCL_MOSI_PIN)
    #define serial_RX_SCL_MOSI_HSIOM_REG   (*(reg32 *) serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define serial_RX_SCL_MOSI_HSIOM_PTR   ( (reg32 *) serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    
    #define serial_RX_SCL_MOSI_HSIOM_MASK      (serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define serial_RX_SCL_MOSI_HSIOM_POS       (serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define serial_RX_SCL_MOSI_HSIOM_SEL_GPIO  (serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define serial_RX_SCL_MOSI_HSIOM_SEL_I2C   (serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define serial_RX_SCL_MOSI_HSIOM_SEL_SPI   (serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define serial_RX_SCL_MOSI_HSIOM_SEL_UART  (serial_uart_rx_i2c_scl_spi_mosi__0__HSIOM_UART)
    
#elif (serial_RX_WAKE_SCL_MOSI_PIN)
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_REG   (*(reg32 *) serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_PTR   ( (reg32 *) serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_MASK      (serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_POS       (serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_SEL_GPIO  (serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C   (serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI   (serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define serial_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART  (serial_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_UART)    
   
    #define serial_RX_WAKE_SCL_MOSI_INTCFG_REG (*(reg32 *) serial_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define serial_RX_WAKE_SCL_MOSI_INTCFG_PTR ( (reg32 *) serial_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define serial_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS  (serial_uart_rx_wake_i2c_scl_spi_mosi__SHIFT)
    #define serial_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK ((uint32) serial_INTCFG_TYPE_MASK << \
                                                                           serial_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS)
#else
    /* None of pins serial_RX_SCL_MOSI_PIN or serial_RX_WAKE_SCL_MOSI_PIN present.*/
#endif /* (serial_RX_SCL_MOSI_PIN) */

#if (serial_TX_SDA_MISO_PIN)
    #define serial_TX_SDA_MISO_HSIOM_REG   (*(reg32 *) serial_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define serial_TX_SDA_MISO_HSIOM_PTR   ( (reg32 *) serial_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    
    #define serial_TX_SDA_MISO_HSIOM_MASK      (serial_uart_tx_i2c_sda_spi_miso__0__HSIOM_MASK)
    #define serial_TX_SDA_MISO_HSIOM_POS       (serial_uart_tx_i2c_sda_spi_miso__0__HSIOM_SHIFT)
    #define serial_TX_SDA_MISO_HSIOM_SEL_GPIO  (serial_uart_tx_i2c_sda_spi_miso__0__HSIOM_GPIO)
    #define serial_TX_SDA_MISO_HSIOM_SEL_I2C   (serial_uart_tx_i2c_sda_spi_miso__0__HSIOM_I2C)
    #define serial_TX_SDA_MISO_HSIOM_SEL_SPI   (serial_uart_tx_i2c_sda_spi_miso__0__HSIOM_SPI)
    #define serial_TX_SDA_MISO_HSIOM_SEL_UART  (serial_uart_tx_i2c_sda_spi_miso__0__HSIOM_UART)
#endif /* (serial_TX_SDA_MISO_PIN) */

#if (serial_SCLK_PIN)
    #define serial_SCLK_HSIOM_REG   (*(reg32 *) serial_spi_sclk__0__HSIOM)
    #define serial_SCLK_HSIOM_PTR   ( (reg32 *) serial_spi_sclk__0__HSIOM)
    
    #define serial_SCLK_HSIOM_MASK      (serial_spi_sclk__0__HSIOM_MASK)
    #define serial_SCLK_HSIOM_POS       (serial_spi_sclk__0__HSIOM_SHIFT)
    #define serial_SCLK_HSIOM_SEL_GPIO  (serial_spi_sclk__0__HSIOM_GPIO)
    #define serial_SCLK_HSIOM_SEL_I2C   (serial_spi_sclk__0__HSIOM_I2C)
    #define serial_SCLK_HSIOM_SEL_SPI   (serial_spi_sclk__0__HSIOM_SPI)
    #define serial_SCLK_HSIOM_SEL_UART  (serial_spi_sclk__0__HSIOM_UART)
#endif /* (serial_SCLK_PIN) */

#if (serial_SS0_PIN)
    #define serial_SS0_HSIOM_REG   (*(reg32 *) serial_spi_ss0__0__HSIOM)
    #define serial_SS0_HSIOM_PTR   ( (reg32 *) serial_spi_ss0__0__HSIOM)
    
    #define serial_SS0_HSIOM_MASK      (serial_spi_ss0__0__HSIOM_MASK)
    #define serial_SS0_HSIOM_POS       (serial_spi_ss0__0__HSIOM_SHIFT)
    #define serial_SS0_HSIOM_SEL_GPIO  (serial_spi_ss0__0__HSIOM_GPIO)
    #define serial_SS0_HSIOM_SEL_I2C   (serial_spi_ss0__0__HSIOM_I2C)
    #define serial_SS0_HSIOM_SEL_SPI   (serial_spi_ss0__0__HSIOM_SPI)
#if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
    #define serial_SS0_HSIOM_SEL_UART  (serial_spi_ss0__0__HSIOM_UART)
#endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */
#endif /* (serial_SS0_PIN) */

#if (serial_SS1_PIN)
    #define serial_SS1_HSIOM_REG  (*(reg32 *) serial_spi_ss1__0__HSIOM)
    #define serial_SS1_HSIOM_PTR  ( (reg32 *) serial_spi_ss1__0__HSIOM)
    
    #define serial_SS1_HSIOM_MASK     (serial_spi_ss1__0__HSIOM_MASK)
    #define serial_SS1_HSIOM_POS      (serial_spi_ss1__0__HSIOM_SHIFT)
    #define serial_SS1_HSIOM_SEL_GPIO (serial_spi_ss1__0__HSIOM_GPIO)
    #define serial_SS1_HSIOM_SEL_I2C  (serial_spi_ss1__0__HSIOM_I2C)
    #define serial_SS1_HSIOM_SEL_SPI  (serial_spi_ss1__0__HSIOM_SPI)
#endif /* (serial_SS1_PIN) */

#if (serial_SS2_PIN)
    #define serial_SS2_HSIOM_REG     (*(reg32 *) serial_spi_ss2__0__HSIOM)
    #define serial_SS2_HSIOM_PTR     ( (reg32 *) serial_spi_ss2__0__HSIOM)
    
    #define serial_SS2_HSIOM_MASK     (serial_spi_ss2__0__HSIOM_MASK)
    #define serial_SS2_HSIOM_POS      (serial_spi_ss2__0__HSIOM_SHIFT)
    #define serial_SS2_HSIOM_SEL_GPIO (serial_spi_ss2__0__HSIOM_GPIO)
    #define serial_SS2_HSIOM_SEL_I2C  (serial_spi_ss2__0__HSIOM_I2C)
    #define serial_SS2_HSIOM_SEL_SPI  (serial_spi_ss2__0__HSIOM_SPI)
#endif /* (serial_SS2_PIN) */

#if (serial_SS3_PIN)
    #define serial_SS3_HSIOM_REG     (*(reg32 *) serial_spi_ss3__0__HSIOM)
    #define serial_SS3_HSIOM_PTR     ( (reg32 *) serial_spi_ss3__0__HSIOM)
    
    #define serial_SS3_HSIOM_MASK     (serial_spi_ss3__0__HSIOM_MASK)
    #define serial_SS3_HSIOM_POS      (serial_spi_ss3__0__HSIOM_SHIFT)
    #define serial_SS3_HSIOM_SEL_GPIO (serial_spi_ss3__0__HSIOM_GPIO)
    #define serial_SS3_HSIOM_SEL_I2C  (serial_spi_ss3__0__HSIOM_I2C)
    #define serial_SS3_HSIOM_SEL_SPI  (serial_spi_ss3__0__HSIOM_SPI)
#endif /* (serial_SS3_PIN) */

#if (serial_I2C_PINS)
    #define serial_SCL_HSIOM_REG  (*(reg32 *) serial_scl__0__HSIOM)
    #define serial_SCL_HSIOM_PTR  ( (reg32 *) serial_scl__0__HSIOM)
    
    #define serial_SCL_HSIOM_MASK     (serial_scl__0__HSIOM_MASK)
    #define serial_SCL_HSIOM_POS      (serial_scl__0__HSIOM_SHIFT)
    #define serial_SCL_HSIOM_SEL_GPIO (serial_sda__0__HSIOM_GPIO)
    #define serial_SCL_HSIOM_SEL_I2C  (serial_sda__0__HSIOM_I2C)
    
    #define serial_SDA_HSIOM_REG  (*(reg32 *) serial_sda__0__HSIOM)
    #define serial_SDA_HSIOM_PTR  ( (reg32 *) serial_sda__0__HSIOM)
    
    #define serial_SDA_HSIOM_MASK     (serial_sda__0__HSIOM_MASK)
    #define serial_SDA_HSIOM_POS      (serial_sda__0__HSIOM_SHIFT)
    #define serial_SDA_HSIOM_SEL_GPIO (serial_sda__0__HSIOM_GPIO)
    #define serial_SDA_HSIOM_SEL_I2C  (serial_sda__0__HSIOM_I2C)
#endif /* (serial_I2C_PINS) */

#if (serial_SPI_SLAVE_PINS)
    #define serial_SCLK_S_HSIOM_REG   (*(reg32 *) serial_sclk_s__0__HSIOM)
    #define serial_SCLK_S_HSIOM_PTR   ( (reg32 *) serial_sclk_s__0__HSIOM)
    
    #define serial_SCLK_S_HSIOM_MASK      (serial_sclk_s__0__HSIOM_MASK)
    #define serial_SCLK_S_HSIOM_POS       (serial_sclk_s__0__HSIOM_SHIFT)
    #define serial_SCLK_S_HSIOM_SEL_GPIO  (serial_sclk_s__0__HSIOM_GPIO)
    #define serial_SCLK_S_HSIOM_SEL_SPI   (serial_sclk_s__0__HSIOM_SPI)
    
    #define serial_SS0_S_HSIOM_REG    (*(reg32 *) serial_ss0_s__0__HSIOM)
    #define serial_SS0_S_HSIOM_PTR    ( (reg32 *) serial_ss0_s__0__HSIOM)
    
    #define serial_SS0_S_HSIOM_MASK       (serial_ss0_s__0__HSIOM_MASK)
    #define serial_SS0_S_HSIOM_POS        (serial_ss0_s__0__HSIOM_SHIFT)
    #define serial_SS0_S_HSIOM_SEL_GPIO   (serial_ss0_s__0__HSIOM_GPIO)  
    #define serial_SS0_S_HSIOM_SEL_SPI    (serial_ss0_s__0__HSIOM_SPI)
#endif /* (serial_SPI_SLAVE_PINS) */

#if (serial_SPI_SLAVE_MOSI_PIN)
    #define serial_MOSI_S_HSIOM_REG   (*(reg32 *) serial_mosi_s__0__HSIOM)
    #define serial_MOSI_S_HSIOM_PTR   ( (reg32 *) serial_mosi_s__0__HSIOM)
    
    #define serial_MOSI_S_HSIOM_MASK      (serial_mosi_s__0__HSIOM_MASK)
    #define serial_MOSI_S_HSIOM_POS       (serial_mosi_s__0__HSIOM_SHIFT)
    #define serial_MOSI_S_HSIOM_SEL_GPIO  (serial_mosi_s__0__HSIOM_GPIO)
    #define serial_MOSI_S_HSIOM_SEL_SPI   (serial_mosi_s__0__HSIOM_SPI)
#endif /* (serial_SPI_SLAVE_MOSI_PIN) */

#if (serial_SPI_SLAVE_MISO_PIN)
    #define serial_MISO_S_HSIOM_REG   (*(reg32 *) serial_miso_s__0__HSIOM)
    #define serial_MISO_S_HSIOM_PTR   ( (reg32 *) serial_miso_s__0__HSIOM)
    
    #define serial_MISO_S_HSIOM_MASK      (serial_miso_s__0__HSIOM_MASK)
    #define serial_MISO_S_HSIOM_POS       (serial_miso_s__0__HSIOM_SHIFT)
    #define serial_MISO_S_HSIOM_SEL_GPIO  (serial_miso_s__0__HSIOM_GPIO)
    #define serial_MISO_S_HSIOM_SEL_SPI   (serial_miso_s__0__HSIOM_SPI)
#endif /* (serial_SPI_SLAVE_MISO_PIN) */

#if (serial_SPI_MASTER_MISO_PIN)
    #define serial_MISO_M_HSIOM_REG   (*(reg32 *) serial_miso_m__0__HSIOM)
    #define serial_MISO_M_HSIOM_PTR   ( (reg32 *) serial_miso_m__0__HSIOM)
    
    #define serial_MISO_M_HSIOM_MASK      (serial_miso_m__0__HSIOM_MASK)
    #define serial_MISO_M_HSIOM_POS       (serial_miso_m__0__HSIOM_SHIFT)
    #define serial_MISO_M_HSIOM_SEL_GPIO  (serial_miso_m__0__HSIOM_GPIO)
    #define serial_MISO_M_HSIOM_SEL_SPI   (serial_miso_m__0__HSIOM_SPI)
#endif /* (serial_SPI_MASTER_MISO_PIN) */

#if (serial_SPI_MASTER_MOSI_PIN)
    #define serial_MOSI_M_HSIOM_REG   (*(reg32 *) serial_mosi_m__0__HSIOM)
    #define serial_MOSI_M_HSIOM_PTR   ( (reg32 *) serial_mosi_m__0__HSIOM)
    
    #define serial_MOSI_M_HSIOM_MASK      (serial_mosi_m__0__HSIOM_MASK)
    #define serial_MOSI_M_HSIOM_POS       (serial_mosi_m__0__HSIOM_SHIFT)
    #define serial_MOSI_M_HSIOM_SEL_GPIO  (serial_mosi_m__0__HSIOM_GPIO)
    #define serial_MOSI_M_HSIOM_SEL_SPI   (serial_mosi_m__0__HSIOM_SPI)
#endif /* (serial_SPI_MASTER_MOSI_PIN) */

#if (serial_SPI_MASTER_SCLK_PIN)
    #define serial_SCLK_M_HSIOM_REG   (*(reg32 *) serial_sclk_m__0__HSIOM)
    #define serial_SCLK_M_HSIOM_PTR   ( (reg32 *) serial_sclk_m__0__HSIOM)
    
    #define serial_SCLK_M_HSIOM_MASK      (serial_sclk_m__0__HSIOM_MASK)
    #define serial_SCLK_M_HSIOM_POS       (serial_sclk_m__0__HSIOM_SHIFT)
    #define serial_SCLK_M_HSIOM_SEL_GPIO  (serial_sclk_m__0__HSIOM_GPIO)
    #define serial_SCLK_M_HSIOM_SEL_SPI   (serial_sclk_m__0__HSIOM_SPI)
#endif /* (serial_SPI_MASTER_SCLK_PIN) */

#if (serial_SPI_MASTER_SS0_PIN)
    #define serial_SS0_M_HSIOM_REG    (*(reg32 *) serial_ss0_m__0__HSIOM)
    #define serial_SS0_M_HSIOM_PTR    ( (reg32 *) serial_ss0_m__0__HSIOM)
    
    #define serial_SS0_M_HSIOM_MASK       (serial_ss0_m__0__HSIOM_MASK)
    #define serial_SS0_M_HSIOM_POS        (serial_ss0_m__0__HSIOM_SHIFT)
    #define serial_SS0_M_HSIOM_SEL_GPIO   (serial_ss0_m__0__HSIOM_GPIO)
    #define serial_SS0_M_HSIOM_SEL_SPI    (serial_ss0_m__0__HSIOM_SPI)
#endif /* (serial_SPI_MASTER_SS0_PIN) */

#if (serial_SPI_MASTER_SS1_PIN)
    #define serial_SS1_M_HSIOM_REG    (*(reg32 *) serial_ss1_m__0__HSIOM)
    #define serial_SS1_M_HSIOM_PTR    ( (reg32 *) serial_ss1_m__0__HSIOM)
    
    #define serial_SS1_M_HSIOM_MASK       (serial_ss1_m__0__HSIOM_MASK)
    #define serial_SS1_M_HSIOM_POS        (serial_ss1_m__0__HSIOM_SHIFT)
    #define serial_SS1_M_HSIOM_SEL_GPIO   (serial_ss1_m__0__HSIOM_GPIO)
    #define serial_SS1_M_HSIOM_SEL_SPI    (serial_ss1_m__0__HSIOM_SPI)
#endif /* (serial_SPI_MASTER_SS1_PIN) */

#if (serial_SPI_MASTER_SS2_PIN)
    #define serial_SS2_M_HSIOM_REG    (*(reg32 *) serial_ss2_m__0__HSIOM)
    #define serial_SS2_M_HSIOM_PTR    ( (reg32 *) serial_ss2_m__0__HSIOM)
    
    #define serial_SS2_M_HSIOM_MASK       (serial_ss2_m__0__HSIOM_MASK)
    #define serial_SS2_M_HSIOM_POS        (serial_ss2_m__0__HSIOM_SHIFT)
    #define serial_SS2_M_HSIOM_SEL_GPIO   (serial_ss2_m__0__HSIOM_GPIO)
    #define serial_SS2_M_HSIOM_SEL_SPI    (serial_ss2_m__0__HSIOM_SPI)
#endif /* (serial_SPI_MASTER_SS2_PIN) */

#if (serial_SPI_MASTER_SS3_PIN)
    #define serial_SS3_M_HSIOM_REG    (*(reg32 *) serial_ss3_m__0__HSIOM)
    #define serial_SS3_M_HSIOM_PTR    ( (reg32 *) serial_ss3_m__0__HSIOM)
    
    #define serial_SS3_M_HSIOM_MASK      (serial_ss3_m__0__HSIOM_MASK)
    #define serial_SS3_M_HSIOM_POS       (serial_ss3_m__0__HSIOM_SHIFT)
    #define serial_SS3_M_HSIOM_SEL_GPIO  (serial_ss3_m__0__HSIOM_GPIO)
    #define serial_SS3_M_HSIOM_SEL_SPI   (serial_ss3_m__0__HSIOM_SPI)
#endif /* (serial_SPI_MASTER_SS3_PIN) */

#if (serial_UART_RX_PIN)
    #define serial_RX_HSIOM_REG   (*(reg32 *) serial_rx__0__HSIOM)
    #define serial_RX_HSIOM_PTR   ( (reg32 *) serial_rx__0__HSIOM)
    
    #define serial_RX_HSIOM_MASK      (serial_rx__0__HSIOM_MASK)
    #define serial_RX_HSIOM_POS       (serial_rx__0__HSIOM_SHIFT)
    #define serial_RX_HSIOM_SEL_GPIO  (serial_rx__0__HSIOM_GPIO)
    #define serial_RX_HSIOM_SEL_UART  (serial_rx__0__HSIOM_UART)
#endif /* (serial_UART_RX_PIN) */

#if (serial_UART_RX_WAKE_PIN)
    #define serial_RX_WAKE_HSIOM_REG   (*(reg32 *) serial_rx_wake__0__HSIOM)
    #define serial_RX_WAKE_HSIOM_PTR   ( (reg32 *) serial_rx_wake__0__HSIOM)
    
    #define serial_RX_WAKE_HSIOM_MASK      (serial_rx_wake__0__HSIOM_MASK)
    #define serial_RX_WAKE_HSIOM_POS       (serial_rx_wake__0__HSIOM_SHIFT)
    #define serial_RX_WAKE_HSIOM_SEL_GPIO  (serial_rx_wake__0__HSIOM_GPIO)
    #define serial_RX_WAKE_HSIOM_SEL_UART  (serial_rx_wake__0__HSIOM_UART)
#endif /* (serial_UART_WAKE_RX_PIN) */

#if (serial_UART_CTS_PIN)
    #define serial_CTS_HSIOM_REG   (*(reg32 *) serial_cts__0__HSIOM)
    #define serial_CTS_HSIOM_PTR   ( (reg32 *) serial_cts__0__HSIOM)
    
    #define serial_CTS_HSIOM_MASK      (serial_cts__0__HSIOM_MASK)
    #define serial_CTS_HSIOM_POS       (serial_cts__0__HSIOM_SHIFT)
    #define serial_CTS_HSIOM_SEL_GPIO  (serial_cts__0__HSIOM_GPIO)
    #define serial_CTS_HSIOM_SEL_UART  (serial_cts__0__HSIOM_UART)
#endif /* (serial_UART_CTS_PIN) */

#if (serial_UART_TX_PIN)
    #define serial_TX_HSIOM_REG   (*(reg32 *) serial_tx__0__HSIOM)
    #define serial_TX_HSIOM_PTR   ( (reg32 *) serial_tx__0__HSIOM)
    
    #define serial_TX_HSIOM_MASK      (serial_tx__0__HSIOM_MASK)
    #define serial_TX_HSIOM_POS       (serial_tx__0__HSIOM_SHIFT)
    #define serial_TX_HSIOM_SEL_GPIO  (serial_tx__0__HSIOM_GPIO)
    #define serial_TX_HSIOM_SEL_UART  (serial_tx__0__HSIOM_UART)
#endif /* (serial_UART_TX_PIN) */

#if (serial_UART_RX_TX_PIN)
    #define serial_RX_TX_HSIOM_REG   (*(reg32 *) serial_rx_tx__0__HSIOM)
    #define serial_RX_TX_HSIOM_PTR   ( (reg32 *) serial_rx_tx__0__HSIOM)
    
    #define serial_RX_TX_HSIOM_MASK      (serial_rx_tx__0__HSIOM_MASK)
    #define serial_RX_TX_HSIOM_POS       (serial_rx_tx__0__HSIOM_SHIFT)
    #define serial_RX_TX_HSIOM_SEL_GPIO  (serial_rx_tx__0__HSIOM_GPIO)
    #define serial_RX_TX_HSIOM_SEL_UART  (serial_rx_tx__0__HSIOM_UART)
#endif /* (serial_UART_RX_TX_PIN) */

#if (serial_UART_RTS_PIN)
    #define serial_RTS_HSIOM_REG      (*(reg32 *) serial_rts__0__HSIOM)
    #define serial_RTS_HSIOM_PTR      ( (reg32 *) serial_rts__0__HSIOM)
    
    #define serial_RTS_HSIOM_MASK     (serial_rts__0__HSIOM_MASK)
    #define serial_RTS_HSIOM_POS      (serial_rts__0__HSIOM_SHIFT)    
    #define serial_RTS_HSIOM_SEL_GPIO (serial_rts__0__HSIOM_GPIO)
    #define serial_RTS_HSIOM_SEL_UART (serial_rts__0__HSIOM_UART)    
#endif /* (serial_UART_RTS_PIN) */


/***************************************
*        Registers Constants
***************************************/

/* HSIOM switch values. */ 
#define serial_HSIOM_DEF_SEL      (0x00u)
#define serial_HSIOM_GPIO_SEL     (0x00u)
/* The HSIOM values provided below are valid only for serial_CY_SCBIP_V0 
* and serial_CY_SCBIP_V1. It is not recommended to use them for 
* serial_CY_SCBIP_V2. Use pin name specific HSIOM constants provided 
* above instead for any SCB IP block version.
*/
#define serial_HSIOM_UART_SEL     (0x09u)
#define serial_HSIOM_I2C_SEL      (0x0Eu)
#define serial_HSIOM_SPI_SEL      (0x0Fu)

/* Pins settings index. */
#define serial_RX_WAKE_SCL_MOSI_PIN_INDEX   (0u)
#define serial_RX_SCL_MOSI_PIN_INDEX       (0u)
#define serial_TX_SDA_MISO_PIN_INDEX       (1u)
#define serial_SCLK_PIN_INDEX       (2u)
#define serial_SS0_PIN_INDEX       (3u)
#define serial_SS1_PIN_INDEX                  (4u)
#define serial_SS2_PIN_INDEX                  (5u)
#define serial_SS3_PIN_INDEX                  (6u)

/* Pins settings mask. */
#define serial_RX_WAKE_SCL_MOSI_PIN_MASK ((uint32) 0x01u << serial_RX_WAKE_SCL_MOSI_PIN_INDEX)
#define serial_RX_SCL_MOSI_PIN_MASK     ((uint32) 0x01u << serial_RX_SCL_MOSI_PIN_INDEX)
#define serial_TX_SDA_MISO_PIN_MASK     ((uint32) 0x01u << serial_TX_SDA_MISO_PIN_INDEX)
#define serial_SCLK_PIN_MASK     ((uint32) 0x01u << serial_SCLK_PIN_INDEX)
#define serial_SS0_PIN_MASK     ((uint32) 0x01u << serial_SS0_PIN_INDEX)
#define serial_SS1_PIN_MASK                ((uint32) 0x01u << serial_SS1_PIN_INDEX)
#define serial_SS2_PIN_MASK                ((uint32) 0x01u << serial_SS2_PIN_INDEX)
#define serial_SS3_PIN_MASK                ((uint32) 0x01u << serial_SS3_PIN_INDEX)

/* Pin interrupt constants. */
#define serial_INTCFG_TYPE_MASK           (0x03u)
#define serial_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin Drive Mode constants. */
#define serial_PIN_DM_ALG_HIZ  (0u)
#define serial_PIN_DM_DIG_HIZ  (1u)
#define serial_PIN_DM_OD_LO    (4u)
#define serial_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Return drive mode of the pin */
#define serial_DM_MASK    (0x7u)
#define serial_DM_SIZE    (3u)
#define serial_GET_P4_PIN_DM(reg, pos) \
    ( ((reg) & (uint32) ((uint32) serial_DM_MASK << (serial_DM_SIZE * (pos)))) >> \
                                                              (serial_DM_SIZE * (pos)) )

#if (serial_TX_SDA_MISO_PIN)
    #define serial_CHECK_TX_SDA_MISO_PIN_USED \
                (serial_PIN_DM_ALG_HIZ != \
                    serial_GET_P4_PIN_DM(serial_uart_tx_i2c_sda_spi_miso_PC, \
                                                   serial_uart_tx_i2c_sda_spi_miso_SHIFT))
#endif /* (serial_TX_SDA_MISO_PIN) */

#if (serial_SS0_PIN)
    #define serial_CHECK_SS0_PIN_USED \
                (serial_PIN_DM_ALG_HIZ != \
                    serial_GET_P4_PIN_DM(serial_spi_ss0_PC, \
                                                   serial_spi_ss0_SHIFT))
#endif /* (serial_SS0_PIN) */

/* Set bits-mask in register */
#define serial_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define serial_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define serial_SET_HSIOM_SEL(reg, mask, pos, sel) serial_SET_REGISTER_BITS(reg, mask, pos, sel)
#define serial_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        serial_SET_REGISTER_BITS(reg, mask, pos, intType)
#define serial_SET_INP_DIS(reg, mask, val) serial_SET_REGISTER_BIT(reg, mask, val)

/* serial_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  serial_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if (serial_CY_SCBIP_V0)
#if (serial_I2C_PINS)
    #define serial_SET_I2C_SCL_DR(val) serial_scl_Write(val)

    #define serial_SET_I2C_SCL_HSIOM_SEL(sel) \
                          serial_SET_HSIOM_SEL(serial_SCL_HSIOM_REG,  \
                                                         serial_SCL_HSIOM_MASK, \
                                                         serial_SCL_HSIOM_POS,  \
                                                         (sel))
    #define serial_WAIT_SCL_SET_HIGH  (0u == serial_scl_Read())

/* Unconfigured SCB: scl signal */
#elif (serial_RX_WAKE_SCL_MOSI_PIN)
    #define serial_SET_I2C_SCL_DR(val) \
                            serial_uart_rx_wake_i2c_scl_spi_mosi_Write(val)

    #define serial_SET_I2C_SCL_HSIOM_SEL(sel) \
                    serial_SET_HSIOM_SEL(serial_RX_WAKE_SCL_MOSI_HSIOM_REG,  \
                                                   serial_RX_WAKE_SCL_MOSI_HSIOM_MASK, \
                                                   serial_RX_WAKE_SCL_MOSI_HSIOM_POS,  \
                                                   (sel))

    #define serial_WAIT_SCL_SET_HIGH  (0u == serial_uart_rx_wake_i2c_scl_spi_mosi_Read())

#elif (serial_RX_SCL_MOSI_PIN)
    #define serial_SET_I2C_SCL_DR(val) \
                            serial_uart_rx_i2c_scl_spi_mosi_Write(val)


    #define serial_SET_I2C_SCL_HSIOM_SEL(sel) \
                            serial_SET_HSIOM_SEL(serial_RX_SCL_MOSI_HSIOM_REG,  \
                                                           serial_RX_SCL_MOSI_HSIOM_MASK, \
                                                           serial_RX_SCL_MOSI_HSIOM_POS,  \
                                                           (sel))

    #define serial_WAIT_SCL_SET_HIGH  (0u == serial_uart_rx_i2c_scl_spi_mosi_Read())

#else
    #define serial_SET_I2C_SCL_DR(val)        do{ /* Does nothing */ }while(0)
    #define serial_SET_I2C_SCL_HSIOM_SEL(sel) do{ /* Does nothing */ }while(0)

    #define serial_WAIT_SCL_SET_HIGH  (0u)
#endif /* (serial_I2C_PINS) */

/* SCB I2C: sda signal */
#if (serial_I2C_PINS)
    #define serial_WAIT_SDA_SET_HIGH  (0u == serial_sda_Read())
/* Unconfigured SCB: sda signal */
#elif (serial_TX_SDA_MISO_PIN)
    #define serial_WAIT_SDA_SET_HIGH  (0u == serial_uart_tx_i2c_sda_spi_miso_Read())
#else
    #define serial_WAIT_SDA_SET_HIGH  (0u)
#endif /* (serial_MOSI_SCL_RX_PIN) */
#endif /* (serial_CY_SCBIP_V0) */

/* Clear UART wakeup source */
#if (serial_RX_SCL_MOSI_PIN)
    #define serial_CLEAR_UART_RX_WAKE_INTR        do{ /* Does nothing */ }while(0)
    
#elif (serial_RX_WAKE_SCL_MOSI_PIN)
    #define serial_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) serial_uart_rx_wake_i2c_scl_spi_mosi_ClearInterrupt(); \
            }while(0)

#elif(serial_UART_RX_WAKE_PIN)
    #define serial_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) serial_rx_wake_ClearInterrupt(); \
            }while(0)
#else
#endif /* (serial_RX_SCL_MOSI_PIN) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Unconfigured pins */
#define serial_REMOVE_MOSI_SCL_RX_WAKE_PIN    serial_REMOVE_RX_WAKE_SCL_MOSI_PIN
#define serial_REMOVE_MOSI_SCL_RX_PIN         serial_REMOVE_RX_SCL_MOSI_PIN
#define serial_REMOVE_MISO_SDA_TX_PIN         serial_REMOVE_TX_SDA_MISO_PIN
#ifndef serial_REMOVE_SCLK_PIN
#define serial_REMOVE_SCLK_PIN                serial_REMOVE_SCLK_PIN
#endif /* serial_REMOVE_SCLK_PIN */
#ifndef serial_REMOVE_SS0_PIN
#define serial_REMOVE_SS0_PIN                 serial_REMOVE_SS0_PIN
#endif /* serial_REMOVE_SS0_PIN */

/* Unconfigured pins */
#define serial_MOSI_SCL_RX_WAKE_PIN   serial_RX_WAKE_SCL_MOSI_PIN
#define serial_MOSI_SCL_RX_PIN        serial_RX_SCL_MOSI_PIN
#define serial_MISO_SDA_TX_PIN        serial_TX_SDA_MISO_PIN
#ifndef serial_SCLK_PIN
#define serial_SCLK_PIN               serial_SCLK_PIN
#endif /* serial_SCLK_PIN */
#ifndef serial_SS0_PIN
#define serial_SS0_PIN                serial_SS0_PIN
#endif /* serial_SS0_PIN */

#if (serial_MOSI_SCL_RX_WAKE_PIN)
    #define serial_MOSI_SCL_RX_WAKE_HSIOM_REG     serial_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define serial_MOSI_SCL_RX_WAKE_HSIOM_PTR     serial_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define serial_MOSI_SCL_RX_WAKE_HSIOM_MASK    serial_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define serial_MOSI_SCL_RX_WAKE_HSIOM_POS     serial_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define serial_MOSI_SCL_RX_WAKE_INTCFG_REG    serial_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define serial_MOSI_SCL_RX_WAKE_INTCFG_PTR    serial_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define serial_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS   serial_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define serial_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK  serial_RX_WAKE_SCL_MOSI_HSIOM_REG
#endif /* (serial_RX_WAKE_SCL_MOSI_PIN) */

#if (serial_MOSI_SCL_RX_PIN)
    #define serial_MOSI_SCL_RX_HSIOM_REG      serial_RX_SCL_MOSI_HSIOM_REG
    #define serial_MOSI_SCL_RX_HSIOM_PTR      serial_RX_SCL_MOSI_HSIOM_PTR
    #define serial_MOSI_SCL_RX_HSIOM_MASK     serial_RX_SCL_MOSI_HSIOM_MASK
    #define serial_MOSI_SCL_RX_HSIOM_POS      serial_RX_SCL_MOSI_HSIOM_POS
#endif /* (serial_MOSI_SCL_RX_PIN) */

#if (serial_MISO_SDA_TX_PIN)
    #define serial_MISO_SDA_TX_HSIOM_REG      serial_TX_SDA_MISO_HSIOM_REG
    #define serial_MISO_SDA_TX_HSIOM_PTR      serial_TX_SDA_MISO_HSIOM_REG
    #define serial_MISO_SDA_TX_HSIOM_MASK     serial_TX_SDA_MISO_HSIOM_REG
    #define serial_MISO_SDA_TX_HSIOM_POS      serial_TX_SDA_MISO_HSIOM_REG
#endif /* (serial_MISO_SDA_TX_PIN_PIN) */

#if (serial_SCLK_PIN)
    #ifndef serial_SCLK_HSIOM_REG
    #define serial_SCLK_HSIOM_REG     serial_SCLK_HSIOM_REG
    #define serial_SCLK_HSIOM_PTR     serial_SCLK_HSIOM_PTR
    #define serial_SCLK_HSIOM_MASK    serial_SCLK_HSIOM_MASK
    #define serial_SCLK_HSIOM_POS     serial_SCLK_HSIOM_POS
    #endif /* serial_SCLK_HSIOM_REG */
#endif /* (serial_SCLK_PIN) */

#if (serial_SS0_PIN)
    #ifndef serial_SS0_HSIOM_REG
    #define serial_SS0_HSIOM_REG      serial_SS0_HSIOM_REG
    #define serial_SS0_HSIOM_PTR      serial_SS0_HSIOM_PTR
    #define serial_SS0_HSIOM_MASK     serial_SS0_HSIOM_MASK
    #define serial_SS0_HSIOM_POS      serial_SS0_HSIOM_POS
    #endif /* serial_SS0_HSIOM_REG */
#endif /* (serial_SS0_PIN) */

#define serial_MOSI_SCL_RX_WAKE_PIN_INDEX serial_RX_WAKE_SCL_MOSI_PIN_INDEX
#define serial_MOSI_SCL_RX_PIN_INDEX      serial_RX_SCL_MOSI_PIN_INDEX
#define serial_MISO_SDA_TX_PIN_INDEX      serial_TX_SDA_MISO_PIN_INDEX
#ifndef serial_SCLK_PIN_INDEX
#define serial_SCLK_PIN_INDEX             serial_SCLK_PIN_INDEX
#endif /* serial_SCLK_PIN_INDEX */
#ifndef serial_SS0_PIN_INDEX
#define serial_SS0_PIN_INDEX              serial_SS0_PIN_INDEX
#endif /* serial_SS0_PIN_INDEX */

#define serial_MOSI_SCL_RX_WAKE_PIN_MASK serial_RX_WAKE_SCL_MOSI_PIN_MASK
#define serial_MOSI_SCL_RX_PIN_MASK      serial_RX_SCL_MOSI_PIN_MASK
#define serial_MISO_SDA_TX_PIN_MASK      serial_TX_SDA_MISO_PIN_MASK
#ifndef serial_SCLK_PIN_MASK
#define serial_SCLK_PIN_MASK             serial_SCLK_PIN_MASK
#endif /* serial_SCLK_PIN_MASK */
#ifndef serial_SS0_PIN_MASK
#define serial_SS0_PIN_MASK              serial_SS0_PIN_MASK
#endif /* serial_SS0_PIN_MASK */

#endif /* (CY_SCB_PINS_serial_H) */


/* [] END OF FILE */
