/***************************************************************************//**
* \file serial.c
* \version 3.20
*
* \brief
*  This file provides the source code to the API for the SCB Component.
*
* Note:
*
*******************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "serial_PVT.h"

#if (serial_SCB_MODE_I2C_INC)
    #include "serial_I2C_PVT.h"
#endif /* (serial_SCB_MODE_I2C_INC) */

#if (serial_SCB_MODE_EZI2C_INC)
    #include "serial_EZI2C_PVT.h"
#endif /* (serial_SCB_MODE_EZI2C_INC) */

#if (serial_SCB_MODE_SPI_INC || serial_SCB_MODE_UART_INC)
    #include "serial_SPI_UART_PVT.h"
#endif /* (serial_SCB_MODE_SPI_INC || serial_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 serial_scbMode = serial_SCB_MODE_UNCONFIG;
    uint8 serial_scbEnableWake;
    uint8 serial_scbEnableIntr;

    /* I2C configuration variables */
    uint8 serial_mode;
    uint8 serial_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * serial_rxBuffer;
    uint8  serial_rxDataBits;
    uint32 serial_rxBufferSize;

    volatile uint8 * serial_txBuffer;
    uint8  serial_txDataBits;
    uint32 serial_txBufferSize;

    /* EZI2C configuration variables */
    uint8 serial_numberOfAddr;
    uint8 serial_subAddrSize;
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/
/**
* \addtogroup group_general
* \{
*/

/** serial_initVar indicates whether the serial 
*  component has been initialized. The variable is initialized to 0 
*  and set to 1 the first time SCB_Start() is called. This allows 
*  the component to restart without reinitialization after the first 
*  call to the serial_Start() routine.
*
*  If re-initialization of the component is required, then the 
*  serial_Init() function can be called before the 
*  serial_Start() or serial_Enable() function.
*/
uint8 serial_initVar = 0u;


#if (! (serial_SCB_MODE_I2C_CONST_CFG || \
        serial_SCB_MODE_EZI2C_CONST_CFG))
    /** This global variable stores TX interrupt sources after 
    * serial_Stop() is called. Only these TX interrupt sources 
    * will be restored on a subsequent serial_Enable() call.
    */
    uint16 serial_IntrTxMask = 0u;
#endif /* (! (serial_SCB_MODE_I2C_CONST_CFG || \
              serial_SCB_MODE_EZI2C_CONST_CFG)) */
/** \} globals */

#if (serial_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_serial_CUSTOM_INTR_HANDLER)
    void (*serial_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_serial_CUSTOM_INTR_HANDLER) */
#endif /* (serial_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void serial_ScbEnableIntr(void);
static void serial_ScbModeStop(void);
static void serial_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: serial_Init
****************************************************************************//**
*
*  Initializes the serial component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  serial_I2CInit, serial_SpiInit, 
*  serial_UartInit or serial_EzI2CInit.
*
*******************************************************************************/
void serial_Init(void)
{
#if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
    if (serial_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        serial_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (serial_SCB_MODE_I2C_CONST_CFG)
    serial_I2CInit();

#elif (serial_SCB_MODE_SPI_CONST_CFG)
    serial_SpiInit();

#elif (serial_SCB_MODE_UART_CONST_CFG)
    serial_UartInit();

#elif (serial_SCB_MODE_EZI2C_CONST_CFG)
    serial_EzI2CInit();

#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: serial_Enable
****************************************************************************//**
*
*  Enables serial component operation: activates the hardware and 
*  internal interrupt. It also restores TX interrupt sources disabled after the 
*  serial_Stop() function was called (note that level-triggered TX 
*  interrupt sources remain disabled to not cause code lock-up).
*  For I2C and EZI2C modes the interrupt is internal and mandatory for 
*  operation. For SPI and UART modes the interrupt can be configured as none, 
*  internal or external.
*  The serial configuration should be not changed when the component
*  is enabled. Any configuration changes should be made after disabling the 
*  component.
*  When configuration is set to “Unconfigured serial”, the component 
*  must first be initialized to operate in one of the following configurations: 
*  I2C, SPI, UART or EZ I2C, using the mode-specific initialization API. 
*  Otherwise this function does not enable the component.
*
*******************************************************************************/
void serial_Enable(void)
{
#if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!serial_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        serial_CTRL_REG |= serial_CTRL_ENABLED;

        serial_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        serial_ScbModePostEnable();
    }
#else
    serial_CTRL_REG |= serial_CTRL_ENABLED;

    serial_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    serial_ScbModePostEnable();
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: serial_Start
****************************************************************************//**
*
*  Invokes serial_Init() and serial_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZI2C. Otherwise this function does not enable the component.
*
* \globalvars
*  serial_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void serial_Start(void)
{
    if (0u == serial_initVar)
    {
        serial_Init();
        serial_initVar = 1u; /* Component was initialized */
    }

    serial_Enable();
}


/*******************************************************************************
* Function Name: serial_Stop
****************************************************************************//**
*
*  Disables the serial component: disable the hardware and internal 
*  interrupt. It also disables all TX interrupt sources so as not to cause an 
*  unexpected interrupt trigger because after the component is enabled, the 
*  TX FIFO is empty.
*  Refer to the function serial_Enable() for the interrupt 
*  configuration details.
*  This function disables the SCB component without checking to see if 
*  communication is in progress. Before calling this function it may be 
*  necessary to check the status of communication to make sure communication 
*  is complete. If this is not done then communication could be stopped mid 
*  byte and corrupted data could result.
*
*******************************************************************************/
void serial_Stop(void)
{
#if (serial_SCB_IRQ_INTERNAL)
    serial_DisableInt();
#endif /* (serial_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    serial_ScbModeStop();

    /* Disable SCB IP */
    serial_CTRL_REG &= (uint32) ~serial_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger after the component will be enabled because the 
    * TX FIFO is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when it is disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    serial_SetTxInterruptMode(serial_NO_INTR_SOURCES);

#if (serial_SCB_IRQ_INTERNAL)
    serial_ClearPendingInt();
#endif /* (serial_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: serial_SetRxFifoLevel
****************************************************************************//**
*
*  Sets level in the RX FIFO to generate a RX level interrupt.
*  When the RX FIFO has more entries than the RX FIFO level an RX level
*  interrupt request is generated.
*
*  \param level: Level in the RX FIFO to generate RX level interrupt.
*   The range of valid level values is between 0 and RX FIFO depth - 1.
*
*******************************************************************************/
void serial_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = serial_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~serial_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (serial_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    serial_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: serial_SetTxFifoLevel
****************************************************************************//**
*
*  Sets level in the TX FIFO to generate a TX level interrupt.
*  When the TX FIFO has less entries than the TX FIFO level an TX level
*  interrupt request is generated.
*
*  \param level: Level in the TX FIFO to generate TX level interrupt.
*   The range of valid level values is between 0 and TX FIFO depth - 1.
*
*******************************************************************************/
void serial_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = serial_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~serial_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (serial_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    serial_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (serial_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: serial_SetCustomInterruptHandler
    ****************************************************************************//**
    *
    *  Registers a function to be called by the internal interrupt handler.
    *  First the function that is registered is called, then the internal interrupt
    *  handler performs any operation such as software buffer management functions
    *  before the interrupt returns.  It is the user's responsibility not to break
    *  the software buffer operations. Only one custom handler is supported, which
    *  is the function provided by the most recent call.
    *  At the initialization time no custom handler is registered.
    *
    *  \param func: Pointer to the function to register.
    *        The value NULL indicates to remove the current custom interrupt
    *        handler.
    *
    *******************************************************************************/
    void serial_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_serial_CUSTOM_INTR_HANDLER)
        serial_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_serial_CUSTOM_INTR_HANDLER) */
    }
#endif /* (serial_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: serial_ScbModeEnableIntr
****************************************************************************//**
*
*  Enables an interrupt for a specific mode.
*
*******************************************************************************/
static void serial_ScbEnableIntr(void)
{
#if (serial_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != serial_scbEnableIntr)
        {
            serial_EnableInt();
        }

    #else
        serial_EnableInt();

    #endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (serial_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: serial_ScbModePostEnable
****************************************************************************//**
*
*  Calls the PostEnable function for a specific operation mode.
*
*******************************************************************************/
static void serial_ScbModePostEnable(void)
{
#if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!serial_CY_SCBIP_V1)
    if (serial_SCB_MODE_SPI_RUNTM_CFG)
    {
        serial_SpiPostEnable();
    }
    else if (serial_SCB_MODE_UART_RUNTM_CFG)
    {
        serial_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!serial_CY_SCBIP_V1) */

#elif (serial_SCB_MODE_SPI_CONST_CFG)
    serial_SpiPostEnable();

#elif (serial_SCB_MODE_UART_CONST_CFG)
    serial_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: serial_ScbModeStop
****************************************************************************//**
*
*  Calls the Stop function for a specific operation mode.
*
*******************************************************************************/
static void serial_ScbModeStop(void)
{
#if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
    if (serial_SCB_MODE_I2C_RUNTM_CFG)
    {
        serial_I2CStop();
    }
    else if (serial_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        serial_EzI2CStop();
    }
#if (!serial_CY_SCBIP_V1)
    else if (serial_SCB_MODE_SPI_RUNTM_CFG)
    {
        serial_SpiStop();
    }
    else if (serial_SCB_MODE_UART_RUNTM_CFG)
    {
        serial_UartStop();
    }
#endif /* (!serial_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (serial_SCB_MODE_I2C_CONST_CFG)
    serial_I2CStop();

#elif (serial_SCB_MODE_EZI2C_CONST_CFG)
    serial_EzI2CStop();

#elif (serial_SCB_MODE_SPI_CONST_CFG)
    serial_SpiStop();

#elif (serial_SCB_MODE_UART_CONST_CFG)
    serial_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: serial_SetPins
    ****************************************************************************//**
    *
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    *  \param mode:      Mode of SCB operation.
    *  \param subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  \param uartEnableMask: enables TX or RX direction and RTS and CTS signals.
    *
    *******************************************************************************/
    void serial_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 pinsDm[serial_SCB_PINS_NUMBER];
        uint32 i;
        
    #if (!serial_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!serial_CY_SCBIP_V1) */
        
        uint32 hsiomSel[serial_SCB_PINS_NUMBER] = 
        {
            serial_RX_SCL_MOSI_HSIOM_SEL_GPIO,
            serial_TX_SDA_MISO_HSIOM_SEL_GPIO,
            0u,
            0u,
            0u,
            0u,
            0u,
        };

    #if (serial_CY_SCBIP_V1)
        /* Supress compiler warning. */
        if ((0u == subMode) || (0u == uartEnableMask))
        {
        }
    #endif /* (serial_CY_SCBIP_V1) */

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < serial_SCB_PINS_NUMBER; i++)
        {
            pinsDm[i] = serial_PIN_DM_ALG_HIZ;
        }

        if ((serial_SCB_MODE_I2C   == mode) ||
            (serial_SCB_MODE_EZI2C == mode))
        {
        #if (serial_RX_SCL_MOSI_PIN)
            hsiomSel[serial_RX_SCL_MOSI_PIN_INDEX] = serial_RX_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [serial_RX_SCL_MOSI_PIN_INDEX] = serial_PIN_DM_OD_LO;
        #elif (serial_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[serial_RX_WAKE_SCL_MOSI_PIN_INDEX] = serial_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [serial_RX_WAKE_SCL_MOSI_PIN_INDEX] = serial_PIN_DM_OD_LO;
        #else
        #endif /* (serial_RX_SCL_MOSI_PIN) */
        
        #if (serial_TX_SDA_MISO_PIN)
            hsiomSel[serial_TX_SDA_MISO_PIN_INDEX] = serial_TX_SDA_MISO_HSIOM_SEL_I2C;
            pinsDm  [serial_TX_SDA_MISO_PIN_INDEX] = serial_PIN_DM_OD_LO;
        #endif /* (serial_TX_SDA_MISO_PIN) */
        }
    #if (!serial_CY_SCBIP_V1)
        else if (serial_SCB_MODE_SPI == mode)
        {
        #if (serial_RX_SCL_MOSI_PIN)
            hsiomSel[serial_RX_SCL_MOSI_PIN_INDEX] = serial_RX_SCL_MOSI_HSIOM_SEL_SPI;
        #elif (serial_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[serial_RX_WAKE_SCL_MOSI_PIN_INDEX] = serial_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI;
        #else
        #endif /* (serial_RX_SCL_MOSI_PIN) */
        
        #if (serial_TX_SDA_MISO_PIN)
            hsiomSel[serial_TX_SDA_MISO_PIN_INDEX] = serial_TX_SDA_MISO_HSIOM_SEL_SPI;
        #endif /* (serial_TX_SDA_MISO_PIN) */
        
        #if (serial_SCLK_PIN)
            hsiomSel[serial_SCLK_PIN_INDEX] = serial_SCLK_HSIOM_SEL_SPI;
        #endif /* (serial_SCLK_PIN) */

            if (serial_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[serial_RX_SCL_MOSI_PIN_INDEX] = serial_PIN_DM_DIG_HIZ;
                pinsDm[serial_TX_SDA_MISO_PIN_INDEX] = serial_PIN_DM_STRONG;
                pinsDm[serial_SCLK_PIN_INDEX] = serial_PIN_DM_DIG_HIZ;

            #if (serial_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[serial_SS0_PIN_INDEX] = serial_SS0_HSIOM_SEL_SPI;
                pinsDm  [serial_SS0_PIN_INDEX] = serial_PIN_DM_DIG_HIZ;
            #endif /* (serial_SS0_PIN) */

            #if (serial_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= serial_TX_SDA_MISO_PIN_MASK;
            #endif /* (serial_TX_SDA_MISO_PIN) */
            }
            else 
            {
                /* (Master) */
                pinsDm[serial_RX_SCL_MOSI_PIN_INDEX] = serial_PIN_DM_STRONG;
                pinsDm[serial_TX_SDA_MISO_PIN_INDEX] = serial_PIN_DM_DIG_HIZ;
                pinsDm[serial_SCLK_PIN_INDEX] = serial_PIN_DM_STRONG;

            #if (serial_SS0_PIN)
                hsiomSel [serial_SS0_PIN_INDEX] = serial_SS0_HSIOM_SEL_SPI;
                pinsDm   [serial_SS0_PIN_INDEX] = serial_PIN_DM_STRONG;
                pinsInBuf |= serial_SS0_PIN_MASK;
            #endif /* (serial_SS0_PIN) */

            #if (serial_SS1_PIN)
                hsiomSel [serial_SS1_PIN_INDEX] = serial_SS1_HSIOM_SEL_SPI;
                pinsDm   [serial_SS1_PIN_INDEX] = serial_PIN_DM_STRONG;
                pinsInBuf |= serial_SS1_PIN_MASK;
            #endif /* (serial_SS1_PIN) */

            #if (serial_SS2_PIN)
                hsiomSel [serial_SS2_PIN_INDEX] = serial_SS2_HSIOM_SEL_SPI;
                pinsDm   [serial_SS2_PIN_INDEX] = serial_PIN_DM_STRONG;
                pinsInBuf |= serial_SS2_PIN_MASK;
            #endif /* (serial_SS2_PIN) */

            #if (serial_SS3_PIN)
                hsiomSel [serial_SS3_PIN_INDEX] = serial_SS3_HSIOM_SEL_SPI;
                pinsDm   [serial_SS3_PIN_INDEX] = serial_PIN_DM_STRONG;
                pinsInBuf |= serial_SS3_PIN_MASK;
            #endif /* (serial_SS3_PIN) */

                /* Disable input buffers */
            #if (serial_RX_SCL_MOSI_PIN)
                pinsInBuf |= serial_RX_SCL_MOSI_PIN_MASK;
            #elif (serial_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= serial_RX_WAKE_SCL_MOSI_PIN_MASK;
            #else
            #endif /* (serial_RX_SCL_MOSI_PIN) */

            #if (serial_SCLK_PIN)
                pinsInBuf |= serial_SCLK_PIN_MASK;
            #endif /* (serial_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (serial_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
            #if (serial_TX_SDA_MISO_PIN)
                hsiomSel[serial_TX_SDA_MISO_PIN_INDEX] = serial_TX_SDA_MISO_HSIOM_SEL_UART;
                pinsDm  [serial_TX_SDA_MISO_PIN_INDEX] = serial_PIN_DM_OD_LO;
            #endif /* (serial_TX_SDA_MISO_PIN) */
            }
            else /* Standard or IrDA */
            {
                if (0u != (serial_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                #if (serial_RX_SCL_MOSI_PIN)
                    hsiomSel[serial_RX_SCL_MOSI_PIN_INDEX] = serial_RX_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [serial_RX_SCL_MOSI_PIN_INDEX] = serial_PIN_DM_DIG_HIZ;
                #elif (serial_RX_WAKE_SCL_MOSI_PIN)
                    hsiomSel[serial_RX_WAKE_SCL_MOSI_PIN_INDEX] = serial_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [serial_RX_WAKE_SCL_MOSI_PIN_INDEX] = serial_PIN_DM_DIG_HIZ;
                #else
                #endif /* (serial_RX_SCL_MOSI_PIN) */
                }

                if (0u != (serial_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                #if (serial_TX_SDA_MISO_PIN)
                    hsiomSel[serial_TX_SDA_MISO_PIN_INDEX] = serial_TX_SDA_MISO_HSIOM_SEL_UART;
                    pinsDm  [serial_TX_SDA_MISO_PIN_INDEX] = serial_PIN_DM_STRONG;
                    
                    /* Disable input buffer */
                    pinsInBuf |= serial_TX_SDA_MISO_PIN_MASK;
                #endif /* (serial_TX_SDA_MISO_PIN) */
                }

            #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
                if (serial_UART_MODE_STD == subMode)
                {
                    if (0u != (serial_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                    #if (serial_SCLK_PIN)
                        hsiomSel[serial_SCLK_PIN_INDEX] = serial_SCLK_HSIOM_SEL_UART;
                        pinsDm  [serial_SCLK_PIN_INDEX] = serial_PIN_DM_DIG_HIZ;
                    #endif /* (serial_SCLK_PIN) */
                    }

                    if (0u != (serial_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                    #if (serial_SS0_PIN)
                        hsiomSel[serial_SS0_PIN_INDEX] = serial_SS0_HSIOM_SEL_UART;
                        pinsDm  [serial_SS0_PIN_INDEX] = serial_PIN_DM_STRONG;
                        
                        /* Disable input buffer */
                        pinsInBuf |= serial_SS0_PIN_MASK;
                    #endif /* (serial_SS0_PIN) */
                    }
                }
            #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */
            }
        }
    #endif /* (!serial_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (serial_RX_SCL_MOSI_PIN)
        serial_SET_HSIOM_SEL(serial_RX_SCL_MOSI_HSIOM_REG,
                                       serial_RX_SCL_MOSI_HSIOM_MASK,
                                       serial_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[serial_RX_SCL_MOSI_PIN_INDEX]);

        serial_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[serial_RX_SCL_MOSI_PIN_INDEX]);

        #if (!serial_CY_SCBIP_V1)
            serial_SET_INP_DIS(serial_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                         serial_uart_rx_i2c_scl_spi_mosi_MASK,
                                         (0u != (pinsInBuf & serial_RX_SCL_MOSI_PIN_MASK)));
        #endif /* (!serial_CY_SCBIP_V1) */
    
    #elif (serial_RX_WAKE_SCL_MOSI_PIN)
        serial_SET_HSIOM_SEL(serial_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       serial_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       serial_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[serial_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        serial_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[serial_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        serial_SET_INP_DIS(serial_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     serial_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & serial_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        serial_SET_INCFG_TYPE(serial_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        serial_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        serial_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        serial_INTCFG_TYPE_FALLING_EDGE);
    #else
    #endif /* (serial_RX_WAKE_SCL_MOSI_PIN) */

    #if (serial_TX_SDA_MISO_PIN)
        serial_SET_HSIOM_SEL(serial_TX_SDA_MISO_HSIOM_REG,
                                       serial_TX_SDA_MISO_HSIOM_MASK,
                                       serial_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[serial_TX_SDA_MISO_PIN_INDEX]);

        serial_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[serial_TX_SDA_MISO_PIN_INDEX]);

    #if (!serial_CY_SCBIP_V1)
        serial_SET_INP_DIS(serial_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     serial_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & serial_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!serial_CY_SCBIP_V1) */
    #endif /* (serial_RX_SCL_MOSI_PIN) */

    #if (serial_SCLK_PIN)
        serial_SET_HSIOM_SEL(serial_SCLK_HSIOM_REG,
                                       serial_SCLK_HSIOM_MASK,
                                       serial_SCLK_HSIOM_POS,
                                       hsiomSel[serial_SCLK_PIN_INDEX]);

        serial_spi_sclk_SetDriveMode((uint8) pinsDm[serial_SCLK_PIN_INDEX]);

        serial_SET_INP_DIS(serial_spi_sclk_INP_DIS,
                                     serial_spi_sclk_MASK,
                                     (0u != (pinsInBuf & serial_SCLK_PIN_MASK)));
    #endif /* (serial_SCLK_PIN) */

    #if (serial_SS0_PIN)
        serial_SET_HSIOM_SEL(serial_SS0_HSIOM_REG,
                                       serial_SS0_HSIOM_MASK,
                                       serial_SS0_HSIOM_POS,
                                       hsiomSel[serial_SS0_PIN_INDEX]);

        serial_spi_ss0_SetDriveMode((uint8) pinsDm[serial_SS0_PIN_INDEX]);

        serial_SET_INP_DIS(serial_spi_ss0_INP_DIS,
                                     serial_spi_ss0_MASK,
                                     (0u != (pinsInBuf & serial_SS0_PIN_MASK)));
    #endif /* (serial_SS0_PIN) */

    #if (serial_SS1_PIN)
        serial_SET_HSIOM_SEL(serial_SS1_HSIOM_REG,
                                       serial_SS1_HSIOM_MASK,
                                       serial_SS1_HSIOM_POS,
                                       hsiomSel[serial_SS1_PIN_INDEX]);

        serial_spi_ss1_SetDriveMode((uint8) pinsDm[serial_SS1_PIN_INDEX]);

        serial_SET_INP_DIS(serial_spi_ss1_INP_DIS,
                                     serial_spi_ss1_MASK,
                                     (0u != (pinsInBuf & serial_SS1_PIN_MASK)));
    #endif /* (serial_SS1_PIN) */

    #if (serial_SS2_PIN)
        serial_SET_HSIOM_SEL(serial_SS2_HSIOM_REG,
                                       serial_SS2_HSIOM_MASK,
                                       serial_SS2_HSIOM_POS,
                                       hsiomSel[serial_SS2_PIN_INDEX]);

        serial_spi_ss2_SetDriveMode((uint8) pinsDm[serial_SS2_PIN_INDEX]);

        serial_SET_INP_DIS(serial_spi_ss2_INP_DIS,
                                     serial_spi_ss2_MASK,
                                     (0u != (pinsInBuf & serial_SS2_PIN_MASK)));
    #endif /* (serial_SS2_PIN) */

    #if (serial_SS3_PIN)
        serial_SET_HSIOM_SEL(serial_SS3_HSIOM_REG,
                                       serial_SS3_HSIOM_MASK,
                                       serial_SS3_HSIOM_POS,
                                       hsiomSel[serial_SS3_PIN_INDEX]);

        serial_spi_ss3_SetDriveMode((uint8) pinsDm[serial_SS3_PIN_INDEX]);

        serial_SET_INP_DIS(serial_spi_ss3_INP_DIS,
                                     serial_spi_ss3_MASK,
                                     (0u != (pinsInBuf & serial_SS3_PIN_MASK)));
    #endif /* (serial_SS3_PIN) */
    }

#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: serial_I2CSlaveNackGeneration
    ****************************************************************************//**
    *
    *  Sets command to generate NACK to the address or data.
    *
    *******************************************************************************/
    void serial_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (serial_CTRL_REG & serial_CTRL_EC_AM_MODE)) &&
            (0u == (serial_I2C_CTRL_REG & serial_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            serial_CTRL_REG &= ~serial_CTRL_EC_AM_MODE;
            serial_CTRL_REG |=  serial_CTRL_EC_AM_MODE;
        }

        serial_I2C_SLAVE_CMD_REG = serial_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */


/* [] END OF FILE */
