/***************************************************************************//**
* \file serial_UART.c
* \version 3.20
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  UART mode.
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
#include "serial_SPI_UART_PVT.h"
#include "cyapicallbacks.h"

#if (serial_UART_WAKE_ENABLE_CONST && serial_UART_RX_WAKEUP_IRQ)
    /**
    * \addtogroup group_globals
    * \{
    */
    /** This global variable determines whether to enable Skip Start
    * functionality when serial_Sleep() function is called:
    * 0 – disable, other values – enable. Default value is 1.
    * It is only available when Enable wakeup from Deep Sleep Mode is enabled.
    */
    uint8 serial_skipStart = 1u;
    /** \} globals */
#endif /* (serial_UART_WAKE_ENABLE_CONST && serial_UART_RX_WAKEUP_IRQ) */

#if(serial_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    const serial_UART_INIT_STRUCT serial_configUart =
    {
        serial_UART_SUB_MODE,
        serial_UART_DIRECTION,
        serial_UART_DATA_BITS_NUM,
        serial_UART_PARITY_TYPE,
        serial_UART_STOP_BITS_NUM,
        serial_UART_OVS_FACTOR,
        serial_UART_IRDA_LOW_POWER,
        serial_UART_MEDIAN_FILTER_ENABLE,
        serial_UART_RETRY_ON_NACK,
        serial_UART_IRDA_POLARITY,
        serial_UART_DROP_ON_PARITY_ERR,
        serial_UART_DROP_ON_FRAME_ERR,
        serial_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        serial_UART_MP_MODE_ENABLE,
        serial_UART_MP_ACCEPT_ADDRESS,
        serial_UART_MP_RX_ADDRESS,
        serial_UART_MP_RX_ADDRESS_MASK,
        (uint32) serial_SCB_IRQ_INTERNAL,
        serial_UART_INTR_RX_MASK,
        serial_UART_RX_TRIGGER_LEVEL,
        serial_UART_INTR_TX_MASK,
        serial_UART_TX_TRIGGER_LEVEL,
        (uint8) serial_UART_BYTE_MODE_ENABLE,
        (uint8) serial_UART_CTS_ENABLE,
        (uint8) serial_UART_CTS_POLARITY,
        (uint8) serial_UART_RTS_POLARITY,
        (uint8) serial_UART_RTS_FIFO_LEVEL
    };


    /*******************************************************************************
    * Function Name: serial_UartInit
    ****************************************************************************//**
    *
    *  Configures the serial for UART operation.
    *
    *  This function is intended specifically to be used when the serial
    *  configuration is set to “Unconfigured serial” in the customizer.
    *  After initializing the serial in UART mode using this function,
    *  the component can be enabled using the serial_Start() or
    * serial_Enable() function.
    *  This function uses a pointer to a structure that provides the configuration
    *  settings. This structure contains the same information that would otherwise
    *  be provided by the customizer settings.
    *
    *  \param config: pointer to a structure that contains the following list of
    *   fields. These fields match the selections available in the customizer.
    *   Refer to the customizer for further description of the settings.
    *
    *******************************************************************************/
    void serial_UartInit(const serial_UART_INIT_STRUCT *config)
    {
        uint32 pinsConfig;

        if (NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Get direction to configure UART pins: TX, RX or TX+RX */
            pinsConfig  = config->direction;

        #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
            /* Add RTS and CTS pins to configure */
            pinsConfig |= (0u != config->rtsRxFifoLevel) ? (serial_UART_RTS_PIN_ENABLE) : (0u);
            pinsConfig |= (0u != config->enableCts)      ? (serial_UART_CTS_PIN_ENABLE) : (0u);
        #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */

            /* Configure pins */
            serial_SetPins(serial_SCB_MODE_UART, config->mode, pinsConfig);

            /* Store internal configuration */
            serial_scbMode       = (uint8) serial_SCB_MODE_UART;
            serial_scbEnableWake = (uint8) config->enableWake;
            serial_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            serial_rxBuffer      =         config->rxBuffer;
            serial_rxDataBits    = (uint8) config->dataBits;
            serial_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            serial_txBuffer      =         config->txBuffer;
            serial_txDataBits    = (uint8) config->dataBits;
            serial_txBufferSize  = (uint8) config->txBufferSize;

            /* Configure UART interface */
            if(serial_UART_MODE_IRDA == config->mode)
            {
                /* OVS settings: IrDA */
                serial_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (serial_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (serial_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settings: UART and SmartCard */
                serial_CTRL_REG  = serial_GET_CTRL_OVS(config->oversample);
            }

            serial_CTRL_REG     |= serial_GET_CTRL_BYTE_MODE  (config->enableByteMode)      |
                                             serial_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             serial_CTRL_UART;

            /* Configure sub-mode: UART, SmartCard or IrDA */
            serial_UART_CTRL_REG = serial_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            serial_UART_RX_CTRL_REG = serial_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        serial_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        serial_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        serial_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        serial_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(serial_UART_PARITY_NONE != config->parity)
            {
               serial_UART_RX_CTRL_REG |= serial_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    serial_UART_RX_CTRL_PARITY_ENABLED;
            }

            serial_RX_CTRL_REG      = serial_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                serial_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                serial_GET_UART_RX_CTRL_ENABLED(config->direction);

            serial_RX_FIFO_CTRL_REG = serial_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            serial_RX_MATCH_REG     = serial_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                serial_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            serial_UART_TX_CTRL_REG = serial_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                serial_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(serial_UART_PARITY_NONE != config->parity)
            {
               serial_UART_TX_CTRL_REG |= serial_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    serial_UART_TX_CTRL_PARITY_ENABLED;
            }

            serial_TX_CTRL_REG      = serial_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                serial_GET_UART_TX_CTRL_ENABLED(config->direction);

            serial_TX_FIFO_CTRL_REG = serial_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);

        #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
            serial_UART_FLOW_CTRL_REG = serial_GET_UART_FLOW_CTRL_CTS_ENABLE(config->enableCts) | \
                                            serial_GET_UART_FLOW_CTRL_CTS_POLARITY (config->ctsPolarity)  | \
                                            serial_GET_UART_FLOW_CTRL_RTS_POLARITY (config->rtsPolarity)  | \
                                            serial_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(config->rtsRxFifoLevel);
        #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */

            /* Configure interrupt with UART handler but do not enable it */
            CyIntDisable    (serial_ISR_NUMBER);
            CyIntSetPriority(serial_ISR_NUMBER, serial_ISR_PRIORITY);
            (void) CyIntSetVector(serial_ISR_NUMBER, &serial_SPI_UART_ISR);

            /* Configure WAKE interrupt */
        #if(serial_UART_RX_WAKEUP_IRQ)
            CyIntDisable    (serial_RX_WAKE_ISR_NUMBER);
            CyIntSetPriority(serial_RX_WAKE_ISR_NUMBER, serial_RX_WAKE_ISR_PRIORITY);
            (void) CyIntSetVector(serial_RX_WAKE_ISR_NUMBER, &serial_UART_WAKEUP_ISR);
        #endif /* (serial_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt sources */
            serial_INTR_I2C_EC_MASK_REG = serial_NO_INTR_SOURCES;
            serial_INTR_SPI_EC_MASK_REG = serial_NO_INTR_SOURCES;
            serial_INTR_SLAVE_MASK_REG  = serial_NO_INTR_SOURCES;
            serial_INTR_MASTER_MASK_REG = serial_NO_INTR_SOURCES;
            serial_INTR_RX_MASK_REG     = config->rxInterruptMask;
            serial_INTR_TX_MASK_REG     = config->txInterruptMask;
        
            /* Configure TX interrupt sources to restore. */
            serial_IntrTxMask = LO16(serial_INTR_TX_MASK_REG);

            /* Clear RX buffer indexes */
            serial_rxBufferHead     = 0u;
            serial_rxBufferTail     = 0u;
            serial_rxBufferOverflow = 0u;

            /* Clear TX buffer indexes */
            serial_txBufferHead = 0u;
            serial_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: serial_UartInit
    ****************************************************************************//**
    *
    *  Configures the SCB for the UART operation.
    *
    *******************************************************************************/
    void serial_UartInit(void)
    {
        /* Configure UART interface */
        serial_CTRL_REG = serial_UART_DEFAULT_CTRL;

        /* Configure sub-mode: UART, SmartCard or IrDA */
        serial_UART_CTRL_REG = serial_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        serial_UART_RX_CTRL_REG = serial_UART_DEFAULT_UART_RX_CTRL;
        serial_RX_CTRL_REG      = serial_UART_DEFAULT_RX_CTRL;
        serial_RX_FIFO_CTRL_REG = serial_UART_DEFAULT_RX_FIFO_CTRL;
        serial_RX_MATCH_REG     = serial_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        serial_UART_TX_CTRL_REG = serial_UART_DEFAULT_UART_TX_CTRL;
        serial_TX_CTRL_REG      = serial_UART_DEFAULT_TX_CTRL;
        serial_TX_FIFO_CTRL_REG = serial_UART_DEFAULT_TX_FIFO_CTRL;

    #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
        serial_UART_FLOW_CTRL_REG = serial_UART_DEFAULT_FLOW_CTRL;
    #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */

        /* Configure interrupt with UART handler but do not enable it */
    #if(serial_SCB_IRQ_INTERNAL)
        CyIntDisable    (serial_ISR_NUMBER);
        CyIntSetPriority(serial_ISR_NUMBER, serial_ISR_PRIORITY);
        (void) CyIntSetVector(serial_ISR_NUMBER, &serial_SPI_UART_ISR);
    #endif /* (serial_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
    #if(serial_UART_RX_WAKEUP_IRQ)
        CyIntDisable    (serial_RX_WAKE_ISR_NUMBER);
        CyIntSetPriority(serial_RX_WAKE_ISR_NUMBER, serial_RX_WAKE_ISR_PRIORITY);
        (void) CyIntSetVector(serial_RX_WAKE_ISR_NUMBER, &serial_UART_WAKEUP_ISR);
    #endif /* (serial_UART_RX_WAKEUP_IRQ) */

        /* Configure interrupt sources */
        serial_INTR_I2C_EC_MASK_REG = serial_UART_DEFAULT_INTR_I2C_EC_MASK;
        serial_INTR_SPI_EC_MASK_REG = serial_UART_DEFAULT_INTR_SPI_EC_MASK;
        serial_INTR_SLAVE_MASK_REG  = serial_UART_DEFAULT_INTR_SLAVE_MASK;
        serial_INTR_MASTER_MASK_REG = serial_UART_DEFAULT_INTR_MASTER_MASK;
        serial_INTR_RX_MASK_REG     = serial_UART_DEFAULT_INTR_RX_MASK;
        serial_INTR_TX_MASK_REG     = serial_UART_DEFAULT_INTR_TX_MASK;
    
        /* Configure TX interrupt sources to restore. */
        serial_IntrTxMask = LO16(serial_INTR_TX_MASK_REG);

    #if(serial_INTERNAL_RX_SW_BUFFER_CONST)
        serial_rxBufferHead     = 0u;
        serial_rxBufferTail     = 0u;
        serial_rxBufferOverflow = 0u;
    #endif /* (serial_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(serial_INTERNAL_TX_SW_BUFFER_CONST)
        serial_txBufferHead = 0u;
        serial_txBufferTail = 0u;
    #endif /* (serial_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: serial_UartPostEnable
****************************************************************************//**
*
*  Restores HSIOM settings for the UART output pins (TX and/or RTS) to be
*  controlled by the SCB UART.
*
*******************************************************************************/
void serial_UartPostEnable(void)
{
#if (serial_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (serial_TX_SDA_MISO_PIN)
        if (serial_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set SCB UART to drive the output pin */
            serial_SET_HSIOM_SEL(serial_TX_SDA_MISO_HSIOM_REG, serial_TX_SDA_MISO_HSIOM_MASK,
                                           serial_TX_SDA_MISO_HSIOM_POS, serial_TX_SDA_MISO_HSIOM_SEL_UART);
        }
    #endif /* (serial_TX_SDA_MISO_PIN_PIN) */

    #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
        #if (serial_SS0_PIN)
            if (serial_CHECK_SS0_PIN_USED)
            {
                /* Set SCB UART to drive the output pin */
                serial_SET_HSIOM_SEL(serial_SS0_HSIOM_REG, serial_SS0_HSIOM_MASK,
                                               serial_SS0_HSIOM_POS, serial_SS0_HSIOM_SEL_UART);
            }
        #endif /* (serial_SS0_PIN) */
    #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */

#else
    #if (serial_UART_TX_PIN)
         /* Set SCB UART to drive the output pin */
        serial_SET_HSIOM_SEL(serial_TX_HSIOM_REG, serial_TX_HSIOM_MASK,
                                       serial_TX_HSIOM_POS, serial_TX_HSIOM_SEL_UART);
    #endif /* (serial_UART_TX_PIN) */

    #if (serial_UART_RTS_PIN)
        /* Set SCB UART to drive the output pin */
        serial_SET_HSIOM_SEL(serial_RTS_HSIOM_REG, serial_RTS_HSIOM_MASK,
                                       serial_RTS_HSIOM_POS, serial_RTS_HSIOM_SEL_UART);
    #endif /* (serial_UART_RTS_PIN) */
#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Restore TX interrupt sources. */
    serial_SetTxInterruptMode(serial_IntrTxMask);
}


/*******************************************************************************
* Function Name: serial_UartStop
****************************************************************************//**
*
*  Changes the HSIOM settings for the UART output pins (TX and/or RTS) to keep
*  them inactive after the block is disabled. The output pins are controlled by
*  the GPIO data register. Also, the function disables the skip start feature
*  to not cause it to trigger after the component is enabled.
*
*******************************************************************************/
void serial_UartStop(void)
{
#if(serial_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (serial_TX_SDA_MISO_PIN)
        if (serial_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set GPIO to drive output pin */
            serial_SET_HSIOM_SEL(serial_TX_SDA_MISO_HSIOM_REG, serial_TX_SDA_MISO_HSIOM_MASK,
                                           serial_TX_SDA_MISO_HSIOM_POS, serial_TX_SDA_MISO_HSIOM_SEL_GPIO);
        }
    #endif /* (serial_TX_SDA_MISO_PIN_PIN) */

    #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
        #if (serial_SS0_PIN)
            if (serial_CHECK_SS0_PIN_USED)
            {
                /* Set output pin state after block is disabled */
                serial_spi_ss0_Write(serial_GET_UART_RTS_INACTIVE);

                /* Set GPIO to drive output pin */
                serial_SET_HSIOM_SEL(serial_SS0_HSIOM_REG, serial_SS0_HSIOM_MASK,
                                               serial_SS0_HSIOM_POS, serial_SS0_HSIOM_SEL_GPIO);
            }
        #endif /* (serial_SS0_PIN) */
    #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */

#else
    #if (serial_UART_TX_PIN)
        /* Set GPIO to drive output pin */
        serial_SET_HSIOM_SEL(serial_TX_HSIOM_REG, serial_TX_HSIOM_MASK,
                                       serial_TX_HSIOM_POS, serial_TX_HSIOM_SEL_GPIO);
    #endif /* (serial_UART_TX_PIN) */

    #if (serial_UART_RTS_PIN)
        /* Set output pin state after block is disabled */
        serial_rts_Write(serial_GET_UART_RTS_INACTIVE);

        /* Set GPIO to drive output pin */
        serial_SET_HSIOM_SEL(serial_RTS_HSIOM_REG, serial_RTS_HSIOM_MASK,
                                       serial_RTS_HSIOM_POS, serial_RTS_HSIOM_SEL_GPIO);
    #endif /* (serial_UART_RTS_PIN) */

#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (serial_UART_WAKE_ENABLE_CONST)
    /* Disable skip start feature used for wakeup */
    serial_UART_RX_CTRL_REG &= (uint32) ~serial_UART_RX_CTRL_SKIP_START;
#endif /* (serial_UART_WAKE_ENABLE_CONST) */

    /* Store TX interrupt sources (exclude level triggered). */
    serial_IntrTxMask = LO16(serial_GetTxInterruptMode() & serial_INTR_UART_TX_RESTORE);
}


/*******************************************************************************
* Function Name: serial_UartSetRxAddress
****************************************************************************//**
*
*  Sets the hardware detectable receiver address for the UART in the
*  Multiprocessor mode.
*
*  \param address: Address for hardware address detection.
*
*******************************************************************************/
void serial_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = serial_RX_MATCH_REG;

    matchReg &= ((uint32) ~serial_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & serial_RX_MATCH_ADDR_MASK)); /* Set address  */

    serial_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: serial_UartSetRxAddressMask
****************************************************************************//**
*
*  Sets the hardware address mask for the UART in the Multiprocessor mode.
*
*  \param addressMask: Address mask.
*   - Bit value 0 – excludes bit from address comparison.
*   - Bit value 1 – the bit needs to match with the corresponding bit
*     of the address.
*
*******************************************************************************/
void serial_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = serial_RX_MATCH_REG;

    matchReg &= ((uint32) ~serial_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << serial_RX_MATCH_MASK_POS));

    serial_RX_MATCH_REG = matchReg;
}


#if(serial_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: serial_UartGetChar
    ****************************************************************************//**
    *
    *  Retrieves next data element from receive buffer.
    *  This function is designed for ASCII characters and returns a char where
    *  1 to 255 are valid characters and 0 indicates an error occurred or no data
    *  is present.
    *  - RX software buffer is disabled: Returns data element retrieved from RX
    *    FIFO.
    *  - RX software buffer is enabled: Returns data element from the software
    *    receive buffer.
    *
    *  \return
    *   Next data element from the receive buffer. ASCII character values from
    *   1 to 255 are valid. A returned zero signifies an error condition or no
    *   data available.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check serial_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 serial_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Reads data only if there is data to read */
        if (0u != serial_SpiUartGetRxBufferSize())
        {
            rxData = serial_SpiUartReadRxData();
        }

        if (serial_CHECK_INTR_RX(serial_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occurred: returns zero */
            serial_ClearRxInterruptSource(serial_INTR_RX_ERR);
        }

        return (rxData);
    }


    /*******************************************************************************
    * Function Name: serial_UartGetByte
    ****************************************************************************//**
    *
    *  Retrieves the next data element from the receive buffer, returns the
    *  received byte and error condition.
    *   - The RX software buffer is disabled: returns the data element retrieved
    *     from the RX FIFO. Undefined data will be returned if the RX FIFO is
    *     empty.
    *   - The RX software buffer is enabled: returns data element from the
    *     software receive buffer.
    *
    *  \return
    *   Bits 7-0 contain the next data element from the receive buffer and
    *   other bits contain the error condition.
    *   - serial_UART_RX_OVERFLOW - Attempt to write to a full
    *     receiver FIFO.
    *   - serial_UART_RX_UNDERFLOW	Attempt to read from an empty
    *     receiver FIFO.
    *   - serial_UART_RX_FRAME_ERROR - UART framing error detected.
    *   - serial_UART_RX_PARITY_ERROR - UART parity error detected.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check serial_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 serial_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;

        #if (serial_CHECK_RX_SW_BUFFER)
        {
            serial_DisableInt();
        }
        #endif

        if (0u != serial_SpiUartGetRxBufferSize())
        {
            /* Enables interrupt to receive more bytes: at least one byte is in
            * buffer.
            */
            #if (serial_CHECK_RX_SW_BUFFER)
            {
                serial_EnableInt();
            }
            #endif

            /* Get received byte */
            rxData = serial_SpiUartReadRxData();
        }
        else
        {
            /* Reads a byte directly from RX FIFO: underflow is raised in the
            * case of empty. Otherwise the first received byte will be read.
            */
            rxData = serial_RX_FIFO_RD_REG;


            /* Enables interrupt to receive more bytes. */
            #if (serial_CHECK_RX_SW_BUFFER)
            {

                /* The byte has been read from RX FIFO. Clear RX interrupt to
                * not involve interrupt handler when RX FIFO is empty.
                */
                serial_ClearRxInterruptSource(serial_INTR_RX_NOT_EMPTY);

                serial_EnableInt();
            }
            #endif
        }

        /* Get and clear RX error mask */
        tmpStatus = (serial_GetRxInterruptSource() & serial_INTR_RX_ERR);
        serial_ClearRxInterruptSource(serial_INTR_RX_ERR);

        /* Puts together data and error status:
        * MP mode and accept address: 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return (rxData);
    }


    #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: serial_UartSetRtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of RTS output signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param polarity: Active polarity of RTS output signal.
        *   - serial_UART_RTS_ACTIVE_LOW  - RTS signal is active low.
        *   - serial_UART_RTS_ACTIVE_HIGH - RTS signal is active high.
        *
        *******************************************************************************/
        void serial_UartSetRtsPolarity(uint32 polarity)
        {
            if(0u != polarity)
            {
                serial_UART_FLOW_CTRL_REG |= (uint32)  serial_UART_FLOW_CTRL_RTS_POLARITY;
            }
            else
            {
                serial_UART_FLOW_CTRL_REG &= (uint32) ~serial_UART_FLOW_CTRL_RTS_POLARITY;
            }
        }


        /*******************************************************************************
        * Function Name: serial_UartSetRtsFifoLevel
        ****************************************************************************//**
        *
        *  Sets level in the RX FIFO for RTS signal activation.
        *  While the RX FIFO has fewer entries than the RX FIFO level the RTS signal
        *  remains active, otherwise the RTS signal becomes inactive.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param level: Level in the RX FIFO for RTS signal activation.
        *   The range of valid level values is between 0 and RX FIFO depth - 1.
        *   Setting level value to 0 disables RTS signal activation.
        *
        *******************************************************************************/
        void serial_UartSetRtsFifoLevel(uint32 level)
        {
            uint32 uartFlowCtrl;

            uartFlowCtrl = serial_UART_FLOW_CTRL_REG;

            uartFlowCtrl &= ((uint32) ~serial_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
            uartFlowCtrl |= ((uint32) (serial_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK & level));

            serial_UART_FLOW_CTRL_REG = uartFlowCtrl;
        }
    #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */

#endif /* (serial_UART_RX_DIRECTION) */


#if(serial_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: serial_UartPutString
    ****************************************************************************//**
    *
    *  Places a NULL terminated string in the transmit buffer to be sent at the
    *  next available bus time.
    *  This function is blocking and waits until there is a space available to put
    *  requested data in transmit buffer.
    *
    *  \param string: pointer to the null terminated string array to be placed in the
    *   transmit buffer.
    *
    *******************************************************************************/
    void serial_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data has been sent */
        while(string[bufIndex] != ((char8) 0))
        {
            serial_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: serial_UartPutCRLF
    ****************************************************************************//**
    *
    *  Places byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) in the transmit buffer.
    *  This function is blocking and waits until there is a space available to put
    *  all requested data in transmit buffer.
    *
    *  \param txDataByte: the data to be transmitted.
    *
    *******************************************************************************/
    void serial_UartPutCRLF(uint32 txDataByte)
    {
        serial_UartPutChar(txDataByte);  /* Blocks control flow until all data has been sent */
        serial_UartPutChar(0x0Du);       /* Blocks control flow until all data has been sent */
        serial_UartPutChar(0x0Au);       /* Blocks control flow until all data has been sent */
    }


    #if !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: serialSCB_UartEnableCts
        ****************************************************************************//**
        *
        *  Enables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void serial_UartEnableCts(void)
        {
            serial_UART_FLOW_CTRL_REG |= (uint32)  serial_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: serial_UartDisableCts
        ****************************************************************************//**
        *
        *  Disables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void serial_UartDisableCts(void)
        {
            serial_UART_FLOW_CTRL_REG &= (uint32) ~serial_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: serial_UartSetCtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of CTS input signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param polarity: Active polarity of CTS output signal.
        *   - serial_UART_CTS_ACTIVE_LOW  - CTS signal is active low.
        *   - serial_UART_CTS_ACTIVE_HIGH - CTS signal is active high.
        *
        *******************************************************************************/
        void serial_UartSetCtsPolarity(uint32 polarity)
        {
            if (0u != polarity)
            {
                serial_UART_FLOW_CTRL_REG |= (uint32)  serial_UART_FLOW_CTRL_CTS_POLARITY;
            }
            else
            {
                serial_UART_FLOW_CTRL_REG &= (uint32) ~serial_UART_FLOW_CTRL_CTS_POLARITY;
            }
        }
    #endif /* !(serial_CY_SCBIP_V0 || serial_CY_SCBIP_V1) */

#endif /* (serial_UART_TX_DIRECTION) */


#if (serial_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: serial_UartSaveConfig
    ****************************************************************************//**
    *
    *  Clears and enables an interrupt on a falling edge of the Rx input. The GPIO
    *  interrupt does not track in the active mode, therefore requires to be 
    *  cleared by this API.
    *
    *******************************************************************************/
    void serial_UartSaveConfig(void)
    {
    #if (serial_UART_RX_WAKEUP_IRQ)
        /* Set SKIP_START if requested (set by default). */
        if (0u != serial_skipStart)
        {
            serial_UART_RX_CTRL_REG |= (uint32)  serial_UART_RX_CTRL_SKIP_START;
        }
        else
        {
            serial_UART_RX_CTRL_REG &= (uint32) ~serial_UART_RX_CTRL_SKIP_START;
        }
        
        /* Clear RX GPIO interrupt status and pending interrupt in NVIC because
        * falling edge on RX line occurs while UART communication in active mode.
        * Enable interrupt: next interrupt trigger should wakeup device.
        */
        serial_CLEAR_UART_RX_WAKE_INTR;
        serial_RxWakeClearPendingInt();
        serial_RxWakeEnableInt();
    #endif /* (serial_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: serial_UartRestoreConfig
    ****************************************************************************//**
    *
    *  Disables the RX GPIO interrupt. Until this function is called the interrupt
    *  remains active and triggers on every falling edge of the UART RX line.
    *
    *******************************************************************************/
    void serial_UartRestoreConfig(void)
    {
    #if (serial_UART_RX_WAKEUP_IRQ)
        /* Disable interrupt: no more triggers in active mode */
        serial_RxWakeDisableInt();
    #endif /* (serial_UART_RX_WAKEUP_IRQ) */
    }


    #if (serial_UART_RX_WAKEUP_IRQ)
        /*******************************************************************************
        * Function Name: serial_UART_WAKEUP_ISR
        ****************************************************************************//**
        *
        *  Handles the Interrupt Service Routine for the SCB UART mode GPIO wakeup
        *  event. This event is configured to trigger on a falling edge of the RX line.
        *
        *******************************************************************************/
        CY_ISR(serial_UART_WAKEUP_ISR)
        {
        #ifdef serial_UART_WAKEUP_ISR_ENTRY_CALLBACK
            serial_UART_WAKEUP_ISR_EntryCallback();
        #endif /* serial_UART_WAKEUP_ISR_ENTRY_CALLBACK */

            serial_CLEAR_UART_RX_WAKE_INTR;

        #ifdef serial_UART_WAKEUP_ISR_EXIT_CALLBACK
            serial_UART_WAKEUP_ISR_ExitCallback();
        #endif /* serial_UART_WAKEUP_ISR_EXIT_CALLBACK */
        }
    #endif /* (serial_UART_RX_WAKEUP_IRQ) */
#endif /* (serial_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
