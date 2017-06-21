/*******************************************************************************
* File Name: serial.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "serial.h"
#if (serial_INTERNAL_CLOCK_USED)
    #include "serial_IntClock.h"
#endif /* End serial_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 serial_initVar = 0u;

#if (serial_TX_INTERRUPT_ENABLED && serial_TX_ENABLED)
    volatile uint8 serial_txBuffer[serial_TX_BUFFER_SIZE];
    volatile uint8 serial_txBufferRead = 0u;
    uint8 serial_txBufferWrite = 0u;
#endif /* (serial_TX_INTERRUPT_ENABLED && serial_TX_ENABLED) */

#if (serial_RX_INTERRUPT_ENABLED && (serial_RX_ENABLED || serial_HD_ENABLED))
    uint8 serial_errorStatus = 0u;
    volatile uint8 serial_rxBuffer[serial_RX_BUFFER_SIZE];
    volatile uint8 serial_rxBufferRead  = 0u;
    volatile uint8 serial_rxBufferWrite = 0u;
    volatile uint8 serial_rxBufferLoopDetect = 0u;
    volatile uint8 serial_rxBufferOverflow   = 0u;
    #if (serial_RXHW_ADDRESS_ENABLED)
        volatile uint8 serial_rxAddressMode = serial_RX_ADDRESS_MODE;
        volatile uint8 serial_rxAddressDetected = 0u;
    #endif /* (serial_RXHW_ADDRESS_ENABLED) */
#endif /* (serial_RX_INTERRUPT_ENABLED && (serial_RX_ENABLED || serial_HD_ENABLED)) */


/*******************************************************************************
* Function Name: serial_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  serial_Start() sets the initVar variable, calls the
*  serial_Init() function, and then calls the
*  serial_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The serial_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time serial_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the serial_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void serial_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(serial_initVar == 0u)
    {
        serial_Init();
        serial_initVar = 1u;
    }

    serial_Enable();
}


/*******************************************************************************
* Function Name: serial_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call serial_Init() because
*  the serial_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void serial_Init(void) 
{
    #if(serial_RX_ENABLED || serial_HD_ENABLED)

        #if (serial_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(serial_RX_VECT_NUM, &serial_RXISR);
            CyIntSetPriority(serial_RX_VECT_NUM, serial_RX_PRIOR_NUM);
            serial_errorStatus = 0u;
        #endif /* (serial_RX_INTERRUPT_ENABLED) */

        #if (serial_RXHW_ADDRESS_ENABLED)
            serial_SetRxAddressMode(serial_RX_ADDRESS_MODE);
            serial_SetRxAddress1(serial_RX_HW_ADDRESS1);
            serial_SetRxAddress2(serial_RX_HW_ADDRESS2);
        #endif /* End serial_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        serial_RXBITCTR_PERIOD_REG = serial_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        serial_RXSTATUS_MASK_REG  = serial_INIT_RX_INTERRUPTS_MASK;
    #endif /* End serial_RX_ENABLED || serial_HD_ENABLED*/

    #if(serial_TX_ENABLED)
        #if (serial_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(serial_TX_VECT_NUM, &serial_TXISR);
            CyIntSetPriority(serial_TX_VECT_NUM, serial_TX_PRIOR_NUM);
        #endif /* (serial_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (serial_TXCLKGEN_DP)
            serial_TXBITCLKGEN_CTR_REG = serial_BIT_CENTER;
            serial_TXBITCLKTX_COMPLETE_REG = ((serial_NUMBER_OF_DATA_BITS +
                        serial_NUMBER_OF_START_BIT) * serial_OVER_SAMPLE_COUNT) - 1u;
        #else
            serial_TXBITCTR_PERIOD_REG = ((serial_NUMBER_OF_DATA_BITS +
                        serial_NUMBER_OF_START_BIT) * serial_OVER_SAMPLE_8) - 1u;
        #endif /* End serial_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (serial_TX_INTERRUPT_ENABLED)
            serial_TXSTATUS_MASK_REG = serial_TX_STS_FIFO_EMPTY;
        #else
            serial_TXSTATUS_MASK_REG = serial_INIT_TX_INTERRUPTS_MASK;
        #endif /*End serial_TX_INTERRUPT_ENABLED*/

    #endif /* End serial_TX_ENABLED */

    #if(serial_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        serial_WriteControlRegister( \
            (serial_ReadControlRegister() & (uint8)~serial_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(serial_PARITY_TYPE << serial_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End serial_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: serial_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call serial_Enable() because the serial_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  serial_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void serial_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (serial_RX_ENABLED || serial_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        serial_RXBITCTR_CONTROL_REG |= serial_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        serial_RXSTATUS_ACTL_REG  |= serial_INT_ENABLE;

        #if (serial_RX_INTERRUPT_ENABLED)
            serial_EnableRxInt();

            #if (serial_RXHW_ADDRESS_ENABLED)
                serial_rxAddressDetected = 0u;
            #endif /* (serial_RXHW_ADDRESS_ENABLED) */
        #endif /* (serial_RX_INTERRUPT_ENABLED) */
    #endif /* (serial_RX_ENABLED || serial_HD_ENABLED) */

    #if(serial_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!serial_TXCLKGEN_DP)
            serial_TXBITCTR_CONTROL_REG |= serial_CNTR_ENABLE;
        #endif /* End serial_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        serial_TXSTATUS_ACTL_REG |= serial_INT_ENABLE;
        #if (serial_TX_INTERRUPT_ENABLED)
            serial_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            serial_EnableTxInt();
        #endif /* (serial_TX_INTERRUPT_ENABLED) */
     #endif /* (serial_TX_INTERRUPT_ENABLED) */

    #if (serial_INTERNAL_CLOCK_USED)
        serial_IntClock_Start();  /* Enable the clock */
    #endif /* (serial_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: serial_Stop
********************************************************************************
*
* Summary:
*  Disables the UART operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void serial_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (serial_RX_ENABLED || serial_HD_ENABLED)
        serial_RXBITCTR_CONTROL_REG &= (uint8) ~serial_CNTR_ENABLE;
    #endif /* (serial_RX_ENABLED || serial_HD_ENABLED) */

    #if (serial_TX_ENABLED)
        #if(!serial_TXCLKGEN_DP)
            serial_TXBITCTR_CONTROL_REG &= (uint8) ~serial_CNTR_ENABLE;
        #endif /* (!serial_TXCLKGEN_DP) */
    #endif /* (serial_TX_ENABLED) */

    #if (serial_INTERNAL_CLOCK_USED)
        serial_IntClock_Stop();   /* Disable the clock */
    #endif /* (serial_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (serial_RX_ENABLED || serial_HD_ENABLED)
        serial_RXSTATUS_ACTL_REG  &= (uint8) ~serial_INT_ENABLE;

        #if (serial_RX_INTERRUPT_ENABLED)
            serial_DisableRxInt();
        #endif /* (serial_RX_INTERRUPT_ENABLED) */
    #endif /* (serial_RX_ENABLED || serial_HD_ENABLED) */

    #if (serial_TX_ENABLED)
        serial_TXSTATUS_ACTL_REG &= (uint8) ~serial_INT_ENABLE;

        #if (serial_TX_INTERRUPT_ENABLED)
            serial_DisableTxInt();
        #endif /* (serial_TX_INTERRUPT_ENABLED) */
    #endif /* (serial_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: serial_ReadControlRegister
********************************************************************************
*
* Summary:
*  Returns the current value of the control register.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the control register.
*
*******************************************************************************/
uint8 serial_ReadControlRegister(void) 
{
    #if (serial_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(serial_CONTROL_REG);
    #endif /* (serial_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: serial_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  serial_WriteControlRegister(uint8 control) 
{
    #if (serial_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       serial_CONTROL_REG = control;
    #endif /* (serial_CONTROL_REG_REMOVED) */
}


#if(serial_RX_ENABLED || serial_HD_ENABLED)
    /*******************************************************************************
    * Function Name: serial_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      serial_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      serial_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      serial_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      serial_RX_STS_BREAK            Interrupt on break.
    *      serial_RX_STS_OVERRUN          Interrupt on overrun error.
    *      serial_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      serial_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void serial_SetRxInterruptMode(uint8 intSrc) 
    {
        serial_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: serial_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns the next byte of received data. This function returns data without
    *  checking the status. You must check the status separately.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  serial_rxBuffer - RAM buffer pointer for save received data.
    *  serial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  serial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  serial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 serial_ReadRxData(void) 
    {
        uint8 rxData;

    #if (serial_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        serial_DisableRxInt();

        locRxBufferRead  = serial_rxBufferRead;
        locRxBufferWrite = serial_rxBufferWrite;

        if( (serial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = serial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= serial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            serial_rxBufferRead = locRxBufferRead;

            if(serial_rxBufferLoopDetect != 0u)
            {
                serial_rxBufferLoopDetect = 0u;
                #if ((serial_RX_INTERRUPT_ENABLED) && (serial_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( serial_HD_ENABLED )
                        if((serial_CONTROL_REG & serial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            serial_RXSTATUS_MASK_REG  |= serial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        serial_RXSTATUS_MASK_REG  |= serial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end serial_HD_ENABLED */
                #endif /* ((serial_RX_INTERRUPT_ENABLED) && (serial_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = serial_RXDATA_REG;
        }

        serial_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = serial_RXDATA_REG;

    #endif /* (serial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: serial_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the current state of the receiver status register and the software
    *  buffer overflow status.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Side Effect:
    *  All status register bits are clear-on-read except
    *  serial_RX_STS_FIFO_NOTEMPTY.
    *  serial_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  serial_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   serial_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   serial_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 serial_ReadRxStatus(void) 
    {
        uint8 status;

        status = serial_RXSTATUS_REG & serial_RX_HW_MASK;

    #if (serial_RX_INTERRUPT_ENABLED)
        if(serial_rxBufferOverflow != 0u)
        {
            status |= serial_RX_STS_SOFT_BUFF_OVER;
            serial_rxBufferOverflow = 0u;
        }
    #endif /* (serial_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: serial_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. serial_GetChar() is
    *  designed for ASCII characters and returns a uint8 where 1 to 255 are values
    *  for valid characters and 0 indicates an error occurred or no data is present.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  serial_rxBuffer - RAM buffer pointer for save received data.
    *  serial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  serial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  serial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 serial_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (serial_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        serial_DisableRxInt();

        locRxBufferRead  = serial_rxBufferRead;
        locRxBufferWrite = serial_rxBufferWrite;

        if( (serial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = serial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= serial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            serial_rxBufferRead = locRxBufferRead;

            if(serial_rxBufferLoopDetect != 0u)
            {
                serial_rxBufferLoopDetect = 0u;
                #if( (serial_RX_INTERRUPT_ENABLED) && (serial_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( serial_HD_ENABLED )
                        if((serial_CONTROL_REG & serial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            serial_RXSTATUS_MASK_REG |= serial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        serial_RXSTATUS_MASK_REG |= serial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end serial_HD_ENABLED */
                #endif /* serial_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = serial_RXSTATUS_REG;
            if((rxStatus & serial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = serial_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (serial_RX_STS_BREAK | serial_RX_STS_PAR_ERROR |
                                serial_RX_STS_STOP_ERROR | serial_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        serial_EnableRxInt();

    #else

        rxStatus =serial_RXSTATUS_REG;
        if((rxStatus & serial_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = serial_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (serial_RX_STS_BREAK | serial_RX_STS_PAR_ERROR |
                            serial_RX_STS_STOP_ERROR | serial_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (serial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: serial_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, returns received character and error
    *  condition.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains status and LSB contains UART RX data. If the MSB is nonzero,
    *  an error has occurred.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 serial_GetByte(void) 
    {
        
    #if (serial_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        serial_DisableRxInt();
        locErrorStatus = (uint16)serial_errorStatus;
        serial_errorStatus = 0u;
        serial_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | serial_ReadRxData() );
    #else
        return ( ((uint16)serial_ReadRxStatus() << 8u) | serial_ReadRxData() );
    #endif /* serial_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: serial_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received bytes available in the RX buffer.
    *  * RX software buffer is disabled (RX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty RX FIFO or 1 for not empty RX FIFO.
    *  * RX software buffer is enabled: returns the number of bytes available in 
    *    the RX software buffer. Bytes available in the RX FIFO do not take to 
    *    account.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Number of bytes in the RX buffer. 
    *    Return value type depends on RX Buffer Size parameter.
    *
    * Global Variables:
    *  serial_rxBufferWrite - used to calculate left bytes.
    *  serial_rxBufferRead - used to calculate left bytes.
    *  serial_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 serial_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (serial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        serial_DisableRxInt();

        if(serial_rxBufferRead == serial_rxBufferWrite)
        {
            if(serial_rxBufferLoopDetect != 0u)
            {
                size = serial_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(serial_rxBufferRead < serial_rxBufferWrite)
        {
            size = (serial_rxBufferWrite - serial_rxBufferRead);
        }
        else
        {
            size = (serial_RX_BUFFER_SIZE - serial_rxBufferRead) + serial_rxBufferWrite;
        }

        serial_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((serial_RXSTATUS_REG & serial_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (serial_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: serial_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receiver memory buffer and hardware RX FIFO of all received data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_rxBufferWrite - cleared to zero.
    *  serial_rxBufferRead - cleared to zero.
    *  serial_rxBufferLoopDetect - cleared to zero.
    *  serial_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *
    *******************************************************************************/
    void serial_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        serial_RXDATA_AUX_CTL_REG |= (uint8)  serial_RX_FIFO_CLR;
        serial_RXDATA_AUX_CTL_REG &= (uint8) ~serial_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (serial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        serial_DisableRxInt();

        serial_rxBufferRead = 0u;
        serial_rxBufferWrite = 0u;
        serial_rxBufferLoopDetect = 0u;
        serial_rxBufferOverflow = 0u;

        serial_EnableRxInt();

    #endif /* (serial_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: serial_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  serial__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  serial__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  serial__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  serial__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  serial__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  serial_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void serial_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(serial_RXHW_ADDRESS_ENABLED)
            #if(serial_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* serial_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = serial_CONTROL_REG & (uint8)~serial_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << serial_CTRL_RXADDR_MODE0_SHIFT);
                serial_CONTROL_REG = tmpCtrl;

                #if(serial_RX_INTERRUPT_ENABLED && \
                   (serial_RXBUFFERSIZE > serial_FIFO_LENGTH) )
                    serial_rxAddressMode = addressMode;
                    serial_rxAddressDetected = 0u;
                #endif /* End serial_RXBUFFERSIZE > serial_FIFO_LENGTH*/
            #endif /* End serial_CONTROL_REG_REMOVED */
        #else /* serial_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End serial_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: serial_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Sets the first of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #1 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void serial_SetRxAddress1(uint8 address) 
    {
        serial_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: serial_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Sets the second of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #2 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void serial_SetRxAddress2(uint8 address) 
    {
        serial_RXADDRESS2_REG = address;
    }

#endif  /* serial_RX_ENABLED || serial_HD_ENABLED*/


#if( (serial_TX_ENABLED) || (serial_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: serial_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   serial_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   serial_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   serial_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   serial_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void serial_SetTxInterruptMode(uint8 intSrc) 
    {
        serial_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: serial_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data into the transmit buffer to be sent when the bus is
    *  available without checking the TX status register. You must check status
    *  separately.
    *
    * Parameters:
    *  txDataByte: data byte
    *
    * Return:
    * None.
    *
    * Global Variables:
    *  serial_txBuffer - RAM buffer pointer for save data for transmission
    *  serial_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  serial_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  serial_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void serial_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(serial_initVar != 0u)
        {
        #if (serial_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            serial_DisableTxInt();

            if( (serial_txBufferRead == serial_txBufferWrite) &&
                ((serial_TXSTATUS_REG & serial_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                serial_TXDATA_REG = txDataByte;
            }
            else
            {
                if(serial_txBufferWrite >= serial_TX_BUFFER_SIZE)
                {
                    serial_txBufferWrite = 0u;
                }

                serial_txBuffer[serial_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                serial_txBufferWrite++;
            }

            serial_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            serial_TXDATA_REG = txDataByte;

        #endif /*(serial_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: serial_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Reads the status register for the TX portion of the UART.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the TX status register, which is cleared on read.
    *  It is up to the user to handle all bits in this return value accordingly,
    *  even if the bit was not enabled as an interrupt source the event happened
    *  and must be handled accordingly.
    *
    *******************************************************************************/
    uint8 serial_ReadTxStatus(void) 
    {
        return(serial_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: serial_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Puts a byte of data into the transmit buffer to be sent when the bus is
    *  available. This is a blocking API that waits until the TX buffer has room to
    *  hold the data.
    *
    * Parameters:
    *  txDataByte: Byte containing the data to transmit
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_txBuffer - RAM buffer pointer for save data for transmission
    *  serial_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  serial_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void serial_PutChar(uint8 txDataByte) 
    {
    #if (serial_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            serial_DisableTxInt();
        #endif /* (serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = serial_txBufferWrite;
            locTxBufferRead  = serial_txBufferRead;

        #if ((serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            serial_EnableTxInt();
        #endif /* (serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(serial_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((serial_TXSTATUS_REG & serial_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            serial_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= serial_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            serial_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            serial_DisableTxInt();
        #endif /* (serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            serial_txBufferWrite = locTxBufferWrite;

        #if ((serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3))
            serial_EnableTxInt();
        #endif /* (serial_TX_BUFFER_SIZE > serial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (serial_TXSTATUS_REG & serial_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                serial_SetPendingTxInt();
            }
        }

    #else

        while((serial_TXSTATUS_REG & serial_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        serial_TXDATA_REG = txDataByte;

    #endif /* serial_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: serial_PutString
    ********************************************************************************
    *
    * Summary:
    *  Sends a NULL terminated string to the TX buffer for transmission.
    *
    * Parameters:
    *  string[]: Pointer to the null terminated string array residing in RAM or ROM
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void serial_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(serial_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                serial_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: serial_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Places N bytes of data from a memory array into the TX buffer for
    *  transmission.
    *
    * Parameters:
    *  string[]: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of bytes to be transmitted. The type depends on TX Buffer
    *             Size parameter.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void serial_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(serial_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                serial_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: serial_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Writes a byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) to the transmit buffer.
    *
    * Parameters:
    *  txDataByte: Data byte to transmit before the carriage return and line feed.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void serial_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(serial_initVar != 0u)
        {
            serial_PutChar(txDataByte);
            serial_PutChar(0x0Du);
            serial_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: serial_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes in the TX buffer which are waiting to be 
    *  transmitted.
    *  * TX software buffer is disabled (TX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty TX FIFO, 1 for not full TX FIFO or 4 for full TX FIFO.
    *  * TX software buffer is enabled: returns the number of bytes in the TX 
    *    software buffer which are waiting to be transmitted. Bytes available in the
    *    TX FIFO do not count.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Number of bytes used in the TX buffer. Return value type depends on the TX 
    *  Buffer Size parameter.
    *
    * Global Variables:
    *  serial_txBufferWrite - used to calculate left space.
    *  serial_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 serial_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (serial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        serial_DisableTxInt();

        if(serial_txBufferRead == serial_txBufferWrite)
        {
            size = 0u;
        }
        else if(serial_txBufferRead < serial_txBufferWrite)
        {
            size = (serial_txBufferWrite - serial_txBufferRead);
        }
        else
        {
            size = (serial_TX_BUFFER_SIZE - serial_txBufferRead) +
                    serial_txBufferWrite;
        }

        serial_EnableTxInt();

    #else

        size = serial_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & serial_TX_STS_FIFO_FULL) != 0u)
        {
            size = serial_FIFO_LENGTH;
        }
        else if((size & serial_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (serial_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: serial_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears all data from the TX buffer and hardware TX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_txBufferWrite - cleared to zero.
    *  serial_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Data waiting in the transmit buffer is not sent; a byte that is currently
    *  transmitting finishes transmitting.
    *
    *******************************************************************************/
    void serial_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        serial_TXDATA_AUX_CTL_REG |= (uint8)  serial_TX_FIFO_CLR;
        serial_TXDATA_AUX_CTL_REG &= (uint8) ~serial_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (serial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        serial_DisableTxInt();

        serial_txBufferRead = 0u;
        serial_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        serial_EnableTxInt();

    #endif /* (serial_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: serial_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   serial_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   serial_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   serial_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   serial_SEND_WAIT_REINIT - Performs both options: 
    *      serial_SEND_BREAK and serial_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  serial_initVar - checked to identify that the component has been
    *     initialized.
    *  txPeriod - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  There are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Function will block CPU until transmission
    *     complete.
    *  2) User may want to use blocking time if UART configured to the low speed
    *     operation
    *     Example for this case:
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to initialize and use the interrupt to
    *     complete break operation.
    *     Example for this case:
    *     Initialize TX interrupt with "TX - On TX Complete" parameter
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     When interrupt appear with serial_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The serial_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void serial_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(serial_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(serial_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == serial_SEND_BREAK) ||
                (retMode == serial_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                serial_WriteControlRegister(serial_ReadControlRegister() |
                                                      serial_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                serial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = serial_TXSTATUS_REG;
                }
                while((tmpStat & serial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == serial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = serial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & serial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == serial_REINIT) ||
                (retMode == serial_SEND_WAIT_REINIT) )
            {
                serial_WriteControlRegister(serial_ReadControlRegister() &
                                              (uint8)~serial_CTRL_HD_SEND_BREAK);
            }

        #else /* serial_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == serial_SEND_BREAK) ||
                (retMode == serial_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (serial_PARITY_TYPE != serial__B_UART__NONE_REVB) || \
                                    (serial_PARITY_TYPE_SW != 0u) )
                    serial_WriteControlRegister(serial_ReadControlRegister() |
                                                          serial_CTRL_HD_SEND_BREAK);
                #endif /* End serial_PARITY_TYPE != serial__B_UART__NONE_REVB  */

                #if(serial_TXCLKGEN_DP)
                    txPeriod = serial_TXBITCLKTX_COMPLETE_REG;
                    serial_TXBITCLKTX_COMPLETE_REG = serial_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = serial_TXBITCTR_PERIOD_REG;
                    serial_TXBITCTR_PERIOD_REG = serial_TXBITCTR_BREAKBITS8X;
                #endif /* End serial_TXCLKGEN_DP */

                /* Send zeros */
                serial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = serial_TXSTATUS_REG;
                }
                while((tmpStat & serial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == serial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = serial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & serial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == serial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == serial_REINIT) ||
                (retMode == serial_SEND_WAIT_REINIT) )
            {

            #if(serial_TXCLKGEN_DP)
                serial_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                serial_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End serial_TXCLKGEN_DP */

            #if( (serial_PARITY_TYPE != serial__B_UART__NONE_REVB) || \
                 (serial_PARITY_TYPE_SW != 0u) )
                serial_WriteControlRegister(serial_ReadControlRegister() &
                                                      (uint8) ~serial_CTRL_HD_SEND_BREAK);
            #endif /* End serial_PARITY_TYPE != NONE */
            }
        #endif    /* End serial_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: serial_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       serial_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       serial_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears serial_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void serial_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( serial_CONTROL_REG_REMOVED == 0u )
            serial_WriteControlRegister(serial_ReadControlRegister() |
                                                  serial_CTRL_MARK);
        #endif /* End serial_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( serial_CONTROL_REG_REMOVED == 0u )
            serial_WriteControlRegister(serial_ReadControlRegister() &
                                                  (uint8) ~serial_CTRL_MARK);
        #endif /* End serial_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* Endserial_TX_ENABLED */

#if(serial_HD_ENABLED)


    /*******************************************************************************
    * Function Name: serial_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the receiver configuration in half duplex mode. After calling this
    *  function, the UART is ready to receive data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the transmitter
    *  configuration.
    *
    *******************************************************************************/
    void serial_LoadRxConfig(void) 
    {
        serial_WriteControlRegister(serial_ReadControlRegister() &
                                                (uint8)~serial_CTRL_HD_SEND);
        serial_RXBITCTR_PERIOD_REG = serial_HD_RXBITCTR_INIT;

    #if (serial_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        serial_SetRxInterruptMode(serial_INIT_RX_INTERRUPTS_MASK);
    #endif /* (serial_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: serial_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the transmitter configuration in half duplex mode. After calling this
    *  function, the UART is ready to transmit data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the receiver configuration.
    *
    *******************************************************************************/
    void serial_LoadTxConfig(void) 
    {
    #if (serial_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        serial_SetRxInterruptMode(0u);
    #endif /* (serial_RX_INTERRUPT_ENABLED) */

        serial_WriteControlRegister(serial_ReadControlRegister() | serial_CTRL_HD_SEND);
        serial_RXBITCTR_PERIOD_REG = serial_HD_TXBITCTR_INIT;
    }

#endif  /* serial_HD_ENABLED */


/* [] END OF FILE */
