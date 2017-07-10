/*******************************************************************************
* File Name: debug.c
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

#include "debug.h"
#if (debug_INTERNAL_CLOCK_USED)
    #include "debug_IntClock.h"
#endif /* End debug_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 debug_initVar = 0u;

#if (debug_TX_INTERRUPT_ENABLED && debug_TX_ENABLED)
    volatile uint8 debug_txBuffer[debug_TX_BUFFER_SIZE];
    volatile uint8 debug_txBufferRead = 0u;
    uint8 debug_txBufferWrite = 0u;
#endif /* (debug_TX_INTERRUPT_ENABLED && debug_TX_ENABLED) */

#if (debug_RX_INTERRUPT_ENABLED && (debug_RX_ENABLED || debug_HD_ENABLED))
    uint8 debug_errorStatus = 0u;
    volatile uint8 debug_rxBuffer[debug_RX_BUFFER_SIZE];
    volatile uint8 debug_rxBufferRead  = 0u;
    volatile uint8 debug_rxBufferWrite = 0u;
    volatile uint8 debug_rxBufferLoopDetect = 0u;
    volatile uint8 debug_rxBufferOverflow   = 0u;
    #if (debug_RXHW_ADDRESS_ENABLED)
        volatile uint8 debug_rxAddressMode = debug_RX_ADDRESS_MODE;
        volatile uint8 debug_rxAddressDetected = 0u;
    #endif /* (debug_RXHW_ADDRESS_ENABLED) */
#endif /* (debug_RX_INTERRUPT_ENABLED && (debug_RX_ENABLED || debug_HD_ENABLED)) */


/*******************************************************************************
* Function Name: debug_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  debug_Start() sets the initVar variable, calls the
*  debug_Init() function, and then calls the
*  debug_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The debug_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time debug_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the debug_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debug_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(debug_initVar == 0u)
    {
        debug_Init();
        debug_initVar = 1u;
    }

    debug_Enable();
}


/*******************************************************************************
* Function Name: debug_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call debug_Init() because
*  the debug_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void debug_Init(void) 
{
    #if(debug_RX_ENABLED || debug_HD_ENABLED)

        #if (debug_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(debug_RX_VECT_NUM, &debug_RXISR);
            CyIntSetPriority(debug_RX_VECT_NUM, debug_RX_PRIOR_NUM);
            debug_errorStatus = 0u;
        #endif /* (debug_RX_INTERRUPT_ENABLED) */

        #if (debug_RXHW_ADDRESS_ENABLED)
            debug_SetRxAddressMode(debug_RX_ADDRESS_MODE);
            debug_SetRxAddress1(debug_RX_HW_ADDRESS1);
            debug_SetRxAddress2(debug_RX_HW_ADDRESS2);
        #endif /* End debug_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        debug_RXBITCTR_PERIOD_REG = debug_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        debug_RXSTATUS_MASK_REG  = debug_INIT_RX_INTERRUPTS_MASK;
    #endif /* End debug_RX_ENABLED || debug_HD_ENABLED*/

    #if(debug_TX_ENABLED)
        #if (debug_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(debug_TX_VECT_NUM, &debug_TXISR);
            CyIntSetPriority(debug_TX_VECT_NUM, debug_TX_PRIOR_NUM);
        #endif /* (debug_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (debug_TXCLKGEN_DP)
            debug_TXBITCLKGEN_CTR_REG = debug_BIT_CENTER;
            debug_TXBITCLKTX_COMPLETE_REG = ((debug_NUMBER_OF_DATA_BITS +
                        debug_NUMBER_OF_START_BIT) * debug_OVER_SAMPLE_COUNT) - 1u;
        #else
            debug_TXBITCTR_PERIOD_REG = ((debug_NUMBER_OF_DATA_BITS +
                        debug_NUMBER_OF_START_BIT) * debug_OVER_SAMPLE_8) - 1u;
        #endif /* End debug_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (debug_TX_INTERRUPT_ENABLED)
            debug_TXSTATUS_MASK_REG = debug_TX_STS_FIFO_EMPTY;
        #else
            debug_TXSTATUS_MASK_REG = debug_INIT_TX_INTERRUPTS_MASK;
        #endif /*End debug_TX_INTERRUPT_ENABLED*/

    #endif /* End debug_TX_ENABLED */

    #if(debug_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        debug_WriteControlRegister( \
            (debug_ReadControlRegister() & (uint8)~debug_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(debug_PARITY_TYPE << debug_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End debug_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: debug_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call debug_Enable() because the debug_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debug_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void debug_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (debug_RX_ENABLED || debug_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        debug_RXBITCTR_CONTROL_REG |= debug_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        debug_RXSTATUS_ACTL_REG  |= debug_INT_ENABLE;

        #if (debug_RX_INTERRUPT_ENABLED)
            debug_EnableRxInt();

            #if (debug_RXHW_ADDRESS_ENABLED)
                debug_rxAddressDetected = 0u;
            #endif /* (debug_RXHW_ADDRESS_ENABLED) */
        #endif /* (debug_RX_INTERRUPT_ENABLED) */
    #endif /* (debug_RX_ENABLED || debug_HD_ENABLED) */

    #if(debug_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!debug_TXCLKGEN_DP)
            debug_TXBITCTR_CONTROL_REG |= debug_CNTR_ENABLE;
        #endif /* End debug_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        debug_TXSTATUS_ACTL_REG |= debug_INT_ENABLE;
        #if (debug_TX_INTERRUPT_ENABLED)
            debug_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            debug_EnableTxInt();
        #endif /* (debug_TX_INTERRUPT_ENABLED) */
     #endif /* (debug_TX_INTERRUPT_ENABLED) */

    #if (debug_INTERNAL_CLOCK_USED)
        debug_IntClock_Start();  /* Enable the clock */
    #endif /* (debug_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: debug_Stop
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
void debug_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (debug_RX_ENABLED || debug_HD_ENABLED)
        debug_RXBITCTR_CONTROL_REG &= (uint8) ~debug_CNTR_ENABLE;
    #endif /* (debug_RX_ENABLED || debug_HD_ENABLED) */

    #if (debug_TX_ENABLED)
        #if(!debug_TXCLKGEN_DP)
            debug_TXBITCTR_CONTROL_REG &= (uint8) ~debug_CNTR_ENABLE;
        #endif /* (!debug_TXCLKGEN_DP) */
    #endif /* (debug_TX_ENABLED) */

    #if (debug_INTERNAL_CLOCK_USED)
        debug_IntClock_Stop();   /* Disable the clock */
    #endif /* (debug_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (debug_RX_ENABLED || debug_HD_ENABLED)
        debug_RXSTATUS_ACTL_REG  &= (uint8) ~debug_INT_ENABLE;

        #if (debug_RX_INTERRUPT_ENABLED)
            debug_DisableRxInt();
        #endif /* (debug_RX_INTERRUPT_ENABLED) */
    #endif /* (debug_RX_ENABLED || debug_HD_ENABLED) */

    #if (debug_TX_ENABLED)
        debug_TXSTATUS_ACTL_REG &= (uint8) ~debug_INT_ENABLE;

        #if (debug_TX_INTERRUPT_ENABLED)
            debug_DisableTxInt();
        #endif /* (debug_TX_INTERRUPT_ENABLED) */
    #endif /* (debug_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: debug_ReadControlRegister
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
uint8 debug_ReadControlRegister(void) 
{
    #if (debug_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(debug_CONTROL_REG);
    #endif /* (debug_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: debug_WriteControlRegister
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
void  debug_WriteControlRegister(uint8 control) 
{
    #if (debug_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       debug_CONTROL_REG = control;
    #endif /* (debug_CONTROL_REG_REMOVED) */
}


#if(debug_RX_ENABLED || debug_HD_ENABLED)
    /*******************************************************************************
    * Function Name: debug_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      debug_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      debug_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      debug_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      debug_RX_STS_BREAK            Interrupt on break.
    *      debug_RX_STS_OVERRUN          Interrupt on overrun error.
    *      debug_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      debug_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void debug_SetRxInterruptMode(uint8 intSrc) 
    {
        debug_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: debug_ReadRxData
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
    *  debug_rxBuffer - RAM buffer pointer for save received data.
    *  debug_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  debug_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  debug_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 debug_ReadRxData(void) 
    {
        uint8 rxData;

    #if (debug_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        debug_DisableRxInt();

        locRxBufferRead  = debug_rxBufferRead;
        locRxBufferWrite = debug_rxBufferWrite;

        if( (debug_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = debug_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= debug_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            debug_rxBufferRead = locRxBufferRead;

            if(debug_rxBufferLoopDetect != 0u)
            {
                debug_rxBufferLoopDetect = 0u;
                #if ((debug_RX_INTERRUPT_ENABLED) && (debug_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( debug_HD_ENABLED )
                        if((debug_CONTROL_REG & debug_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            debug_RXSTATUS_MASK_REG  |= debug_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        debug_RXSTATUS_MASK_REG  |= debug_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end debug_HD_ENABLED */
                #endif /* ((debug_RX_INTERRUPT_ENABLED) && (debug_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = debug_RXDATA_REG;
        }

        debug_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = debug_RXDATA_REG;

    #endif /* (debug_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: debug_ReadRxStatus
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
    *  debug_RX_STS_FIFO_NOTEMPTY.
    *  debug_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  debug_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   debug_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   debug_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 debug_ReadRxStatus(void) 
    {
        uint8 status;

        status = debug_RXSTATUS_REG & debug_RX_HW_MASK;

    #if (debug_RX_INTERRUPT_ENABLED)
        if(debug_rxBufferOverflow != 0u)
        {
            status |= debug_RX_STS_SOFT_BUFF_OVER;
            debug_rxBufferOverflow = 0u;
        }
    #endif /* (debug_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: debug_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. debug_GetChar() is
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
    *  debug_rxBuffer - RAM buffer pointer for save received data.
    *  debug_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  debug_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  debug_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 debug_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (debug_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        debug_DisableRxInt();

        locRxBufferRead  = debug_rxBufferRead;
        locRxBufferWrite = debug_rxBufferWrite;

        if( (debug_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = debug_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= debug_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            debug_rxBufferRead = locRxBufferRead;

            if(debug_rxBufferLoopDetect != 0u)
            {
                debug_rxBufferLoopDetect = 0u;
                #if( (debug_RX_INTERRUPT_ENABLED) && (debug_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( debug_HD_ENABLED )
                        if((debug_CONTROL_REG & debug_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            debug_RXSTATUS_MASK_REG |= debug_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        debug_RXSTATUS_MASK_REG |= debug_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end debug_HD_ENABLED */
                #endif /* debug_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = debug_RXSTATUS_REG;
            if((rxStatus & debug_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = debug_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (debug_RX_STS_BREAK | debug_RX_STS_PAR_ERROR |
                                debug_RX_STS_STOP_ERROR | debug_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        debug_EnableRxInt();

    #else

        rxStatus =debug_RXSTATUS_REG;
        if((rxStatus & debug_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = debug_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (debug_RX_STS_BREAK | debug_RX_STS_PAR_ERROR |
                            debug_RX_STS_STOP_ERROR | debug_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (debug_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: debug_GetByte
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
    uint16 debug_GetByte(void) 
    {
        
    #if (debug_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        debug_DisableRxInt();
        locErrorStatus = (uint16)debug_errorStatus;
        debug_errorStatus = 0u;
        debug_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | debug_ReadRxData() );
    #else
        return ( ((uint16)debug_ReadRxStatus() << 8u) | debug_ReadRxData() );
    #endif /* debug_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: debug_GetRxBufferSize
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
    *  debug_rxBufferWrite - used to calculate left bytes.
    *  debug_rxBufferRead - used to calculate left bytes.
    *  debug_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 debug_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (debug_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        debug_DisableRxInt();

        if(debug_rxBufferRead == debug_rxBufferWrite)
        {
            if(debug_rxBufferLoopDetect != 0u)
            {
                size = debug_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(debug_rxBufferRead < debug_rxBufferWrite)
        {
            size = (debug_rxBufferWrite - debug_rxBufferRead);
        }
        else
        {
            size = (debug_RX_BUFFER_SIZE - debug_rxBufferRead) + debug_rxBufferWrite;
        }

        debug_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((debug_RXSTATUS_REG & debug_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (debug_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: debug_ClearRxBuffer
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
    *  debug_rxBufferWrite - cleared to zero.
    *  debug_rxBufferRead - cleared to zero.
    *  debug_rxBufferLoopDetect - cleared to zero.
    *  debug_rxBufferOverflow - cleared to zero.
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
    void debug_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        debug_RXDATA_AUX_CTL_REG |= (uint8)  debug_RX_FIFO_CLR;
        debug_RXDATA_AUX_CTL_REG &= (uint8) ~debug_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (debug_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debug_DisableRxInt();

        debug_rxBufferRead = 0u;
        debug_rxBufferWrite = 0u;
        debug_rxBufferLoopDetect = 0u;
        debug_rxBufferOverflow = 0u;

        debug_EnableRxInt();

    #endif /* (debug_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: debug_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  debug__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  debug__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  debug__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  debug__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  debug__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  debug_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  debug_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void debug_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(debug_RXHW_ADDRESS_ENABLED)
            #if(debug_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* debug_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = debug_CONTROL_REG & (uint8)~debug_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << debug_CTRL_RXADDR_MODE0_SHIFT);
                debug_CONTROL_REG = tmpCtrl;

                #if(debug_RX_INTERRUPT_ENABLED && \
                   (debug_RXBUFFERSIZE > debug_FIFO_LENGTH) )
                    debug_rxAddressMode = addressMode;
                    debug_rxAddressDetected = 0u;
                #endif /* End debug_RXBUFFERSIZE > debug_FIFO_LENGTH*/
            #endif /* End debug_CONTROL_REG_REMOVED */
        #else /* debug_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End debug_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: debug_SetRxAddress1
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
    void debug_SetRxAddress1(uint8 address) 
    {
        debug_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: debug_SetRxAddress2
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
    void debug_SetRxAddress2(uint8 address) 
    {
        debug_RXADDRESS2_REG = address;
    }

#endif  /* debug_RX_ENABLED || debug_HD_ENABLED*/


#if( (debug_TX_ENABLED) || (debug_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: debug_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   debug_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   debug_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   debug_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   debug_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void debug_SetTxInterruptMode(uint8 intSrc) 
    {
        debug_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: debug_WriteTxData
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
    *  debug_txBuffer - RAM buffer pointer for save data for transmission
    *  debug_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  debug_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  debug_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void debug_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(debug_initVar != 0u)
        {
        #if (debug_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            debug_DisableTxInt();

            if( (debug_txBufferRead == debug_txBufferWrite) &&
                ((debug_TXSTATUS_REG & debug_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                debug_TXDATA_REG = txDataByte;
            }
            else
            {
                if(debug_txBufferWrite >= debug_TX_BUFFER_SIZE)
                {
                    debug_txBufferWrite = 0u;
                }

                debug_txBuffer[debug_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                debug_txBufferWrite++;
            }

            debug_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            debug_TXDATA_REG = txDataByte;

        #endif /*(debug_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: debug_ReadTxStatus
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
    uint8 debug_ReadTxStatus(void) 
    {
        return(debug_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: debug_PutChar
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
    *  debug_txBuffer - RAM buffer pointer for save data for transmission
    *  debug_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  debug_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  debug_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void debug_PutChar(uint8 txDataByte) 
    {
    #if (debug_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            debug_DisableTxInt();
        #endif /* (debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = debug_txBufferWrite;
            locTxBufferRead  = debug_txBufferRead;

        #if ((debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            debug_EnableTxInt();
        #endif /* (debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(debug_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((debug_TXSTATUS_REG & debug_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            debug_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= debug_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            debug_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3))
            debug_DisableTxInt();
        #endif /* (debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3) */

            debug_txBufferWrite = locTxBufferWrite;

        #if ((debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3))
            debug_EnableTxInt();
        #endif /* (debug_TX_BUFFER_SIZE > debug_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (debug_TXSTATUS_REG & debug_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                debug_SetPendingTxInt();
            }
        }

    #else

        while((debug_TXSTATUS_REG & debug_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        debug_TXDATA_REG = txDataByte;

    #endif /* debug_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: debug_PutString
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
    *  debug_initVar - checked to identify that the component has been
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
    void debug_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(debug_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                debug_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: debug_PutArray
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
    *  debug_initVar - checked to identify that the component has been
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
    void debug_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(debug_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                debug_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: debug_PutCRLF
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
    *  debug_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void debug_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(debug_initVar != 0u)
        {
            debug_PutChar(txDataByte);
            debug_PutChar(0x0Du);
            debug_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: debug_GetTxBufferSize
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
    *  debug_txBufferWrite - used to calculate left space.
    *  debug_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 debug_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (debug_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debug_DisableTxInt();

        if(debug_txBufferRead == debug_txBufferWrite)
        {
            size = 0u;
        }
        else if(debug_txBufferRead < debug_txBufferWrite)
        {
            size = (debug_txBufferWrite - debug_txBufferRead);
        }
        else
        {
            size = (debug_TX_BUFFER_SIZE - debug_txBufferRead) +
                    debug_txBufferWrite;
        }

        debug_EnableTxInt();

    #else

        size = debug_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & debug_TX_STS_FIFO_FULL) != 0u)
        {
            size = debug_FIFO_LENGTH;
        }
        else if((size & debug_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (debug_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: debug_ClearTxBuffer
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
    *  debug_txBufferWrite - cleared to zero.
    *  debug_txBufferRead - cleared to zero.
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
    void debug_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        debug_TXDATA_AUX_CTL_REG |= (uint8)  debug_TX_FIFO_CLR;
        debug_TXDATA_AUX_CTL_REG &= (uint8) ~debug_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (debug_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debug_DisableTxInt();

        debug_txBufferRead = 0u;
        debug_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        debug_EnableTxInt();

    #endif /* (debug_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: debug_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   debug_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   debug_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   debug_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   debug_SEND_WAIT_REINIT - Performs both options: 
    *      debug_SEND_BREAK and debug_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  debug_initVar - checked to identify that the component has been
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
    *     When interrupt appear with debug_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The debug_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void debug_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(debug_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(debug_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == debug_SEND_BREAK) ||
                (retMode == debug_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                debug_WriteControlRegister(debug_ReadControlRegister() |
                                                      debug_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                debug_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = debug_TXSTATUS_REG;
                }
                while((tmpStat & debug_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == debug_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debug_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = debug_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & debug_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == debug_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debug_REINIT) ||
                (retMode == debug_SEND_WAIT_REINIT) )
            {
                debug_WriteControlRegister(debug_ReadControlRegister() &
                                              (uint8)~debug_CTRL_HD_SEND_BREAK);
            }

        #else /* debug_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == debug_SEND_BREAK) ||
                (retMode == debug_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (debug_PARITY_TYPE != debug__B_UART__NONE_REVB) || \
                                    (debug_PARITY_TYPE_SW != 0u) )
                    debug_WriteControlRegister(debug_ReadControlRegister() |
                                                          debug_CTRL_HD_SEND_BREAK);
                #endif /* End debug_PARITY_TYPE != debug__B_UART__NONE_REVB  */

                #if(debug_TXCLKGEN_DP)
                    txPeriod = debug_TXBITCLKTX_COMPLETE_REG;
                    debug_TXBITCLKTX_COMPLETE_REG = debug_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = debug_TXBITCTR_PERIOD_REG;
                    debug_TXBITCTR_PERIOD_REG = debug_TXBITCTR_BREAKBITS8X;
                #endif /* End debug_TXCLKGEN_DP */

                /* Send zeros */
                debug_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = debug_TXSTATUS_REG;
                }
                while((tmpStat & debug_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == debug_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debug_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = debug_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & debug_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == debug_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debug_REINIT) ||
                (retMode == debug_SEND_WAIT_REINIT) )
            {

            #if(debug_TXCLKGEN_DP)
                debug_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                debug_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End debug_TXCLKGEN_DP */

            #if( (debug_PARITY_TYPE != debug__B_UART__NONE_REVB) || \
                 (debug_PARITY_TYPE_SW != 0u) )
                debug_WriteControlRegister(debug_ReadControlRegister() &
                                                      (uint8) ~debug_CTRL_HD_SEND_BREAK);
            #endif /* End debug_PARITY_TYPE != NONE */
            }
        #endif    /* End debug_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: debug_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       debug_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       debug_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears debug_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void debug_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( debug_CONTROL_REG_REMOVED == 0u )
            debug_WriteControlRegister(debug_ReadControlRegister() |
                                                  debug_CTRL_MARK);
        #endif /* End debug_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( debug_CONTROL_REG_REMOVED == 0u )
            debug_WriteControlRegister(debug_ReadControlRegister() &
                                                  (uint8) ~debug_CTRL_MARK);
        #endif /* End debug_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* Enddebug_TX_ENABLED */

#if(debug_HD_ENABLED)


    /*******************************************************************************
    * Function Name: debug_LoadRxConfig
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
    void debug_LoadRxConfig(void) 
    {
        debug_WriteControlRegister(debug_ReadControlRegister() &
                                                (uint8)~debug_CTRL_HD_SEND);
        debug_RXBITCTR_PERIOD_REG = debug_HD_RXBITCTR_INIT;

    #if (debug_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        debug_SetRxInterruptMode(debug_INIT_RX_INTERRUPTS_MASK);
    #endif /* (debug_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: debug_LoadTxConfig
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
    void debug_LoadTxConfig(void) 
    {
    #if (debug_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        debug_SetRxInterruptMode(0u);
    #endif /* (debug_RX_INTERRUPT_ENABLED) */

        debug_WriteControlRegister(debug_ReadControlRegister() | debug_CTRL_HD_SEND);
        debug_RXBITCTR_PERIOD_REG = debug_HD_TXBITCTR_INIT;
    }

#endif  /* debug_HD_ENABLED */


/* [] END OF FILE */
