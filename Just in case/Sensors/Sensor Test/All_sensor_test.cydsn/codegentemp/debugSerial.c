/*******************************************************************************
* File Name: debugSerial.c
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

#include "debugSerial.h"
#if (debugSerial_INTERNAL_CLOCK_USED)
    #include "debugSerial_IntClock.h"
#endif /* End debugSerial_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 debugSerial_initVar = 0u;

#if (debugSerial_TX_INTERRUPT_ENABLED && debugSerial_TX_ENABLED)
    volatile uint8 debugSerial_txBuffer[debugSerial_TX_BUFFER_SIZE];
    volatile uint8 debugSerial_txBufferRead = 0u;
    uint8 debugSerial_txBufferWrite = 0u;
#endif /* (debugSerial_TX_INTERRUPT_ENABLED && debugSerial_TX_ENABLED) */

#if (debugSerial_RX_INTERRUPT_ENABLED && (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED))
    uint8 debugSerial_errorStatus = 0u;
    volatile uint8 debugSerial_rxBuffer[debugSerial_RX_BUFFER_SIZE];
    volatile uint8 debugSerial_rxBufferRead  = 0u;
    volatile uint8 debugSerial_rxBufferWrite = 0u;
    volatile uint8 debugSerial_rxBufferLoopDetect = 0u;
    volatile uint8 debugSerial_rxBufferOverflow   = 0u;
    #if (debugSerial_RXHW_ADDRESS_ENABLED)
        volatile uint8 debugSerial_rxAddressMode = debugSerial_RX_ADDRESS_MODE;
        volatile uint8 debugSerial_rxAddressDetected = 0u;
    #endif /* (debugSerial_RXHW_ADDRESS_ENABLED) */
#endif /* (debugSerial_RX_INTERRUPT_ENABLED && (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)) */


/*******************************************************************************
* Function Name: debugSerial_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  debugSerial_Start() sets the initVar variable, calls the
*  debugSerial_Init() function, and then calls the
*  debugSerial_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The debugSerial_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time debugSerial_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the debugSerial_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debugSerial_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(debugSerial_initVar == 0u)
    {
        debugSerial_Init();
        debugSerial_initVar = 1u;
    }

    debugSerial_Enable();
}


/*******************************************************************************
* Function Name: debugSerial_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call debugSerial_Init() because
*  the debugSerial_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void debugSerial_Init(void) 
{
    #if(debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)

        #if (debugSerial_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(debugSerial_RX_VECT_NUM, &debugSerial_RXISR);
            CyIntSetPriority(debugSerial_RX_VECT_NUM, debugSerial_RX_PRIOR_NUM);
            debugSerial_errorStatus = 0u;
        #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */

        #if (debugSerial_RXHW_ADDRESS_ENABLED)
            debugSerial_SetRxAddressMode(debugSerial_RX_ADDRESS_MODE);
            debugSerial_SetRxAddress1(debugSerial_RX_HW_ADDRESS1);
            debugSerial_SetRxAddress2(debugSerial_RX_HW_ADDRESS2);
        #endif /* End debugSerial_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        debugSerial_RXBITCTR_PERIOD_REG = debugSerial_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        debugSerial_RXSTATUS_MASK_REG  = debugSerial_INIT_RX_INTERRUPTS_MASK;
    #endif /* End debugSerial_RX_ENABLED || debugSerial_HD_ENABLED*/

    #if(debugSerial_TX_ENABLED)
        #if (debugSerial_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(debugSerial_TX_VECT_NUM, &debugSerial_TXISR);
            CyIntSetPriority(debugSerial_TX_VECT_NUM, debugSerial_TX_PRIOR_NUM);
        #endif /* (debugSerial_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (debugSerial_TXCLKGEN_DP)
            debugSerial_TXBITCLKGEN_CTR_REG = debugSerial_BIT_CENTER;
            debugSerial_TXBITCLKTX_COMPLETE_REG = ((debugSerial_NUMBER_OF_DATA_BITS +
                        debugSerial_NUMBER_OF_START_BIT) * debugSerial_OVER_SAMPLE_COUNT) - 1u;
        #else
            debugSerial_TXBITCTR_PERIOD_REG = ((debugSerial_NUMBER_OF_DATA_BITS +
                        debugSerial_NUMBER_OF_START_BIT) * debugSerial_OVER_SAMPLE_8) - 1u;
        #endif /* End debugSerial_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (debugSerial_TX_INTERRUPT_ENABLED)
            debugSerial_TXSTATUS_MASK_REG = debugSerial_TX_STS_FIFO_EMPTY;
        #else
            debugSerial_TXSTATUS_MASK_REG = debugSerial_INIT_TX_INTERRUPTS_MASK;
        #endif /*End debugSerial_TX_INTERRUPT_ENABLED*/

    #endif /* End debugSerial_TX_ENABLED */

    #if(debugSerial_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        debugSerial_WriteControlRegister( \
            (debugSerial_ReadControlRegister() & (uint8)~debugSerial_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(debugSerial_PARITY_TYPE << debugSerial_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End debugSerial_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: debugSerial_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call debugSerial_Enable() because the debugSerial_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debugSerial_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void debugSerial_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        debugSerial_RXBITCTR_CONTROL_REG |= debugSerial_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        debugSerial_RXSTATUS_ACTL_REG  |= debugSerial_INT_ENABLE;

        #if (debugSerial_RX_INTERRUPT_ENABLED)
            debugSerial_EnableRxInt();

            #if (debugSerial_RXHW_ADDRESS_ENABLED)
                debugSerial_rxAddressDetected = 0u;
            #endif /* (debugSerial_RXHW_ADDRESS_ENABLED) */
        #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */
    #endif /* (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED) */

    #if(debugSerial_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!debugSerial_TXCLKGEN_DP)
            debugSerial_TXBITCTR_CONTROL_REG |= debugSerial_CNTR_ENABLE;
        #endif /* End debugSerial_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        debugSerial_TXSTATUS_ACTL_REG |= debugSerial_INT_ENABLE;
        #if (debugSerial_TX_INTERRUPT_ENABLED)
            debugSerial_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            debugSerial_EnableTxInt();
        #endif /* (debugSerial_TX_INTERRUPT_ENABLED) */
     #endif /* (debugSerial_TX_INTERRUPT_ENABLED) */

    #if (debugSerial_INTERNAL_CLOCK_USED)
        debugSerial_IntClock_Start();  /* Enable the clock */
    #endif /* (debugSerial_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: debugSerial_Stop
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
void debugSerial_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)
        debugSerial_RXBITCTR_CONTROL_REG &= (uint8) ~debugSerial_CNTR_ENABLE;
    #endif /* (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED) */

    #if (debugSerial_TX_ENABLED)
        #if(!debugSerial_TXCLKGEN_DP)
            debugSerial_TXBITCTR_CONTROL_REG &= (uint8) ~debugSerial_CNTR_ENABLE;
        #endif /* (!debugSerial_TXCLKGEN_DP) */
    #endif /* (debugSerial_TX_ENABLED) */

    #if (debugSerial_INTERNAL_CLOCK_USED)
        debugSerial_IntClock_Stop();   /* Disable the clock */
    #endif /* (debugSerial_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)
        debugSerial_RXSTATUS_ACTL_REG  &= (uint8) ~debugSerial_INT_ENABLE;

        #if (debugSerial_RX_INTERRUPT_ENABLED)
            debugSerial_DisableRxInt();
        #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */
    #endif /* (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED) */

    #if (debugSerial_TX_ENABLED)
        debugSerial_TXSTATUS_ACTL_REG &= (uint8) ~debugSerial_INT_ENABLE;

        #if (debugSerial_TX_INTERRUPT_ENABLED)
            debugSerial_DisableTxInt();
        #endif /* (debugSerial_TX_INTERRUPT_ENABLED) */
    #endif /* (debugSerial_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: debugSerial_ReadControlRegister
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
uint8 debugSerial_ReadControlRegister(void) 
{
    #if (debugSerial_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(debugSerial_CONTROL_REG);
    #endif /* (debugSerial_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: debugSerial_WriteControlRegister
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
void  debugSerial_WriteControlRegister(uint8 control) 
{
    #if (debugSerial_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       debugSerial_CONTROL_REG = control;
    #endif /* (debugSerial_CONTROL_REG_REMOVED) */
}


#if(debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)
    /*******************************************************************************
    * Function Name: debugSerial_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      debugSerial_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      debugSerial_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      debugSerial_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      debugSerial_RX_STS_BREAK            Interrupt on break.
    *      debugSerial_RX_STS_OVERRUN          Interrupt on overrun error.
    *      debugSerial_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      debugSerial_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void debugSerial_SetRxInterruptMode(uint8 intSrc) 
    {
        debugSerial_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: debugSerial_ReadRxData
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
    *  debugSerial_rxBuffer - RAM buffer pointer for save received data.
    *  debugSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  debugSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  debugSerial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 debugSerial_ReadRxData(void) 
    {
        uint8 rxData;

    #if (debugSerial_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        debugSerial_DisableRxInt();

        locRxBufferRead  = debugSerial_rxBufferRead;
        locRxBufferWrite = debugSerial_rxBufferWrite;

        if( (debugSerial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = debugSerial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= debugSerial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            debugSerial_rxBufferRead = locRxBufferRead;

            if(debugSerial_rxBufferLoopDetect != 0u)
            {
                debugSerial_rxBufferLoopDetect = 0u;
                #if ((debugSerial_RX_INTERRUPT_ENABLED) && (debugSerial_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( debugSerial_HD_ENABLED )
                        if((debugSerial_CONTROL_REG & debugSerial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            debugSerial_RXSTATUS_MASK_REG  |= debugSerial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        debugSerial_RXSTATUS_MASK_REG  |= debugSerial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end debugSerial_HD_ENABLED */
                #endif /* ((debugSerial_RX_INTERRUPT_ENABLED) && (debugSerial_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = debugSerial_RXDATA_REG;
        }

        debugSerial_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = debugSerial_RXDATA_REG;

    #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: debugSerial_ReadRxStatus
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
    *  debugSerial_RX_STS_FIFO_NOTEMPTY.
    *  debugSerial_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  debugSerial_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   debugSerial_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   debugSerial_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 debugSerial_ReadRxStatus(void) 
    {
        uint8 status;

        status = debugSerial_RXSTATUS_REG & debugSerial_RX_HW_MASK;

    #if (debugSerial_RX_INTERRUPT_ENABLED)
        if(debugSerial_rxBufferOverflow != 0u)
        {
            status |= debugSerial_RX_STS_SOFT_BUFF_OVER;
            debugSerial_rxBufferOverflow = 0u;
        }
    #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: debugSerial_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. debugSerial_GetChar() is
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
    *  debugSerial_rxBuffer - RAM buffer pointer for save received data.
    *  debugSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  debugSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  debugSerial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 debugSerial_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (debugSerial_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        debugSerial_DisableRxInt();

        locRxBufferRead  = debugSerial_rxBufferRead;
        locRxBufferWrite = debugSerial_rxBufferWrite;

        if( (debugSerial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = debugSerial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= debugSerial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            debugSerial_rxBufferRead = locRxBufferRead;

            if(debugSerial_rxBufferLoopDetect != 0u)
            {
                debugSerial_rxBufferLoopDetect = 0u;
                #if( (debugSerial_RX_INTERRUPT_ENABLED) && (debugSerial_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( debugSerial_HD_ENABLED )
                        if((debugSerial_CONTROL_REG & debugSerial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            debugSerial_RXSTATUS_MASK_REG |= debugSerial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        debugSerial_RXSTATUS_MASK_REG |= debugSerial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end debugSerial_HD_ENABLED */
                #endif /* debugSerial_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = debugSerial_RXSTATUS_REG;
            if((rxStatus & debugSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = debugSerial_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (debugSerial_RX_STS_BREAK | debugSerial_RX_STS_PAR_ERROR |
                                debugSerial_RX_STS_STOP_ERROR | debugSerial_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        debugSerial_EnableRxInt();

    #else

        rxStatus =debugSerial_RXSTATUS_REG;
        if((rxStatus & debugSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = debugSerial_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (debugSerial_RX_STS_BREAK | debugSerial_RX_STS_PAR_ERROR |
                            debugSerial_RX_STS_STOP_ERROR | debugSerial_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: debugSerial_GetByte
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
    uint16 debugSerial_GetByte(void) 
    {
        
    #if (debugSerial_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        debugSerial_DisableRxInt();
        locErrorStatus = (uint16)debugSerial_errorStatus;
        debugSerial_errorStatus = 0u;
        debugSerial_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | debugSerial_ReadRxData() );
    #else
        return ( ((uint16)debugSerial_ReadRxStatus() << 8u) | debugSerial_ReadRxData() );
    #endif /* debugSerial_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: debugSerial_GetRxBufferSize
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
    *  debugSerial_rxBufferWrite - used to calculate left bytes.
    *  debugSerial_rxBufferRead - used to calculate left bytes.
    *  debugSerial_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 debugSerial_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (debugSerial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        debugSerial_DisableRxInt();

        if(debugSerial_rxBufferRead == debugSerial_rxBufferWrite)
        {
            if(debugSerial_rxBufferLoopDetect != 0u)
            {
                size = debugSerial_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(debugSerial_rxBufferRead < debugSerial_rxBufferWrite)
        {
            size = (debugSerial_rxBufferWrite - debugSerial_rxBufferRead);
        }
        else
        {
            size = (debugSerial_RX_BUFFER_SIZE - debugSerial_rxBufferRead) + debugSerial_rxBufferWrite;
        }

        debugSerial_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((debugSerial_RXSTATUS_REG & debugSerial_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: debugSerial_ClearRxBuffer
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
    *  debugSerial_rxBufferWrite - cleared to zero.
    *  debugSerial_rxBufferRead - cleared to zero.
    *  debugSerial_rxBufferLoopDetect - cleared to zero.
    *  debugSerial_rxBufferOverflow - cleared to zero.
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
    void debugSerial_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        debugSerial_RXDATA_AUX_CTL_REG |= (uint8)  debugSerial_RX_FIFO_CLR;
        debugSerial_RXDATA_AUX_CTL_REG &= (uint8) ~debugSerial_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (debugSerial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debugSerial_DisableRxInt();

        debugSerial_rxBufferRead = 0u;
        debugSerial_rxBufferWrite = 0u;
        debugSerial_rxBufferLoopDetect = 0u;
        debugSerial_rxBufferOverflow = 0u;

        debugSerial_EnableRxInt();

    #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: debugSerial_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  debugSerial__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  debugSerial__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  debugSerial__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  debugSerial__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  debugSerial__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  debugSerial_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  debugSerial_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void debugSerial_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(debugSerial_RXHW_ADDRESS_ENABLED)
            #if(debugSerial_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* debugSerial_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = debugSerial_CONTROL_REG & (uint8)~debugSerial_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << debugSerial_CTRL_RXADDR_MODE0_SHIFT);
                debugSerial_CONTROL_REG = tmpCtrl;

                #if(debugSerial_RX_INTERRUPT_ENABLED && \
                   (debugSerial_RXBUFFERSIZE > debugSerial_FIFO_LENGTH) )
                    debugSerial_rxAddressMode = addressMode;
                    debugSerial_rxAddressDetected = 0u;
                #endif /* End debugSerial_RXBUFFERSIZE > debugSerial_FIFO_LENGTH*/
            #endif /* End debugSerial_CONTROL_REG_REMOVED */
        #else /* debugSerial_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End debugSerial_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: debugSerial_SetRxAddress1
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
    void debugSerial_SetRxAddress1(uint8 address) 
    {
        debugSerial_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: debugSerial_SetRxAddress2
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
    void debugSerial_SetRxAddress2(uint8 address) 
    {
        debugSerial_RXADDRESS2_REG = address;
    }

#endif  /* debugSerial_RX_ENABLED || debugSerial_HD_ENABLED*/


#if( (debugSerial_TX_ENABLED) || (debugSerial_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: debugSerial_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   debugSerial_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   debugSerial_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   debugSerial_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   debugSerial_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void debugSerial_SetTxInterruptMode(uint8 intSrc) 
    {
        debugSerial_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: debugSerial_WriteTxData
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
    *  debugSerial_txBuffer - RAM buffer pointer for save data for transmission
    *  debugSerial_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  debugSerial_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  debugSerial_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void debugSerial_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(debugSerial_initVar != 0u)
        {
        #if (debugSerial_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            debugSerial_DisableTxInt();

            if( (debugSerial_txBufferRead == debugSerial_txBufferWrite) &&
                ((debugSerial_TXSTATUS_REG & debugSerial_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                debugSerial_TXDATA_REG = txDataByte;
            }
            else
            {
                if(debugSerial_txBufferWrite >= debugSerial_TX_BUFFER_SIZE)
                {
                    debugSerial_txBufferWrite = 0u;
                }

                debugSerial_txBuffer[debugSerial_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                debugSerial_txBufferWrite++;
            }

            debugSerial_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            debugSerial_TXDATA_REG = txDataByte;

        #endif /*(debugSerial_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: debugSerial_ReadTxStatus
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
    uint8 debugSerial_ReadTxStatus(void) 
    {
        return(debugSerial_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: debugSerial_PutChar
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
    *  debugSerial_txBuffer - RAM buffer pointer for save data for transmission
    *  debugSerial_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  debugSerial_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  debugSerial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void debugSerial_PutChar(uint8 txDataByte) 
    {
    #if (debugSerial_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            debugSerial_DisableTxInt();
        #endif /* (debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = debugSerial_txBufferWrite;
            locTxBufferRead  = debugSerial_txBufferRead;

        #if ((debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            debugSerial_EnableTxInt();
        #endif /* (debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(debugSerial_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((debugSerial_TXSTATUS_REG & debugSerial_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            debugSerial_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= debugSerial_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            debugSerial_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            debugSerial_DisableTxInt();
        #endif /* (debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            debugSerial_txBufferWrite = locTxBufferWrite;

        #if ((debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            debugSerial_EnableTxInt();
        #endif /* (debugSerial_TX_BUFFER_SIZE > debugSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (debugSerial_TXSTATUS_REG & debugSerial_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                debugSerial_SetPendingTxInt();
            }
        }

    #else

        while((debugSerial_TXSTATUS_REG & debugSerial_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        debugSerial_TXDATA_REG = txDataByte;

    #endif /* debugSerial_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: debugSerial_PutString
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
    *  debugSerial_initVar - checked to identify that the component has been
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
    void debugSerial_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(debugSerial_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                debugSerial_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: debugSerial_PutArray
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
    *  debugSerial_initVar - checked to identify that the component has been
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
    void debugSerial_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(debugSerial_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                debugSerial_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: debugSerial_PutCRLF
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
    *  debugSerial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void debugSerial_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(debugSerial_initVar != 0u)
        {
            debugSerial_PutChar(txDataByte);
            debugSerial_PutChar(0x0Du);
            debugSerial_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: debugSerial_GetTxBufferSize
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
    *  debugSerial_txBufferWrite - used to calculate left space.
    *  debugSerial_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 debugSerial_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (debugSerial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debugSerial_DisableTxInt();

        if(debugSerial_txBufferRead == debugSerial_txBufferWrite)
        {
            size = 0u;
        }
        else if(debugSerial_txBufferRead < debugSerial_txBufferWrite)
        {
            size = (debugSerial_txBufferWrite - debugSerial_txBufferRead);
        }
        else
        {
            size = (debugSerial_TX_BUFFER_SIZE - debugSerial_txBufferRead) +
                    debugSerial_txBufferWrite;
        }

        debugSerial_EnableTxInt();

    #else

        size = debugSerial_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & debugSerial_TX_STS_FIFO_FULL) != 0u)
        {
            size = debugSerial_FIFO_LENGTH;
        }
        else if((size & debugSerial_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (debugSerial_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: debugSerial_ClearTxBuffer
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
    *  debugSerial_txBufferWrite - cleared to zero.
    *  debugSerial_txBufferRead - cleared to zero.
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
    void debugSerial_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        debugSerial_TXDATA_AUX_CTL_REG |= (uint8)  debugSerial_TX_FIFO_CLR;
        debugSerial_TXDATA_AUX_CTL_REG &= (uint8) ~debugSerial_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (debugSerial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debugSerial_DisableTxInt();

        debugSerial_txBufferRead = 0u;
        debugSerial_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        debugSerial_EnableTxInt();

    #endif /* (debugSerial_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: debugSerial_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   debugSerial_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   debugSerial_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   debugSerial_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   debugSerial_SEND_WAIT_REINIT - Performs both options: 
    *      debugSerial_SEND_BREAK and debugSerial_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  debugSerial_initVar - checked to identify that the component has been
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
    *     When interrupt appear with debugSerial_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The debugSerial_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void debugSerial_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(debugSerial_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(debugSerial_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == debugSerial_SEND_BREAK) ||
                (retMode == debugSerial_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() |
                                                      debugSerial_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                debugSerial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = debugSerial_TXSTATUS_REG;
                }
                while((tmpStat & debugSerial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == debugSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debugSerial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = debugSerial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & debugSerial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == debugSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debugSerial_REINIT) ||
                (retMode == debugSerial_SEND_WAIT_REINIT) )
            {
                debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() &
                                              (uint8)~debugSerial_CTRL_HD_SEND_BREAK);
            }

        #else /* debugSerial_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == debugSerial_SEND_BREAK) ||
                (retMode == debugSerial_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (debugSerial_PARITY_TYPE != debugSerial__B_UART__NONE_REVB) || \
                                    (debugSerial_PARITY_TYPE_SW != 0u) )
                    debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() |
                                                          debugSerial_CTRL_HD_SEND_BREAK);
                #endif /* End debugSerial_PARITY_TYPE != debugSerial__B_UART__NONE_REVB  */

                #if(debugSerial_TXCLKGEN_DP)
                    txPeriod = debugSerial_TXBITCLKTX_COMPLETE_REG;
                    debugSerial_TXBITCLKTX_COMPLETE_REG = debugSerial_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = debugSerial_TXBITCTR_PERIOD_REG;
                    debugSerial_TXBITCTR_PERIOD_REG = debugSerial_TXBITCTR_BREAKBITS8X;
                #endif /* End debugSerial_TXCLKGEN_DP */

                /* Send zeros */
                debugSerial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = debugSerial_TXSTATUS_REG;
                }
                while((tmpStat & debugSerial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == debugSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debugSerial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = debugSerial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & debugSerial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == debugSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debugSerial_REINIT) ||
                (retMode == debugSerial_SEND_WAIT_REINIT) )
            {

            #if(debugSerial_TXCLKGEN_DP)
                debugSerial_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                debugSerial_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End debugSerial_TXCLKGEN_DP */

            #if( (debugSerial_PARITY_TYPE != debugSerial__B_UART__NONE_REVB) || \
                 (debugSerial_PARITY_TYPE_SW != 0u) )
                debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() &
                                                      (uint8) ~debugSerial_CTRL_HD_SEND_BREAK);
            #endif /* End debugSerial_PARITY_TYPE != NONE */
            }
        #endif    /* End debugSerial_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: debugSerial_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       debugSerial_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       debugSerial_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears debugSerial_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void debugSerial_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( debugSerial_CONTROL_REG_REMOVED == 0u )
            debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() |
                                                  debugSerial_CTRL_MARK);
        #endif /* End debugSerial_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( debugSerial_CONTROL_REG_REMOVED == 0u )
            debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() &
                                                  (uint8) ~debugSerial_CTRL_MARK);
        #endif /* End debugSerial_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EnddebugSerial_TX_ENABLED */

#if(debugSerial_HD_ENABLED)


    /*******************************************************************************
    * Function Name: debugSerial_LoadRxConfig
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
    void debugSerial_LoadRxConfig(void) 
    {
        debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() &
                                                (uint8)~debugSerial_CTRL_HD_SEND);
        debugSerial_RXBITCTR_PERIOD_REG = debugSerial_HD_RXBITCTR_INIT;

    #if (debugSerial_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        debugSerial_SetRxInterruptMode(debugSerial_INIT_RX_INTERRUPTS_MASK);
    #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: debugSerial_LoadTxConfig
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
    void debugSerial_LoadTxConfig(void) 
    {
    #if (debugSerial_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        debugSerial_SetRxInterruptMode(0u);
    #endif /* (debugSerial_RX_INTERRUPT_ENABLED) */

        debugSerial_WriteControlRegister(debugSerial_ReadControlRegister() | debugSerial_CTRL_HD_SEND);
        debugSerial_RXBITCTR_PERIOD_REG = debugSerial_HD_TXBITCTR_INIT;
    }

#endif  /* debugSerial_HD_ENABLED */


/* [] END OF FILE */
