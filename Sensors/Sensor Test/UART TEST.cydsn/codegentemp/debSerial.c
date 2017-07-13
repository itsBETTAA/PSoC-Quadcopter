/*******************************************************************************
* File Name: debSerial.c
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

#include "debSerial.h"
#if (debSerial_INTERNAL_CLOCK_USED)
    #include "debSerial_IntClock.h"
#endif /* End debSerial_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 debSerial_initVar = 0u;

#if (debSerial_TX_INTERRUPT_ENABLED && debSerial_TX_ENABLED)
    volatile uint8 debSerial_txBuffer[debSerial_TX_BUFFER_SIZE];
    volatile uint8 debSerial_txBufferRead = 0u;
    uint8 debSerial_txBufferWrite = 0u;
#endif /* (debSerial_TX_INTERRUPT_ENABLED && debSerial_TX_ENABLED) */

#if (debSerial_RX_INTERRUPT_ENABLED && (debSerial_RX_ENABLED || debSerial_HD_ENABLED))
    uint8 debSerial_errorStatus = 0u;
    volatile uint8 debSerial_rxBuffer[debSerial_RX_BUFFER_SIZE];
    volatile uint8 debSerial_rxBufferRead  = 0u;
    volatile uint8 debSerial_rxBufferWrite = 0u;
    volatile uint8 debSerial_rxBufferLoopDetect = 0u;
    volatile uint8 debSerial_rxBufferOverflow   = 0u;
    #if (debSerial_RXHW_ADDRESS_ENABLED)
        volatile uint8 debSerial_rxAddressMode = debSerial_RX_ADDRESS_MODE;
        volatile uint8 debSerial_rxAddressDetected = 0u;
    #endif /* (debSerial_RXHW_ADDRESS_ENABLED) */
#endif /* (debSerial_RX_INTERRUPT_ENABLED && (debSerial_RX_ENABLED || debSerial_HD_ENABLED)) */


/*******************************************************************************
* Function Name: debSerial_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  debSerial_Start() sets the initVar variable, calls the
*  debSerial_Init() function, and then calls the
*  debSerial_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The debSerial_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time debSerial_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the debSerial_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debSerial_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(debSerial_initVar == 0u)
    {
        debSerial_Init();
        debSerial_initVar = 1u;
    }

    debSerial_Enable();
}


/*******************************************************************************
* Function Name: debSerial_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call debSerial_Init() because
*  the debSerial_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void debSerial_Init(void) 
{
    #if(debSerial_RX_ENABLED || debSerial_HD_ENABLED)

        #if (debSerial_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(debSerial_RX_VECT_NUM, &debSerial_RXISR);
            CyIntSetPriority(debSerial_RX_VECT_NUM, debSerial_RX_PRIOR_NUM);
            debSerial_errorStatus = 0u;
        #endif /* (debSerial_RX_INTERRUPT_ENABLED) */

        #if (debSerial_RXHW_ADDRESS_ENABLED)
            debSerial_SetRxAddressMode(debSerial_RX_ADDRESS_MODE);
            debSerial_SetRxAddress1(debSerial_RX_HW_ADDRESS1);
            debSerial_SetRxAddress2(debSerial_RX_HW_ADDRESS2);
        #endif /* End debSerial_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        debSerial_RXBITCTR_PERIOD_REG = debSerial_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        debSerial_RXSTATUS_MASK_REG  = debSerial_INIT_RX_INTERRUPTS_MASK;
    #endif /* End debSerial_RX_ENABLED || debSerial_HD_ENABLED*/

    #if(debSerial_TX_ENABLED)
        #if (debSerial_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(debSerial_TX_VECT_NUM, &debSerial_TXISR);
            CyIntSetPriority(debSerial_TX_VECT_NUM, debSerial_TX_PRIOR_NUM);
        #endif /* (debSerial_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (debSerial_TXCLKGEN_DP)
            debSerial_TXBITCLKGEN_CTR_REG = debSerial_BIT_CENTER;
            debSerial_TXBITCLKTX_COMPLETE_REG = ((debSerial_NUMBER_OF_DATA_BITS +
                        debSerial_NUMBER_OF_START_BIT) * debSerial_OVER_SAMPLE_COUNT) - 1u;
        #else
            debSerial_TXBITCTR_PERIOD_REG = ((debSerial_NUMBER_OF_DATA_BITS +
                        debSerial_NUMBER_OF_START_BIT) * debSerial_OVER_SAMPLE_8) - 1u;
        #endif /* End debSerial_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (debSerial_TX_INTERRUPT_ENABLED)
            debSerial_TXSTATUS_MASK_REG = debSerial_TX_STS_FIFO_EMPTY;
        #else
            debSerial_TXSTATUS_MASK_REG = debSerial_INIT_TX_INTERRUPTS_MASK;
        #endif /*End debSerial_TX_INTERRUPT_ENABLED*/

    #endif /* End debSerial_TX_ENABLED */

    #if(debSerial_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        debSerial_WriteControlRegister( \
            (debSerial_ReadControlRegister() & (uint8)~debSerial_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(debSerial_PARITY_TYPE << debSerial_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End debSerial_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: debSerial_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call debSerial_Enable() because the debSerial_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debSerial_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void debSerial_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (debSerial_RX_ENABLED || debSerial_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        debSerial_RXBITCTR_CONTROL_REG |= debSerial_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        debSerial_RXSTATUS_ACTL_REG  |= debSerial_INT_ENABLE;

        #if (debSerial_RX_INTERRUPT_ENABLED)
            debSerial_EnableRxInt();

            #if (debSerial_RXHW_ADDRESS_ENABLED)
                debSerial_rxAddressDetected = 0u;
            #endif /* (debSerial_RXHW_ADDRESS_ENABLED) */
        #endif /* (debSerial_RX_INTERRUPT_ENABLED) */
    #endif /* (debSerial_RX_ENABLED || debSerial_HD_ENABLED) */

    #if(debSerial_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!debSerial_TXCLKGEN_DP)
            debSerial_TXBITCTR_CONTROL_REG |= debSerial_CNTR_ENABLE;
        #endif /* End debSerial_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        debSerial_TXSTATUS_ACTL_REG |= debSerial_INT_ENABLE;
        #if (debSerial_TX_INTERRUPT_ENABLED)
            debSerial_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            debSerial_EnableTxInt();
        #endif /* (debSerial_TX_INTERRUPT_ENABLED) */
     #endif /* (debSerial_TX_INTERRUPT_ENABLED) */

    #if (debSerial_INTERNAL_CLOCK_USED)
        debSerial_IntClock_Start();  /* Enable the clock */
    #endif /* (debSerial_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: debSerial_Stop
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
void debSerial_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (debSerial_RX_ENABLED || debSerial_HD_ENABLED)
        debSerial_RXBITCTR_CONTROL_REG &= (uint8) ~debSerial_CNTR_ENABLE;
    #endif /* (debSerial_RX_ENABLED || debSerial_HD_ENABLED) */

    #if (debSerial_TX_ENABLED)
        #if(!debSerial_TXCLKGEN_DP)
            debSerial_TXBITCTR_CONTROL_REG &= (uint8) ~debSerial_CNTR_ENABLE;
        #endif /* (!debSerial_TXCLKGEN_DP) */
    #endif /* (debSerial_TX_ENABLED) */

    #if (debSerial_INTERNAL_CLOCK_USED)
        debSerial_IntClock_Stop();   /* Disable the clock */
    #endif /* (debSerial_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (debSerial_RX_ENABLED || debSerial_HD_ENABLED)
        debSerial_RXSTATUS_ACTL_REG  &= (uint8) ~debSerial_INT_ENABLE;

        #if (debSerial_RX_INTERRUPT_ENABLED)
            debSerial_DisableRxInt();
        #endif /* (debSerial_RX_INTERRUPT_ENABLED) */
    #endif /* (debSerial_RX_ENABLED || debSerial_HD_ENABLED) */

    #if (debSerial_TX_ENABLED)
        debSerial_TXSTATUS_ACTL_REG &= (uint8) ~debSerial_INT_ENABLE;

        #if (debSerial_TX_INTERRUPT_ENABLED)
            debSerial_DisableTxInt();
        #endif /* (debSerial_TX_INTERRUPT_ENABLED) */
    #endif /* (debSerial_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: debSerial_ReadControlRegister
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
uint8 debSerial_ReadControlRegister(void) 
{
    #if (debSerial_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(debSerial_CONTROL_REG);
    #endif /* (debSerial_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: debSerial_WriteControlRegister
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
void  debSerial_WriteControlRegister(uint8 control) 
{
    #if (debSerial_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       debSerial_CONTROL_REG = control;
    #endif /* (debSerial_CONTROL_REG_REMOVED) */
}


#if(debSerial_RX_ENABLED || debSerial_HD_ENABLED)
    /*******************************************************************************
    * Function Name: debSerial_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      debSerial_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      debSerial_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      debSerial_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      debSerial_RX_STS_BREAK            Interrupt on break.
    *      debSerial_RX_STS_OVERRUN          Interrupt on overrun error.
    *      debSerial_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      debSerial_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void debSerial_SetRxInterruptMode(uint8 intSrc) 
    {
        debSerial_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: debSerial_ReadRxData
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
    *  debSerial_rxBuffer - RAM buffer pointer for save received data.
    *  debSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  debSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  debSerial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 debSerial_ReadRxData(void) 
    {
        uint8 rxData;

    #if (debSerial_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        debSerial_DisableRxInt();

        locRxBufferRead  = debSerial_rxBufferRead;
        locRxBufferWrite = debSerial_rxBufferWrite;

        if( (debSerial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = debSerial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= debSerial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            debSerial_rxBufferRead = locRxBufferRead;

            if(debSerial_rxBufferLoopDetect != 0u)
            {
                debSerial_rxBufferLoopDetect = 0u;
                #if ((debSerial_RX_INTERRUPT_ENABLED) && (debSerial_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( debSerial_HD_ENABLED )
                        if((debSerial_CONTROL_REG & debSerial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            debSerial_RXSTATUS_MASK_REG  |= debSerial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        debSerial_RXSTATUS_MASK_REG  |= debSerial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end debSerial_HD_ENABLED */
                #endif /* ((debSerial_RX_INTERRUPT_ENABLED) && (debSerial_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = debSerial_RXDATA_REG;
        }

        debSerial_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = debSerial_RXDATA_REG;

    #endif /* (debSerial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: debSerial_ReadRxStatus
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
    *  debSerial_RX_STS_FIFO_NOTEMPTY.
    *  debSerial_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  debSerial_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   debSerial_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   debSerial_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 debSerial_ReadRxStatus(void) 
    {
        uint8 status;

        status = debSerial_RXSTATUS_REG & debSerial_RX_HW_MASK;

    #if (debSerial_RX_INTERRUPT_ENABLED)
        if(debSerial_rxBufferOverflow != 0u)
        {
            status |= debSerial_RX_STS_SOFT_BUFF_OVER;
            debSerial_rxBufferOverflow = 0u;
        }
    #endif /* (debSerial_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: debSerial_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. debSerial_GetChar() is
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
    *  debSerial_rxBuffer - RAM buffer pointer for save received data.
    *  debSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  debSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  debSerial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 debSerial_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (debSerial_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        debSerial_DisableRxInt();

        locRxBufferRead  = debSerial_rxBufferRead;
        locRxBufferWrite = debSerial_rxBufferWrite;

        if( (debSerial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = debSerial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= debSerial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            debSerial_rxBufferRead = locRxBufferRead;

            if(debSerial_rxBufferLoopDetect != 0u)
            {
                debSerial_rxBufferLoopDetect = 0u;
                #if( (debSerial_RX_INTERRUPT_ENABLED) && (debSerial_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( debSerial_HD_ENABLED )
                        if((debSerial_CONTROL_REG & debSerial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            debSerial_RXSTATUS_MASK_REG |= debSerial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        debSerial_RXSTATUS_MASK_REG |= debSerial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end debSerial_HD_ENABLED */
                #endif /* debSerial_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = debSerial_RXSTATUS_REG;
            if((rxStatus & debSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = debSerial_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (debSerial_RX_STS_BREAK | debSerial_RX_STS_PAR_ERROR |
                                debSerial_RX_STS_STOP_ERROR | debSerial_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        debSerial_EnableRxInt();

    #else

        rxStatus =debSerial_RXSTATUS_REG;
        if((rxStatus & debSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = debSerial_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (debSerial_RX_STS_BREAK | debSerial_RX_STS_PAR_ERROR |
                            debSerial_RX_STS_STOP_ERROR | debSerial_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (debSerial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: debSerial_GetByte
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
    uint16 debSerial_GetByte(void) 
    {
        
    #if (debSerial_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        debSerial_DisableRxInt();
        locErrorStatus = (uint16)debSerial_errorStatus;
        debSerial_errorStatus = 0u;
        debSerial_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | debSerial_ReadRxData() );
    #else
        return ( ((uint16)debSerial_ReadRxStatus() << 8u) | debSerial_ReadRxData() );
    #endif /* debSerial_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: debSerial_GetRxBufferSize
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
    *  debSerial_rxBufferWrite - used to calculate left bytes.
    *  debSerial_rxBufferRead - used to calculate left bytes.
    *  debSerial_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 debSerial_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (debSerial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        debSerial_DisableRxInt();

        if(debSerial_rxBufferRead == debSerial_rxBufferWrite)
        {
            if(debSerial_rxBufferLoopDetect != 0u)
            {
                size = debSerial_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(debSerial_rxBufferRead < debSerial_rxBufferWrite)
        {
            size = (debSerial_rxBufferWrite - debSerial_rxBufferRead);
        }
        else
        {
            size = (debSerial_RX_BUFFER_SIZE - debSerial_rxBufferRead) + debSerial_rxBufferWrite;
        }

        debSerial_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((debSerial_RXSTATUS_REG & debSerial_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (debSerial_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: debSerial_ClearRxBuffer
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
    *  debSerial_rxBufferWrite - cleared to zero.
    *  debSerial_rxBufferRead - cleared to zero.
    *  debSerial_rxBufferLoopDetect - cleared to zero.
    *  debSerial_rxBufferOverflow - cleared to zero.
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
    void debSerial_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        debSerial_RXDATA_AUX_CTL_REG |= (uint8)  debSerial_RX_FIFO_CLR;
        debSerial_RXDATA_AUX_CTL_REG &= (uint8) ~debSerial_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (debSerial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debSerial_DisableRxInt();

        debSerial_rxBufferRead = 0u;
        debSerial_rxBufferWrite = 0u;
        debSerial_rxBufferLoopDetect = 0u;
        debSerial_rxBufferOverflow = 0u;

        debSerial_EnableRxInt();

    #endif /* (debSerial_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: debSerial_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  debSerial__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  debSerial__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  debSerial__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  debSerial__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  debSerial__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  debSerial_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  debSerial_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void debSerial_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(debSerial_RXHW_ADDRESS_ENABLED)
            #if(debSerial_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* debSerial_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = debSerial_CONTROL_REG & (uint8)~debSerial_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << debSerial_CTRL_RXADDR_MODE0_SHIFT);
                debSerial_CONTROL_REG = tmpCtrl;

                #if(debSerial_RX_INTERRUPT_ENABLED && \
                   (debSerial_RXBUFFERSIZE > debSerial_FIFO_LENGTH) )
                    debSerial_rxAddressMode = addressMode;
                    debSerial_rxAddressDetected = 0u;
                #endif /* End debSerial_RXBUFFERSIZE > debSerial_FIFO_LENGTH*/
            #endif /* End debSerial_CONTROL_REG_REMOVED */
        #else /* debSerial_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End debSerial_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: debSerial_SetRxAddress1
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
    void debSerial_SetRxAddress1(uint8 address) 
    {
        debSerial_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: debSerial_SetRxAddress2
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
    void debSerial_SetRxAddress2(uint8 address) 
    {
        debSerial_RXADDRESS2_REG = address;
    }

#endif  /* debSerial_RX_ENABLED || debSerial_HD_ENABLED*/


#if( (debSerial_TX_ENABLED) || (debSerial_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: debSerial_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   debSerial_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   debSerial_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   debSerial_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   debSerial_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void debSerial_SetTxInterruptMode(uint8 intSrc) 
    {
        debSerial_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: debSerial_WriteTxData
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
    *  debSerial_txBuffer - RAM buffer pointer for save data for transmission
    *  debSerial_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  debSerial_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  debSerial_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void debSerial_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(debSerial_initVar != 0u)
        {
        #if (debSerial_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            debSerial_DisableTxInt();

            if( (debSerial_txBufferRead == debSerial_txBufferWrite) &&
                ((debSerial_TXSTATUS_REG & debSerial_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                debSerial_TXDATA_REG = txDataByte;
            }
            else
            {
                if(debSerial_txBufferWrite >= debSerial_TX_BUFFER_SIZE)
                {
                    debSerial_txBufferWrite = 0u;
                }

                debSerial_txBuffer[debSerial_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                debSerial_txBufferWrite++;
            }

            debSerial_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            debSerial_TXDATA_REG = txDataByte;

        #endif /*(debSerial_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: debSerial_ReadTxStatus
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
    uint8 debSerial_ReadTxStatus(void) 
    {
        return(debSerial_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: debSerial_PutChar
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
    *  debSerial_txBuffer - RAM buffer pointer for save data for transmission
    *  debSerial_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  debSerial_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  debSerial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void debSerial_PutChar(uint8 txDataByte) 
    {
    #if (debSerial_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            debSerial_DisableTxInt();
        #endif /* (debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = debSerial_txBufferWrite;
            locTxBufferRead  = debSerial_txBufferRead;

        #if ((debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            debSerial_EnableTxInt();
        #endif /* (debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(debSerial_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((debSerial_TXSTATUS_REG & debSerial_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            debSerial_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= debSerial_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            debSerial_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            debSerial_DisableTxInt();
        #endif /* (debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            debSerial_txBufferWrite = locTxBufferWrite;

        #if ((debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            debSerial_EnableTxInt();
        #endif /* (debSerial_TX_BUFFER_SIZE > debSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (debSerial_TXSTATUS_REG & debSerial_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                debSerial_SetPendingTxInt();
            }
        }

    #else

        while((debSerial_TXSTATUS_REG & debSerial_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        debSerial_TXDATA_REG = txDataByte;

    #endif /* debSerial_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: debSerial_PutString
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
    *  debSerial_initVar - checked to identify that the component has been
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
    void debSerial_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(debSerial_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                debSerial_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: debSerial_PutArray
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
    *  debSerial_initVar - checked to identify that the component has been
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
    void debSerial_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(debSerial_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                debSerial_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: debSerial_PutCRLF
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
    *  debSerial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void debSerial_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(debSerial_initVar != 0u)
        {
            debSerial_PutChar(txDataByte);
            debSerial_PutChar(0x0Du);
            debSerial_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: debSerial_GetTxBufferSize
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
    *  debSerial_txBufferWrite - used to calculate left space.
    *  debSerial_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 debSerial_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (debSerial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debSerial_DisableTxInt();

        if(debSerial_txBufferRead == debSerial_txBufferWrite)
        {
            size = 0u;
        }
        else if(debSerial_txBufferRead < debSerial_txBufferWrite)
        {
            size = (debSerial_txBufferWrite - debSerial_txBufferRead);
        }
        else
        {
            size = (debSerial_TX_BUFFER_SIZE - debSerial_txBufferRead) +
                    debSerial_txBufferWrite;
        }

        debSerial_EnableTxInt();

    #else

        size = debSerial_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & debSerial_TX_STS_FIFO_FULL) != 0u)
        {
            size = debSerial_FIFO_LENGTH;
        }
        else if((size & debSerial_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (debSerial_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: debSerial_ClearTxBuffer
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
    *  debSerial_txBufferWrite - cleared to zero.
    *  debSerial_txBufferRead - cleared to zero.
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
    void debSerial_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        debSerial_TXDATA_AUX_CTL_REG |= (uint8)  debSerial_TX_FIFO_CLR;
        debSerial_TXDATA_AUX_CTL_REG &= (uint8) ~debSerial_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (debSerial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        debSerial_DisableTxInt();

        debSerial_txBufferRead = 0u;
        debSerial_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        debSerial_EnableTxInt();

    #endif /* (debSerial_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: debSerial_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   debSerial_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   debSerial_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   debSerial_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   debSerial_SEND_WAIT_REINIT - Performs both options: 
    *      debSerial_SEND_BREAK and debSerial_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  debSerial_initVar - checked to identify that the component has been
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
    *     When interrupt appear with debSerial_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The debSerial_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void debSerial_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(debSerial_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(debSerial_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == debSerial_SEND_BREAK) ||
                (retMode == debSerial_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                debSerial_WriteControlRegister(debSerial_ReadControlRegister() |
                                                      debSerial_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                debSerial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = debSerial_TXSTATUS_REG;
                }
                while((tmpStat & debSerial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == debSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debSerial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = debSerial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & debSerial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == debSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debSerial_REINIT) ||
                (retMode == debSerial_SEND_WAIT_REINIT) )
            {
                debSerial_WriteControlRegister(debSerial_ReadControlRegister() &
                                              (uint8)~debSerial_CTRL_HD_SEND_BREAK);
            }

        #else /* debSerial_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == debSerial_SEND_BREAK) ||
                (retMode == debSerial_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (debSerial_PARITY_TYPE != debSerial__B_UART__NONE_REVB) || \
                                    (debSerial_PARITY_TYPE_SW != 0u) )
                    debSerial_WriteControlRegister(debSerial_ReadControlRegister() |
                                                          debSerial_CTRL_HD_SEND_BREAK);
                #endif /* End debSerial_PARITY_TYPE != debSerial__B_UART__NONE_REVB  */

                #if(debSerial_TXCLKGEN_DP)
                    txPeriod = debSerial_TXBITCLKTX_COMPLETE_REG;
                    debSerial_TXBITCLKTX_COMPLETE_REG = debSerial_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = debSerial_TXBITCTR_PERIOD_REG;
                    debSerial_TXBITCTR_PERIOD_REG = debSerial_TXBITCTR_BREAKBITS8X;
                #endif /* End debSerial_TXCLKGEN_DP */

                /* Send zeros */
                debSerial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = debSerial_TXSTATUS_REG;
                }
                while((tmpStat & debSerial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == debSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debSerial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = debSerial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & debSerial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == debSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == debSerial_REINIT) ||
                (retMode == debSerial_SEND_WAIT_REINIT) )
            {

            #if(debSerial_TXCLKGEN_DP)
                debSerial_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                debSerial_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End debSerial_TXCLKGEN_DP */

            #if( (debSerial_PARITY_TYPE != debSerial__B_UART__NONE_REVB) || \
                 (debSerial_PARITY_TYPE_SW != 0u) )
                debSerial_WriteControlRegister(debSerial_ReadControlRegister() &
                                                      (uint8) ~debSerial_CTRL_HD_SEND_BREAK);
            #endif /* End debSerial_PARITY_TYPE != NONE */
            }
        #endif    /* End debSerial_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: debSerial_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       debSerial_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       debSerial_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears debSerial_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void debSerial_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( debSerial_CONTROL_REG_REMOVED == 0u )
            debSerial_WriteControlRegister(debSerial_ReadControlRegister() |
                                                  debSerial_CTRL_MARK);
        #endif /* End debSerial_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( debSerial_CONTROL_REG_REMOVED == 0u )
            debSerial_WriteControlRegister(debSerial_ReadControlRegister() &
                                                  (uint8) ~debSerial_CTRL_MARK);
        #endif /* End debSerial_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EnddebSerial_TX_ENABLED */

#if(debSerial_HD_ENABLED)


    /*******************************************************************************
    * Function Name: debSerial_LoadRxConfig
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
    void debSerial_LoadRxConfig(void) 
    {
        debSerial_WriteControlRegister(debSerial_ReadControlRegister() &
                                                (uint8)~debSerial_CTRL_HD_SEND);
        debSerial_RXBITCTR_PERIOD_REG = debSerial_HD_RXBITCTR_INIT;

    #if (debSerial_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        debSerial_SetRxInterruptMode(debSerial_INIT_RX_INTERRUPTS_MASK);
    #endif /* (debSerial_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: debSerial_LoadTxConfig
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
    void debSerial_LoadTxConfig(void) 
    {
    #if (debSerial_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        debSerial_SetRxInterruptMode(0u);
    #endif /* (debSerial_RX_INTERRUPT_ENABLED) */

        debSerial_WriteControlRegister(debSerial_ReadControlRegister() | debSerial_CTRL_HD_SEND);
        debSerial_RXBITCTR_PERIOD_REG = debSerial_HD_TXBITCTR_INIT;
    }

#endif  /* debSerial_HD_ENABLED */


/* [] END OF FILE */
