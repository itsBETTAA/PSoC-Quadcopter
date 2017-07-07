/*******************************************************************************
* File Name: killSwitch.c
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

#include "killSwitch.h"
#if (killSwitch_INTERNAL_CLOCK_USED)
    #include "killSwitch_IntClock.h"
#endif /* End killSwitch_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 killSwitch_initVar = 0u;

#if (killSwitch_TX_INTERRUPT_ENABLED && killSwitch_TX_ENABLED)
    volatile uint8 killSwitch_txBuffer[killSwitch_TX_BUFFER_SIZE];
    volatile uint8 killSwitch_txBufferRead = 0u;
    uint8 killSwitch_txBufferWrite = 0u;
#endif /* (killSwitch_TX_INTERRUPT_ENABLED && killSwitch_TX_ENABLED) */

#if (killSwitch_RX_INTERRUPT_ENABLED && (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED))
    uint8 killSwitch_errorStatus = 0u;
    volatile uint8 killSwitch_rxBuffer[killSwitch_RX_BUFFER_SIZE];
    volatile uint8 killSwitch_rxBufferRead  = 0u;
    volatile uint8 killSwitch_rxBufferWrite = 0u;
    volatile uint8 killSwitch_rxBufferLoopDetect = 0u;
    volatile uint8 killSwitch_rxBufferOverflow   = 0u;
    #if (killSwitch_RXHW_ADDRESS_ENABLED)
        volatile uint8 killSwitch_rxAddressMode = killSwitch_RX_ADDRESS_MODE;
        volatile uint8 killSwitch_rxAddressDetected = 0u;
    #endif /* (killSwitch_RXHW_ADDRESS_ENABLED) */
#endif /* (killSwitch_RX_INTERRUPT_ENABLED && (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)) */


/*******************************************************************************
* Function Name: killSwitch_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  killSwitch_Start() sets the initVar variable, calls the
*  killSwitch_Init() function, and then calls the
*  killSwitch_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The killSwitch_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time killSwitch_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the killSwitch_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void killSwitch_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(killSwitch_initVar == 0u)
    {
        killSwitch_Init();
        killSwitch_initVar = 1u;
    }

    killSwitch_Enable();
}


/*******************************************************************************
* Function Name: killSwitch_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call killSwitch_Init() because
*  the killSwitch_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void killSwitch_Init(void) 
{
    #if(killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)

        #if (killSwitch_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(killSwitch_RX_VECT_NUM, &killSwitch_RXISR);
            CyIntSetPriority(killSwitch_RX_VECT_NUM, killSwitch_RX_PRIOR_NUM);
            killSwitch_errorStatus = 0u;
        #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */

        #if (killSwitch_RXHW_ADDRESS_ENABLED)
            killSwitch_SetRxAddressMode(killSwitch_RX_ADDRESS_MODE);
            killSwitch_SetRxAddress1(killSwitch_RX_HW_ADDRESS1);
            killSwitch_SetRxAddress2(killSwitch_RX_HW_ADDRESS2);
        #endif /* End killSwitch_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        killSwitch_RXBITCTR_PERIOD_REG = killSwitch_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        killSwitch_RXSTATUS_MASK_REG  = killSwitch_INIT_RX_INTERRUPTS_MASK;
    #endif /* End killSwitch_RX_ENABLED || killSwitch_HD_ENABLED*/

    #if(killSwitch_TX_ENABLED)
        #if (killSwitch_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(killSwitch_TX_VECT_NUM, &killSwitch_TXISR);
            CyIntSetPriority(killSwitch_TX_VECT_NUM, killSwitch_TX_PRIOR_NUM);
        #endif /* (killSwitch_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (killSwitch_TXCLKGEN_DP)
            killSwitch_TXBITCLKGEN_CTR_REG = killSwitch_BIT_CENTER;
            killSwitch_TXBITCLKTX_COMPLETE_REG = ((killSwitch_NUMBER_OF_DATA_BITS +
                        killSwitch_NUMBER_OF_START_BIT) * killSwitch_OVER_SAMPLE_COUNT) - 1u;
        #else
            killSwitch_TXBITCTR_PERIOD_REG = ((killSwitch_NUMBER_OF_DATA_BITS +
                        killSwitch_NUMBER_OF_START_BIT) * killSwitch_OVER_SAMPLE_8) - 1u;
        #endif /* End killSwitch_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (killSwitch_TX_INTERRUPT_ENABLED)
            killSwitch_TXSTATUS_MASK_REG = killSwitch_TX_STS_FIFO_EMPTY;
        #else
            killSwitch_TXSTATUS_MASK_REG = killSwitch_INIT_TX_INTERRUPTS_MASK;
        #endif /*End killSwitch_TX_INTERRUPT_ENABLED*/

    #endif /* End killSwitch_TX_ENABLED */

    #if(killSwitch_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        killSwitch_WriteControlRegister( \
            (killSwitch_ReadControlRegister() & (uint8)~killSwitch_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(killSwitch_PARITY_TYPE << killSwitch_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End killSwitch_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: killSwitch_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call killSwitch_Enable() because the killSwitch_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  killSwitch_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void killSwitch_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        killSwitch_RXBITCTR_CONTROL_REG |= killSwitch_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        killSwitch_RXSTATUS_ACTL_REG  |= killSwitch_INT_ENABLE;

        #if (killSwitch_RX_INTERRUPT_ENABLED)
            killSwitch_EnableRxInt();

            #if (killSwitch_RXHW_ADDRESS_ENABLED)
                killSwitch_rxAddressDetected = 0u;
            #endif /* (killSwitch_RXHW_ADDRESS_ENABLED) */
        #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */
    #endif /* (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED) */

    #if(killSwitch_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!killSwitch_TXCLKGEN_DP)
            killSwitch_TXBITCTR_CONTROL_REG |= killSwitch_CNTR_ENABLE;
        #endif /* End killSwitch_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        killSwitch_TXSTATUS_ACTL_REG |= killSwitch_INT_ENABLE;
        #if (killSwitch_TX_INTERRUPT_ENABLED)
            killSwitch_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            killSwitch_EnableTxInt();
        #endif /* (killSwitch_TX_INTERRUPT_ENABLED) */
     #endif /* (killSwitch_TX_INTERRUPT_ENABLED) */

    #if (killSwitch_INTERNAL_CLOCK_USED)
        killSwitch_IntClock_Start();  /* Enable the clock */
    #endif /* (killSwitch_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: killSwitch_Stop
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
void killSwitch_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)
        killSwitch_RXBITCTR_CONTROL_REG &= (uint8) ~killSwitch_CNTR_ENABLE;
    #endif /* (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED) */

    #if (killSwitch_TX_ENABLED)
        #if(!killSwitch_TXCLKGEN_DP)
            killSwitch_TXBITCTR_CONTROL_REG &= (uint8) ~killSwitch_CNTR_ENABLE;
        #endif /* (!killSwitch_TXCLKGEN_DP) */
    #endif /* (killSwitch_TX_ENABLED) */

    #if (killSwitch_INTERNAL_CLOCK_USED)
        killSwitch_IntClock_Stop();   /* Disable the clock */
    #endif /* (killSwitch_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)
        killSwitch_RXSTATUS_ACTL_REG  &= (uint8) ~killSwitch_INT_ENABLE;

        #if (killSwitch_RX_INTERRUPT_ENABLED)
            killSwitch_DisableRxInt();
        #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */
    #endif /* (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED) */

    #if (killSwitch_TX_ENABLED)
        killSwitch_TXSTATUS_ACTL_REG &= (uint8) ~killSwitch_INT_ENABLE;

        #if (killSwitch_TX_INTERRUPT_ENABLED)
            killSwitch_DisableTxInt();
        #endif /* (killSwitch_TX_INTERRUPT_ENABLED) */
    #endif /* (killSwitch_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: killSwitch_ReadControlRegister
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
uint8 killSwitch_ReadControlRegister(void) 
{
    #if (killSwitch_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(killSwitch_CONTROL_REG);
    #endif /* (killSwitch_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: killSwitch_WriteControlRegister
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
void  killSwitch_WriteControlRegister(uint8 control) 
{
    #if (killSwitch_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       killSwitch_CONTROL_REG = control;
    #endif /* (killSwitch_CONTROL_REG_REMOVED) */
}


#if(killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)
    /*******************************************************************************
    * Function Name: killSwitch_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      killSwitch_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      killSwitch_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      killSwitch_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      killSwitch_RX_STS_BREAK            Interrupt on break.
    *      killSwitch_RX_STS_OVERRUN          Interrupt on overrun error.
    *      killSwitch_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      killSwitch_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void killSwitch_SetRxInterruptMode(uint8 intSrc) 
    {
        killSwitch_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: killSwitch_ReadRxData
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
    *  killSwitch_rxBuffer - RAM buffer pointer for save received data.
    *  killSwitch_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  killSwitch_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  killSwitch_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 killSwitch_ReadRxData(void) 
    {
        uint8 rxData;

    #if (killSwitch_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        killSwitch_DisableRxInt();

        locRxBufferRead  = killSwitch_rxBufferRead;
        locRxBufferWrite = killSwitch_rxBufferWrite;

        if( (killSwitch_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = killSwitch_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= killSwitch_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            killSwitch_rxBufferRead = locRxBufferRead;

            if(killSwitch_rxBufferLoopDetect != 0u)
            {
                killSwitch_rxBufferLoopDetect = 0u;
                #if ((killSwitch_RX_INTERRUPT_ENABLED) && (killSwitch_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( killSwitch_HD_ENABLED )
                        if((killSwitch_CONTROL_REG & killSwitch_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            killSwitch_RXSTATUS_MASK_REG  |= killSwitch_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        killSwitch_RXSTATUS_MASK_REG  |= killSwitch_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end killSwitch_HD_ENABLED */
                #endif /* ((killSwitch_RX_INTERRUPT_ENABLED) && (killSwitch_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = killSwitch_RXDATA_REG;
        }

        killSwitch_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = killSwitch_RXDATA_REG;

    #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: killSwitch_ReadRxStatus
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
    *  killSwitch_RX_STS_FIFO_NOTEMPTY.
    *  killSwitch_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  killSwitch_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   killSwitch_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   killSwitch_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 killSwitch_ReadRxStatus(void) 
    {
        uint8 status;

        status = killSwitch_RXSTATUS_REG & killSwitch_RX_HW_MASK;

    #if (killSwitch_RX_INTERRUPT_ENABLED)
        if(killSwitch_rxBufferOverflow != 0u)
        {
            status |= killSwitch_RX_STS_SOFT_BUFF_OVER;
            killSwitch_rxBufferOverflow = 0u;
        }
    #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: killSwitch_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. killSwitch_GetChar() is
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
    *  killSwitch_rxBuffer - RAM buffer pointer for save received data.
    *  killSwitch_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  killSwitch_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  killSwitch_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 killSwitch_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (killSwitch_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        killSwitch_DisableRxInt();

        locRxBufferRead  = killSwitch_rxBufferRead;
        locRxBufferWrite = killSwitch_rxBufferWrite;

        if( (killSwitch_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = killSwitch_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= killSwitch_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            killSwitch_rxBufferRead = locRxBufferRead;

            if(killSwitch_rxBufferLoopDetect != 0u)
            {
                killSwitch_rxBufferLoopDetect = 0u;
                #if( (killSwitch_RX_INTERRUPT_ENABLED) && (killSwitch_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( killSwitch_HD_ENABLED )
                        if((killSwitch_CONTROL_REG & killSwitch_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            killSwitch_RXSTATUS_MASK_REG |= killSwitch_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        killSwitch_RXSTATUS_MASK_REG |= killSwitch_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end killSwitch_HD_ENABLED */
                #endif /* killSwitch_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = killSwitch_RXSTATUS_REG;
            if((rxStatus & killSwitch_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = killSwitch_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (killSwitch_RX_STS_BREAK | killSwitch_RX_STS_PAR_ERROR |
                                killSwitch_RX_STS_STOP_ERROR | killSwitch_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        killSwitch_EnableRxInt();

    #else

        rxStatus =killSwitch_RXSTATUS_REG;
        if((rxStatus & killSwitch_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = killSwitch_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (killSwitch_RX_STS_BREAK | killSwitch_RX_STS_PAR_ERROR |
                            killSwitch_RX_STS_STOP_ERROR | killSwitch_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: killSwitch_GetByte
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
    uint16 killSwitch_GetByte(void) 
    {
        
    #if (killSwitch_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        killSwitch_DisableRxInt();
        locErrorStatus = (uint16)killSwitch_errorStatus;
        killSwitch_errorStatus = 0u;
        killSwitch_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | killSwitch_ReadRxData() );
    #else
        return ( ((uint16)killSwitch_ReadRxStatus() << 8u) | killSwitch_ReadRxData() );
    #endif /* killSwitch_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: killSwitch_GetRxBufferSize
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
    *  killSwitch_rxBufferWrite - used to calculate left bytes.
    *  killSwitch_rxBufferRead - used to calculate left bytes.
    *  killSwitch_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 killSwitch_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (killSwitch_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        killSwitch_DisableRxInt();

        if(killSwitch_rxBufferRead == killSwitch_rxBufferWrite)
        {
            if(killSwitch_rxBufferLoopDetect != 0u)
            {
                size = killSwitch_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(killSwitch_rxBufferRead < killSwitch_rxBufferWrite)
        {
            size = (killSwitch_rxBufferWrite - killSwitch_rxBufferRead);
        }
        else
        {
            size = (killSwitch_RX_BUFFER_SIZE - killSwitch_rxBufferRead) + killSwitch_rxBufferWrite;
        }

        killSwitch_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((killSwitch_RXSTATUS_REG & killSwitch_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: killSwitch_ClearRxBuffer
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
    *  killSwitch_rxBufferWrite - cleared to zero.
    *  killSwitch_rxBufferRead - cleared to zero.
    *  killSwitch_rxBufferLoopDetect - cleared to zero.
    *  killSwitch_rxBufferOverflow - cleared to zero.
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
    void killSwitch_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        killSwitch_RXDATA_AUX_CTL_REG |= (uint8)  killSwitch_RX_FIFO_CLR;
        killSwitch_RXDATA_AUX_CTL_REG &= (uint8) ~killSwitch_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (killSwitch_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        killSwitch_DisableRxInt();

        killSwitch_rxBufferRead = 0u;
        killSwitch_rxBufferWrite = 0u;
        killSwitch_rxBufferLoopDetect = 0u;
        killSwitch_rxBufferOverflow = 0u;

        killSwitch_EnableRxInt();

    #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: killSwitch_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  killSwitch__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  killSwitch__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  killSwitch__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  killSwitch__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  killSwitch__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  killSwitch_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  killSwitch_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void killSwitch_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(killSwitch_RXHW_ADDRESS_ENABLED)
            #if(killSwitch_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* killSwitch_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = killSwitch_CONTROL_REG & (uint8)~killSwitch_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << killSwitch_CTRL_RXADDR_MODE0_SHIFT);
                killSwitch_CONTROL_REG = tmpCtrl;

                #if(killSwitch_RX_INTERRUPT_ENABLED && \
                   (killSwitch_RXBUFFERSIZE > killSwitch_FIFO_LENGTH) )
                    killSwitch_rxAddressMode = addressMode;
                    killSwitch_rxAddressDetected = 0u;
                #endif /* End killSwitch_RXBUFFERSIZE > killSwitch_FIFO_LENGTH*/
            #endif /* End killSwitch_CONTROL_REG_REMOVED */
        #else /* killSwitch_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End killSwitch_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: killSwitch_SetRxAddress1
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
    void killSwitch_SetRxAddress1(uint8 address) 
    {
        killSwitch_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: killSwitch_SetRxAddress2
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
    void killSwitch_SetRxAddress2(uint8 address) 
    {
        killSwitch_RXADDRESS2_REG = address;
    }

#endif  /* killSwitch_RX_ENABLED || killSwitch_HD_ENABLED*/


#if( (killSwitch_TX_ENABLED) || (killSwitch_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: killSwitch_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   killSwitch_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   killSwitch_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   killSwitch_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   killSwitch_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void killSwitch_SetTxInterruptMode(uint8 intSrc) 
    {
        killSwitch_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: killSwitch_WriteTxData
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
    *  killSwitch_txBuffer - RAM buffer pointer for save data for transmission
    *  killSwitch_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  killSwitch_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  killSwitch_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void killSwitch_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(killSwitch_initVar != 0u)
        {
        #if (killSwitch_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            killSwitch_DisableTxInt();

            if( (killSwitch_txBufferRead == killSwitch_txBufferWrite) &&
                ((killSwitch_TXSTATUS_REG & killSwitch_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                killSwitch_TXDATA_REG = txDataByte;
            }
            else
            {
                if(killSwitch_txBufferWrite >= killSwitch_TX_BUFFER_SIZE)
                {
                    killSwitch_txBufferWrite = 0u;
                }

                killSwitch_txBuffer[killSwitch_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                killSwitch_txBufferWrite++;
            }

            killSwitch_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            killSwitch_TXDATA_REG = txDataByte;

        #endif /*(killSwitch_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: killSwitch_ReadTxStatus
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
    uint8 killSwitch_ReadTxStatus(void) 
    {
        return(killSwitch_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: killSwitch_PutChar
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
    *  killSwitch_txBuffer - RAM buffer pointer for save data for transmission
    *  killSwitch_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  killSwitch_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  killSwitch_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void killSwitch_PutChar(uint8 txDataByte) 
    {
    #if (killSwitch_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            killSwitch_DisableTxInt();
        #endif /* (killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = killSwitch_txBufferWrite;
            locTxBufferRead  = killSwitch_txBufferRead;

        #if ((killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            killSwitch_EnableTxInt();
        #endif /* (killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(killSwitch_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((killSwitch_TXSTATUS_REG & killSwitch_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            killSwitch_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= killSwitch_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            killSwitch_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3))
            killSwitch_DisableTxInt();
        #endif /* (killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3) */

            killSwitch_txBufferWrite = locTxBufferWrite;

        #if ((killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3))
            killSwitch_EnableTxInt();
        #endif /* (killSwitch_TX_BUFFER_SIZE > killSwitch_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (killSwitch_TXSTATUS_REG & killSwitch_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                killSwitch_SetPendingTxInt();
            }
        }

    #else

        while((killSwitch_TXSTATUS_REG & killSwitch_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        killSwitch_TXDATA_REG = txDataByte;

    #endif /* killSwitch_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: killSwitch_PutString
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
    *  killSwitch_initVar - checked to identify that the component has been
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
    void killSwitch_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(killSwitch_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                killSwitch_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: killSwitch_PutArray
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
    *  killSwitch_initVar - checked to identify that the component has been
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
    void killSwitch_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(killSwitch_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                killSwitch_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: killSwitch_PutCRLF
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
    *  killSwitch_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void killSwitch_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(killSwitch_initVar != 0u)
        {
            killSwitch_PutChar(txDataByte);
            killSwitch_PutChar(0x0Du);
            killSwitch_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: killSwitch_GetTxBufferSize
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
    *  killSwitch_txBufferWrite - used to calculate left space.
    *  killSwitch_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 killSwitch_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (killSwitch_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        killSwitch_DisableTxInt();

        if(killSwitch_txBufferRead == killSwitch_txBufferWrite)
        {
            size = 0u;
        }
        else if(killSwitch_txBufferRead < killSwitch_txBufferWrite)
        {
            size = (killSwitch_txBufferWrite - killSwitch_txBufferRead);
        }
        else
        {
            size = (killSwitch_TX_BUFFER_SIZE - killSwitch_txBufferRead) +
                    killSwitch_txBufferWrite;
        }

        killSwitch_EnableTxInt();

    #else

        size = killSwitch_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & killSwitch_TX_STS_FIFO_FULL) != 0u)
        {
            size = killSwitch_FIFO_LENGTH;
        }
        else if((size & killSwitch_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (killSwitch_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: killSwitch_ClearTxBuffer
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
    *  killSwitch_txBufferWrite - cleared to zero.
    *  killSwitch_txBufferRead - cleared to zero.
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
    void killSwitch_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        killSwitch_TXDATA_AUX_CTL_REG |= (uint8)  killSwitch_TX_FIFO_CLR;
        killSwitch_TXDATA_AUX_CTL_REG &= (uint8) ~killSwitch_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (killSwitch_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        killSwitch_DisableTxInt();

        killSwitch_txBufferRead = 0u;
        killSwitch_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        killSwitch_EnableTxInt();

    #endif /* (killSwitch_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: killSwitch_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   killSwitch_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   killSwitch_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   killSwitch_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   killSwitch_SEND_WAIT_REINIT - Performs both options: 
    *      killSwitch_SEND_BREAK and killSwitch_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  killSwitch_initVar - checked to identify that the component has been
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
    *     When interrupt appear with killSwitch_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The killSwitch_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void killSwitch_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(killSwitch_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(killSwitch_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == killSwitch_SEND_BREAK) ||
                (retMode == killSwitch_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() |
                                                      killSwitch_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                killSwitch_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = killSwitch_TXSTATUS_REG;
                }
                while((tmpStat & killSwitch_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == killSwitch_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == killSwitch_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = killSwitch_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & killSwitch_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == killSwitch_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == killSwitch_REINIT) ||
                (retMode == killSwitch_SEND_WAIT_REINIT) )
            {
                killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() &
                                              (uint8)~killSwitch_CTRL_HD_SEND_BREAK);
            }

        #else /* killSwitch_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == killSwitch_SEND_BREAK) ||
                (retMode == killSwitch_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (killSwitch_PARITY_TYPE != killSwitch__B_UART__NONE_REVB) || \
                                    (killSwitch_PARITY_TYPE_SW != 0u) )
                    killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() |
                                                          killSwitch_CTRL_HD_SEND_BREAK);
                #endif /* End killSwitch_PARITY_TYPE != killSwitch__B_UART__NONE_REVB  */

                #if(killSwitch_TXCLKGEN_DP)
                    txPeriod = killSwitch_TXBITCLKTX_COMPLETE_REG;
                    killSwitch_TXBITCLKTX_COMPLETE_REG = killSwitch_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = killSwitch_TXBITCTR_PERIOD_REG;
                    killSwitch_TXBITCTR_PERIOD_REG = killSwitch_TXBITCTR_BREAKBITS8X;
                #endif /* End killSwitch_TXCLKGEN_DP */

                /* Send zeros */
                killSwitch_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = killSwitch_TXSTATUS_REG;
                }
                while((tmpStat & killSwitch_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == killSwitch_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == killSwitch_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = killSwitch_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & killSwitch_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == killSwitch_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == killSwitch_REINIT) ||
                (retMode == killSwitch_SEND_WAIT_REINIT) )
            {

            #if(killSwitch_TXCLKGEN_DP)
                killSwitch_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                killSwitch_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End killSwitch_TXCLKGEN_DP */

            #if( (killSwitch_PARITY_TYPE != killSwitch__B_UART__NONE_REVB) || \
                 (killSwitch_PARITY_TYPE_SW != 0u) )
                killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() &
                                                      (uint8) ~killSwitch_CTRL_HD_SEND_BREAK);
            #endif /* End killSwitch_PARITY_TYPE != NONE */
            }
        #endif    /* End killSwitch_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: killSwitch_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       killSwitch_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       killSwitch_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears killSwitch_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void killSwitch_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( killSwitch_CONTROL_REG_REMOVED == 0u )
            killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() |
                                                  killSwitch_CTRL_MARK);
        #endif /* End killSwitch_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( killSwitch_CONTROL_REG_REMOVED == 0u )
            killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() &
                                                  (uint8) ~killSwitch_CTRL_MARK);
        #endif /* End killSwitch_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndkillSwitch_TX_ENABLED */

#if(killSwitch_HD_ENABLED)


    /*******************************************************************************
    * Function Name: killSwitch_LoadRxConfig
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
    void killSwitch_LoadRxConfig(void) 
    {
        killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() &
                                                (uint8)~killSwitch_CTRL_HD_SEND);
        killSwitch_RXBITCTR_PERIOD_REG = killSwitch_HD_RXBITCTR_INIT;

    #if (killSwitch_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        killSwitch_SetRxInterruptMode(killSwitch_INIT_RX_INTERRUPTS_MASK);
    #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: killSwitch_LoadTxConfig
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
    void killSwitch_LoadTxConfig(void) 
    {
    #if (killSwitch_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        killSwitch_SetRxInterruptMode(0u);
    #endif /* (killSwitch_RX_INTERRUPT_ENABLED) */

        killSwitch_WriteControlRegister(killSwitch_ReadControlRegister() | killSwitch_CTRL_HD_SEND);
        killSwitch_RXBITCTR_PERIOD_REG = killSwitch_HD_TXBITCTR_INIT;
    }

#endif  /* killSwitch_HD_ENABLED */


/* [] END OF FILE */
