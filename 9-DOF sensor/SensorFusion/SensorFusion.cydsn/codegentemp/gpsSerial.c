/*******************************************************************************
* File Name: gpsSerial.c
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

#include "gpsSerial.h"
#if (gpsSerial_INTERNAL_CLOCK_USED)
    #include "gpsSerial_IntClock.h"
#endif /* End gpsSerial_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 gpsSerial_initVar = 0u;

#if (gpsSerial_TX_INTERRUPT_ENABLED && gpsSerial_TX_ENABLED)
    volatile uint8 gpsSerial_txBuffer[gpsSerial_TX_BUFFER_SIZE];
    volatile uint8 gpsSerial_txBufferRead = 0u;
    uint8 gpsSerial_txBufferWrite = 0u;
#endif /* (gpsSerial_TX_INTERRUPT_ENABLED && gpsSerial_TX_ENABLED) */

#if (gpsSerial_RX_INTERRUPT_ENABLED && (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED))
    uint8 gpsSerial_errorStatus = 0u;
    volatile uint8 gpsSerial_rxBuffer[gpsSerial_RX_BUFFER_SIZE];
    volatile uint8 gpsSerial_rxBufferRead  = 0u;
    volatile uint8 gpsSerial_rxBufferWrite = 0u;
    volatile uint8 gpsSerial_rxBufferLoopDetect = 0u;
    volatile uint8 gpsSerial_rxBufferOverflow   = 0u;
    #if (gpsSerial_RXHW_ADDRESS_ENABLED)
        volatile uint8 gpsSerial_rxAddressMode = gpsSerial_RX_ADDRESS_MODE;
        volatile uint8 gpsSerial_rxAddressDetected = 0u;
    #endif /* (gpsSerial_RXHW_ADDRESS_ENABLED) */
#endif /* (gpsSerial_RX_INTERRUPT_ENABLED && (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)) */


/*******************************************************************************
* Function Name: gpsSerial_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  gpsSerial_Start() sets the initVar variable, calls the
*  gpsSerial_Init() function, and then calls the
*  gpsSerial_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The gpsSerial_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time gpsSerial_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the gpsSerial_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void gpsSerial_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(gpsSerial_initVar == 0u)
    {
        gpsSerial_Init();
        gpsSerial_initVar = 1u;
    }

    gpsSerial_Enable();
}


/*******************************************************************************
* Function Name: gpsSerial_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call gpsSerial_Init() because
*  the gpsSerial_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void gpsSerial_Init(void) 
{
    #if(gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)

        #if (gpsSerial_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(gpsSerial_RX_VECT_NUM, &gpsSerial_RXISR);
            CyIntSetPriority(gpsSerial_RX_VECT_NUM, gpsSerial_RX_PRIOR_NUM);
            gpsSerial_errorStatus = 0u;
        #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */

        #if (gpsSerial_RXHW_ADDRESS_ENABLED)
            gpsSerial_SetRxAddressMode(gpsSerial_RX_ADDRESS_MODE);
            gpsSerial_SetRxAddress1(gpsSerial_RX_HW_ADDRESS1);
            gpsSerial_SetRxAddress2(gpsSerial_RX_HW_ADDRESS2);
        #endif /* End gpsSerial_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        gpsSerial_RXBITCTR_PERIOD_REG = gpsSerial_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        gpsSerial_RXSTATUS_MASK_REG  = gpsSerial_INIT_RX_INTERRUPTS_MASK;
    #endif /* End gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED*/

    #if(gpsSerial_TX_ENABLED)
        #if (gpsSerial_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(gpsSerial_TX_VECT_NUM, &gpsSerial_TXISR);
            CyIntSetPriority(gpsSerial_TX_VECT_NUM, gpsSerial_TX_PRIOR_NUM);
        #endif /* (gpsSerial_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (gpsSerial_TXCLKGEN_DP)
            gpsSerial_TXBITCLKGEN_CTR_REG = gpsSerial_BIT_CENTER;
            gpsSerial_TXBITCLKTX_COMPLETE_REG = ((gpsSerial_NUMBER_OF_DATA_BITS +
                        gpsSerial_NUMBER_OF_START_BIT) * gpsSerial_OVER_SAMPLE_COUNT) - 1u;
        #else
            gpsSerial_TXBITCTR_PERIOD_REG = ((gpsSerial_NUMBER_OF_DATA_BITS +
                        gpsSerial_NUMBER_OF_START_BIT) * gpsSerial_OVER_SAMPLE_8) - 1u;
        #endif /* End gpsSerial_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (gpsSerial_TX_INTERRUPT_ENABLED)
            gpsSerial_TXSTATUS_MASK_REG = gpsSerial_TX_STS_FIFO_EMPTY;
        #else
            gpsSerial_TXSTATUS_MASK_REG = gpsSerial_INIT_TX_INTERRUPTS_MASK;
        #endif /*End gpsSerial_TX_INTERRUPT_ENABLED*/

    #endif /* End gpsSerial_TX_ENABLED */

    #if(gpsSerial_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        gpsSerial_WriteControlRegister( \
            (gpsSerial_ReadControlRegister() & (uint8)~gpsSerial_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(gpsSerial_PARITY_TYPE << gpsSerial_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End gpsSerial_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: gpsSerial_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call gpsSerial_Enable() because the gpsSerial_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  gpsSerial_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void gpsSerial_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        gpsSerial_RXBITCTR_CONTROL_REG |= gpsSerial_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        gpsSerial_RXSTATUS_ACTL_REG  |= gpsSerial_INT_ENABLE;

        #if (gpsSerial_RX_INTERRUPT_ENABLED)
            gpsSerial_EnableRxInt();

            #if (gpsSerial_RXHW_ADDRESS_ENABLED)
                gpsSerial_rxAddressDetected = 0u;
            #endif /* (gpsSerial_RXHW_ADDRESS_ENABLED) */
        #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */
    #endif /* (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED) */

    #if(gpsSerial_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!gpsSerial_TXCLKGEN_DP)
            gpsSerial_TXBITCTR_CONTROL_REG |= gpsSerial_CNTR_ENABLE;
        #endif /* End gpsSerial_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        gpsSerial_TXSTATUS_ACTL_REG |= gpsSerial_INT_ENABLE;
        #if (gpsSerial_TX_INTERRUPT_ENABLED)
            gpsSerial_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            gpsSerial_EnableTxInt();
        #endif /* (gpsSerial_TX_INTERRUPT_ENABLED) */
     #endif /* (gpsSerial_TX_INTERRUPT_ENABLED) */

    #if (gpsSerial_INTERNAL_CLOCK_USED)
        gpsSerial_IntClock_Start();  /* Enable the clock */
    #endif /* (gpsSerial_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: gpsSerial_Stop
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
void gpsSerial_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)
        gpsSerial_RXBITCTR_CONTROL_REG &= (uint8) ~gpsSerial_CNTR_ENABLE;
    #endif /* (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED) */

    #if (gpsSerial_TX_ENABLED)
        #if(!gpsSerial_TXCLKGEN_DP)
            gpsSerial_TXBITCTR_CONTROL_REG &= (uint8) ~gpsSerial_CNTR_ENABLE;
        #endif /* (!gpsSerial_TXCLKGEN_DP) */
    #endif /* (gpsSerial_TX_ENABLED) */

    #if (gpsSerial_INTERNAL_CLOCK_USED)
        gpsSerial_IntClock_Stop();   /* Disable the clock */
    #endif /* (gpsSerial_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)
        gpsSerial_RXSTATUS_ACTL_REG  &= (uint8) ~gpsSerial_INT_ENABLE;

        #if (gpsSerial_RX_INTERRUPT_ENABLED)
            gpsSerial_DisableRxInt();
        #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */
    #endif /* (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED) */

    #if (gpsSerial_TX_ENABLED)
        gpsSerial_TXSTATUS_ACTL_REG &= (uint8) ~gpsSerial_INT_ENABLE;

        #if (gpsSerial_TX_INTERRUPT_ENABLED)
            gpsSerial_DisableTxInt();
        #endif /* (gpsSerial_TX_INTERRUPT_ENABLED) */
    #endif /* (gpsSerial_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: gpsSerial_ReadControlRegister
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
uint8 gpsSerial_ReadControlRegister(void) 
{
    #if (gpsSerial_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(gpsSerial_CONTROL_REG);
    #endif /* (gpsSerial_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: gpsSerial_WriteControlRegister
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
void  gpsSerial_WriteControlRegister(uint8 control) 
{
    #if (gpsSerial_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       gpsSerial_CONTROL_REG = control;
    #endif /* (gpsSerial_CONTROL_REG_REMOVED) */
}


#if(gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)
    /*******************************************************************************
    * Function Name: gpsSerial_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      gpsSerial_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      gpsSerial_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      gpsSerial_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      gpsSerial_RX_STS_BREAK            Interrupt on break.
    *      gpsSerial_RX_STS_OVERRUN          Interrupt on overrun error.
    *      gpsSerial_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      gpsSerial_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void gpsSerial_SetRxInterruptMode(uint8 intSrc) 
    {
        gpsSerial_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: gpsSerial_ReadRxData
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
    *  gpsSerial_rxBuffer - RAM buffer pointer for save received data.
    *  gpsSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  gpsSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  gpsSerial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 gpsSerial_ReadRxData(void) 
    {
        uint8 rxData;

    #if (gpsSerial_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        gpsSerial_DisableRxInt();

        locRxBufferRead  = gpsSerial_rxBufferRead;
        locRxBufferWrite = gpsSerial_rxBufferWrite;

        if( (gpsSerial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = gpsSerial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= gpsSerial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            gpsSerial_rxBufferRead = locRxBufferRead;

            if(gpsSerial_rxBufferLoopDetect != 0u)
            {
                gpsSerial_rxBufferLoopDetect = 0u;
                #if ((gpsSerial_RX_INTERRUPT_ENABLED) && (gpsSerial_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( gpsSerial_HD_ENABLED )
                        if((gpsSerial_CONTROL_REG & gpsSerial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            gpsSerial_RXSTATUS_MASK_REG  |= gpsSerial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        gpsSerial_RXSTATUS_MASK_REG  |= gpsSerial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end gpsSerial_HD_ENABLED */
                #endif /* ((gpsSerial_RX_INTERRUPT_ENABLED) && (gpsSerial_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = gpsSerial_RXDATA_REG;
        }

        gpsSerial_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = gpsSerial_RXDATA_REG;

    #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: gpsSerial_ReadRxStatus
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
    *  gpsSerial_RX_STS_FIFO_NOTEMPTY.
    *  gpsSerial_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  gpsSerial_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   gpsSerial_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   gpsSerial_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 gpsSerial_ReadRxStatus(void) 
    {
        uint8 status;

        status = gpsSerial_RXSTATUS_REG & gpsSerial_RX_HW_MASK;

    #if (gpsSerial_RX_INTERRUPT_ENABLED)
        if(gpsSerial_rxBufferOverflow != 0u)
        {
            status |= gpsSerial_RX_STS_SOFT_BUFF_OVER;
            gpsSerial_rxBufferOverflow = 0u;
        }
    #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: gpsSerial_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. gpsSerial_GetChar() is
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
    *  gpsSerial_rxBuffer - RAM buffer pointer for save received data.
    *  gpsSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  gpsSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  gpsSerial_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 gpsSerial_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (gpsSerial_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        gpsSerial_DisableRxInt();

        locRxBufferRead  = gpsSerial_rxBufferRead;
        locRxBufferWrite = gpsSerial_rxBufferWrite;

        if( (gpsSerial_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = gpsSerial_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= gpsSerial_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            gpsSerial_rxBufferRead = locRxBufferRead;

            if(gpsSerial_rxBufferLoopDetect != 0u)
            {
                gpsSerial_rxBufferLoopDetect = 0u;
                #if( (gpsSerial_RX_INTERRUPT_ENABLED) && (gpsSerial_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( gpsSerial_HD_ENABLED )
                        if((gpsSerial_CONTROL_REG & gpsSerial_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            gpsSerial_RXSTATUS_MASK_REG |= gpsSerial_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        gpsSerial_RXSTATUS_MASK_REG |= gpsSerial_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end gpsSerial_HD_ENABLED */
                #endif /* gpsSerial_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = gpsSerial_RXSTATUS_REG;
            if((rxStatus & gpsSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = gpsSerial_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (gpsSerial_RX_STS_BREAK | gpsSerial_RX_STS_PAR_ERROR |
                                gpsSerial_RX_STS_STOP_ERROR | gpsSerial_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        gpsSerial_EnableRxInt();

    #else

        rxStatus =gpsSerial_RXSTATUS_REG;
        if((rxStatus & gpsSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = gpsSerial_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (gpsSerial_RX_STS_BREAK | gpsSerial_RX_STS_PAR_ERROR |
                            gpsSerial_RX_STS_STOP_ERROR | gpsSerial_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: gpsSerial_GetByte
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
    uint16 gpsSerial_GetByte(void) 
    {
        
    #if (gpsSerial_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        gpsSerial_DisableRxInt();
        locErrorStatus = (uint16)gpsSerial_errorStatus;
        gpsSerial_errorStatus = 0u;
        gpsSerial_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | gpsSerial_ReadRxData() );
    #else
        return ( ((uint16)gpsSerial_ReadRxStatus() << 8u) | gpsSerial_ReadRxData() );
    #endif /* gpsSerial_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: gpsSerial_GetRxBufferSize
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
    *  gpsSerial_rxBufferWrite - used to calculate left bytes.
    *  gpsSerial_rxBufferRead - used to calculate left bytes.
    *  gpsSerial_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 gpsSerial_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (gpsSerial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        gpsSerial_DisableRxInt();

        if(gpsSerial_rxBufferRead == gpsSerial_rxBufferWrite)
        {
            if(gpsSerial_rxBufferLoopDetect != 0u)
            {
                size = gpsSerial_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(gpsSerial_rxBufferRead < gpsSerial_rxBufferWrite)
        {
            size = (gpsSerial_rxBufferWrite - gpsSerial_rxBufferRead);
        }
        else
        {
            size = (gpsSerial_RX_BUFFER_SIZE - gpsSerial_rxBufferRead) + gpsSerial_rxBufferWrite;
        }

        gpsSerial_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((gpsSerial_RXSTATUS_REG & gpsSerial_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: gpsSerial_ClearRxBuffer
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
    *  gpsSerial_rxBufferWrite - cleared to zero.
    *  gpsSerial_rxBufferRead - cleared to zero.
    *  gpsSerial_rxBufferLoopDetect - cleared to zero.
    *  gpsSerial_rxBufferOverflow - cleared to zero.
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
    void gpsSerial_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        gpsSerial_RXDATA_AUX_CTL_REG |= (uint8)  gpsSerial_RX_FIFO_CLR;
        gpsSerial_RXDATA_AUX_CTL_REG &= (uint8) ~gpsSerial_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (gpsSerial_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        gpsSerial_DisableRxInt();

        gpsSerial_rxBufferRead = 0u;
        gpsSerial_rxBufferWrite = 0u;
        gpsSerial_rxBufferLoopDetect = 0u;
        gpsSerial_rxBufferOverflow = 0u;

        gpsSerial_EnableRxInt();

    #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: gpsSerial_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  gpsSerial__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  gpsSerial__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  gpsSerial__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  gpsSerial__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  gpsSerial__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  gpsSerial_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  gpsSerial_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void gpsSerial_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(gpsSerial_RXHW_ADDRESS_ENABLED)
            #if(gpsSerial_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* gpsSerial_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = gpsSerial_CONTROL_REG & (uint8)~gpsSerial_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << gpsSerial_CTRL_RXADDR_MODE0_SHIFT);
                gpsSerial_CONTROL_REG = tmpCtrl;

                #if(gpsSerial_RX_INTERRUPT_ENABLED && \
                   (gpsSerial_RXBUFFERSIZE > gpsSerial_FIFO_LENGTH) )
                    gpsSerial_rxAddressMode = addressMode;
                    gpsSerial_rxAddressDetected = 0u;
                #endif /* End gpsSerial_RXBUFFERSIZE > gpsSerial_FIFO_LENGTH*/
            #endif /* End gpsSerial_CONTROL_REG_REMOVED */
        #else /* gpsSerial_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End gpsSerial_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: gpsSerial_SetRxAddress1
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
    void gpsSerial_SetRxAddress1(uint8 address) 
    {
        gpsSerial_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: gpsSerial_SetRxAddress2
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
    void gpsSerial_SetRxAddress2(uint8 address) 
    {
        gpsSerial_RXADDRESS2_REG = address;
    }

#endif  /* gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED*/


#if( (gpsSerial_TX_ENABLED) || (gpsSerial_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: gpsSerial_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   gpsSerial_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   gpsSerial_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   gpsSerial_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   gpsSerial_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void gpsSerial_SetTxInterruptMode(uint8 intSrc) 
    {
        gpsSerial_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: gpsSerial_WriteTxData
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
    *  gpsSerial_txBuffer - RAM buffer pointer for save data for transmission
    *  gpsSerial_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  gpsSerial_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  gpsSerial_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void gpsSerial_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(gpsSerial_initVar != 0u)
        {
        #if (gpsSerial_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            gpsSerial_DisableTxInt();

            if( (gpsSerial_txBufferRead == gpsSerial_txBufferWrite) &&
                ((gpsSerial_TXSTATUS_REG & gpsSerial_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                gpsSerial_TXDATA_REG = txDataByte;
            }
            else
            {
                if(gpsSerial_txBufferWrite >= gpsSerial_TX_BUFFER_SIZE)
                {
                    gpsSerial_txBufferWrite = 0u;
                }

                gpsSerial_txBuffer[gpsSerial_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                gpsSerial_txBufferWrite++;
            }

            gpsSerial_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            gpsSerial_TXDATA_REG = txDataByte;

        #endif /*(gpsSerial_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: gpsSerial_ReadTxStatus
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
    uint8 gpsSerial_ReadTxStatus(void) 
    {
        return(gpsSerial_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: gpsSerial_PutChar
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
    *  gpsSerial_txBuffer - RAM buffer pointer for save data for transmission
    *  gpsSerial_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  gpsSerial_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  gpsSerial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void gpsSerial_PutChar(uint8 txDataByte) 
    {
    #if (gpsSerial_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            gpsSerial_DisableTxInt();
        #endif /* (gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = gpsSerial_txBufferWrite;
            locTxBufferRead  = gpsSerial_txBufferRead;

        #if ((gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            gpsSerial_EnableTxInt();
        #endif /* (gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(gpsSerial_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((gpsSerial_TXSTATUS_REG & gpsSerial_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            gpsSerial_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= gpsSerial_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            gpsSerial_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            gpsSerial_DisableTxInt();
        #endif /* (gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            gpsSerial_txBufferWrite = locTxBufferWrite;

        #if ((gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3))
            gpsSerial_EnableTxInt();
        #endif /* (gpsSerial_TX_BUFFER_SIZE > gpsSerial_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (gpsSerial_TXSTATUS_REG & gpsSerial_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                gpsSerial_SetPendingTxInt();
            }
        }

    #else

        while((gpsSerial_TXSTATUS_REG & gpsSerial_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        gpsSerial_TXDATA_REG = txDataByte;

    #endif /* gpsSerial_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: gpsSerial_PutString
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
    *  gpsSerial_initVar - checked to identify that the component has been
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
    void gpsSerial_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(gpsSerial_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                gpsSerial_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: gpsSerial_PutArray
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
    *  gpsSerial_initVar - checked to identify that the component has been
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
    void gpsSerial_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(gpsSerial_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                gpsSerial_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: gpsSerial_PutCRLF
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
    *  gpsSerial_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void gpsSerial_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(gpsSerial_initVar != 0u)
        {
            gpsSerial_PutChar(txDataByte);
            gpsSerial_PutChar(0x0Du);
            gpsSerial_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: gpsSerial_GetTxBufferSize
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
    *  gpsSerial_txBufferWrite - used to calculate left space.
    *  gpsSerial_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 gpsSerial_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (gpsSerial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        gpsSerial_DisableTxInt();

        if(gpsSerial_txBufferRead == gpsSerial_txBufferWrite)
        {
            size = 0u;
        }
        else if(gpsSerial_txBufferRead < gpsSerial_txBufferWrite)
        {
            size = (gpsSerial_txBufferWrite - gpsSerial_txBufferRead);
        }
        else
        {
            size = (gpsSerial_TX_BUFFER_SIZE - gpsSerial_txBufferRead) +
                    gpsSerial_txBufferWrite;
        }

        gpsSerial_EnableTxInt();

    #else

        size = gpsSerial_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & gpsSerial_TX_STS_FIFO_FULL) != 0u)
        {
            size = gpsSerial_FIFO_LENGTH;
        }
        else if((size & gpsSerial_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (gpsSerial_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: gpsSerial_ClearTxBuffer
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
    *  gpsSerial_txBufferWrite - cleared to zero.
    *  gpsSerial_txBufferRead - cleared to zero.
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
    void gpsSerial_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        gpsSerial_TXDATA_AUX_CTL_REG |= (uint8)  gpsSerial_TX_FIFO_CLR;
        gpsSerial_TXDATA_AUX_CTL_REG &= (uint8) ~gpsSerial_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (gpsSerial_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        gpsSerial_DisableTxInt();

        gpsSerial_txBufferRead = 0u;
        gpsSerial_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        gpsSerial_EnableTxInt();

    #endif /* (gpsSerial_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: gpsSerial_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   gpsSerial_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   gpsSerial_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   gpsSerial_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   gpsSerial_SEND_WAIT_REINIT - Performs both options: 
    *      gpsSerial_SEND_BREAK and gpsSerial_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  gpsSerial_initVar - checked to identify that the component has been
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
    *     When interrupt appear with gpsSerial_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The gpsSerial_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void gpsSerial_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(gpsSerial_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(gpsSerial_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == gpsSerial_SEND_BREAK) ||
                (retMode == gpsSerial_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() |
                                                      gpsSerial_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                gpsSerial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = gpsSerial_TXSTATUS_REG;
                }
                while((tmpStat & gpsSerial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == gpsSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == gpsSerial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = gpsSerial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & gpsSerial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == gpsSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == gpsSerial_REINIT) ||
                (retMode == gpsSerial_SEND_WAIT_REINIT) )
            {
                gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() &
                                              (uint8)~gpsSerial_CTRL_HD_SEND_BREAK);
            }

        #else /* gpsSerial_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == gpsSerial_SEND_BREAK) ||
                (retMode == gpsSerial_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (gpsSerial_PARITY_TYPE != gpsSerial__B_UART__NONE_REVB) || \
                                    (gpsSerial_PARITY_TYPE_SW != 0u) )
                    gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() |
                                                          gpsSerial_CTRL_HD_SEND_BREAK);
                #endif /* End gpsSerial_PARITY_TYPE != gpsSerial__B_UART__NONE_REVB  */

                #if(gpsSerial_TXCLKGEN_DP)
                    txPeriod = gpsSerial_TXBITCLKTX_COMPLETE_REG;
                    gpsSerial_TXBITCLKTX_COMPLETE_REG = gpsSerial_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = gpsSerial_TXBITCTR_PERIOD_REG;
                    gpsSerial_TXBITCTR_PERIOD_REG = gpsSerial_TXBITCTR_BREAKBITS8X;
                #endif /* End gpsSerial_TXCLKGEN_DP */

                /* Send zeros */
                gpsSerial_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = gpsSerial_TXSTATUS_REG;
                }
                while((tmpStat & gpsSerial_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == gpsSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == gpsSerial_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = gpsSerial_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & gpsSerial_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == gpsSerial_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == gpsSerial_REINIT) ||
                (retMode == gpsSerial_SEND_WAIT_REINIT) )
            {

            #if(gpsSerial_TXCLKGEN_DP)
                gpsSerial_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                gpsSerial_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End gpsSerial_TXCLKGEN_DP */

            #if( (gpsSerial_PARITY_TYPE != gpsSerial__B_UART__NONE_REVB) || \
                 (gpsSerial_PARITY_TYPE_SW != 0u) )
                gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() &
                                                      (uint8) ~gpsSerial_CTRL_HD_SEND_BREAK);
            #endif /* End gpsSerial_PARITY_TYPE != NONE */
            }
        #endif    /* End gpsSerial_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: gpsSerial_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       gpsSerial_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       gpsSerial_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears gpsSerial_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void gpsSerial_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( gpsSerial_CONTROL_REG_REMOVED == 0u )
            gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() |
                                                  gpsSerial_CTRL_MARK);
        #endif /* End gpsSerial_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( gpsSerial_CONTROL_REG_REMOVED == 0u )
            gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() &
                                                  (uint8) ~gpsSerial_CTRL_MARK);
        #endif /* End gpsSerial_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndgpsSerial_TX_ENABLED */

#if(gpsSerial_HD_ENABLED)


    /*******************************************************************************
    * Function Name: gpsSerial_LoadRxConfig
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
    void gpsSerial_LoadRxConfig(void) 
    {
        gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() &
                                                (uint8)~gpsSerial_CTRL_HD_SEND);
        gpsSerial_RXBITCTR_PERIOD_REG = gpsSerial_HD_RXBITCTR_INIT;

    #if (gpsSerial_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        gpsSerial_SetRxInterruptMode(gpsSerial_INIT_RX_INTERRUPTS_MASK);
    #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: gpsSerial_LoadTxConfig
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
    void gpsSerial_LoadTxConfig(void) 
    {
    #if (gpsSerial_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        gpsSerial_SetRxInterruptMode(0u);
    #endif /* (gpsSerial_RX_INTERRUPT_ENABLED) */

        gpsSerial_WriteControlRegister(gpsSerial_ReadControlRegister() | gpsSerial_CTRL_HD_SEND);
        gpsSerial_RXBITCTR_PERIOD_REG = gpsSerial_HD_TXBITCTR_INIT;
    }

#endif  /* gpsSerial_HD_ENABLED */


/* [] END OF FILE */
