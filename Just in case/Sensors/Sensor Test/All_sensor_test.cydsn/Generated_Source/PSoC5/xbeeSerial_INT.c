/*******************************************************************************
* File Name: xbeeSerialINT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "xbeeSerial.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (xbeeSerial_RX_INTERRUPT_ENABLED && (xbeeSerial_RX_ENABLED || xbeeSerial_HD_ENABLED))
    /*******************************************************************************
    * Function Name: xbeeSerial_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  xbeeSerial_rxBuffer - RAM buffer pointer for save received data.
    *  xbeeSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  xbeeSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  xbeeSerial_rxBufferOverflow - software overflow flag. Set to one
    *     when xbeeSerial_rxBufferWrite index overtakes
    *     xbeeSerial_rxBufferRead index.
    *  xbeeSerial_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when xbeeSerial_rxBufferWrite is equal to
    *    xbeeSerial_rxBufferRead
    *  xbeeSerial_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  xbeeSerial_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(xbeeSerial_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef xbeeSerial_RXISR_ENTRY_CALLBACK
        xbeeSerial_RXISR_EntryCallback();
    #endif /* xbeeSerial_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START xbeeSerial_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = xbeeSerial_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in xbeeSerial_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (xbeeSerial_RX_STS_BREAK | 
                            xbeeSerial_RX_STS_PAR_ERROR |
                            xbeeSerial_RX_STS_STOP_ERROR | 
                            xbeeSerial_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                xbeeSerial_errorStatus |= readStatus & ( xbeeSerial_RX_STS_BREAK | 
                                                            xbeeSerial_RX_STS_PAR_ERROR | 
                                                            xbeeSerial_RX_STS_STOP_ERROR | 
                                                            xbeeSerial_RX_STS_OVERRUN);
                /* `#START xbeeSerial_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef xbeeSerial_RXISR_ERROR_CALLBACK
                xbeeSerial_RXISR_ERROR_Callback();
            #endif /* xbeeSerial_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & xbeeSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = xbeeSerial_RXDATA_REG;
            #if (xbeeSerial_RXHW_ADDRESS_ENABLED)
                if(xbeeSerial_rxAddressMode == (uint8)xbeeSerial__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & xbeeSerial_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & xbeeSerial_RX_STS_ADDR_MATCH) != 0u)
                        {
                            xbeeSerial_rxAddressDetected = 1u;
                        }
                        else
                        {
                            xbeeSerial_rxAddressDetected = 0u;
                        }
                    }
                    if(xbeeSerial_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        xbeeSerial_rxBuffer[xbeeSerial_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    xbeeSerial_rxBuffer[xbeeSerial_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                xbeeSerial_rxBuffer[xbeeSerial_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (xbeeSerial_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(xbeeSerial_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        xbeeSerial_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    xbeeSerial_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(xbeeSerial_rxBufferWrite >= xbeeSerial_RX_BUFFER_SIZE)
                    {
                        xbeeSerial_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(xbeeSerial_rxBufferWrite == xbeeSerial_rxBufferRead)
                    {
                        xbeeSerial_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (xbeeSerial_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            xbeeSerial_RXSTATUS_MASK_REG  &= (uint8)~xbeeSerial_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(xbeeSerial_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (xbeeSerial_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & xbeeSerial_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START xbeeSerial_RXISR_END` */

        /* `#END` */

    #ifdef xbeeSerial_RXISR_EXIT_CALLBACK
        xbeeSerial_RXISR_ExitCallback();
    #endif /* xbeeSerial_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (xbeeSerial_RX_INTERRUPT_ENABLED && (xbeeSerial_RX_ENABLED || xbeeSerial_HD_ENABLED)) */


#if (xbeeSerial_TX_INTERRUPT_ENABLED && xbeeSerial_TX_ENABLED)
    /*******************************************************************************
    * Function Name: xbeeSerial_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  xbeeSerial_txBuffer - RAM buffer pointer for transmit data from.
    *  xbeeSerial_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  xbeeSerial_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(xbeeSerial_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef xbeeSerial_TXISR_ENTRY_CALLBACK
        xbeeSerial_TXISR_EntryCallback();
    #endif /* xbeeSerial_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START xbeeSerial_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((xbeeSerial_txBufferRead != xbeeSerial_txBufferWrite) &&
             ((xbeeSerial_TXSTATUS_REG & xbeeSerial_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(xbeeSerial_txBufferRead >= xbeeSerial_TX_BUFFER_SIZE)
            {
                xbeeSerial_txBufferRead = 0u;
            }

            xbeeSerial_TXDATA_REG = xbeeSerial_txBuffer[xbeeSerial_txBufferRead];

            /* Set next pointer */
            xbeeSerial_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START xbeeSerial_TXISR_END` */

        /* `#END` */

    #ifdef xbeeSerial_TXISR_EXIT_CALLBACK
        xbeeSerial_TXISR_ExitCallback();
    #endif /* xbeeSerial_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (xbeeSerial_TX_INTERRUPT_ENABLED && xbeeSerial_TX_ENABLED) */


/* [] END OF FILE */
