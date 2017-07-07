/*******************************************************************************
* File Name: debugINT.c
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

#include "debug.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (debug_RX_INTERRUPT_ENABLED && (debug_RX_ENABLED || debug_HD_ENABLED))
    /*******************************************************************************
    * Function Name: debug_RXISR
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
    *  debug_rxBuffer - RAM buffer pointer for save received data.
    *  debug_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  debug_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  debug_rxBufferOverflow - software overflow flag. Set to one
    *     when debug_rxBufferWrite index overtakes
    *     debug_rxBufferRead index.
    *  debug_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when debug_rxBufferWrite is equal to
    *    debug_rxBufferRead
    *  debug_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  debug_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(debug_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef debug_RXISR_ENTRY_CALLBACK
        debug_RXISR_EntryCallback();
    #endif /* debug_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START debug_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = debug_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in debug_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (debug_RX_STS_BREAK | 
                            debug_RX_STS_PAR_ERROR |
                            debug_RX_STS_STOP_ERROR | 
                            debug_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                debug_errorStatus |= readStatus & ( debug_RX_STS_BREAK | 
                                                            debug_RX_STS_PAR_ERROR | 
                                                            debug_RX_STS_STOP_ERROR | 
                                                            debug_RX_STS_OVERRUN);
                /* `#START debug_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef debug_RXISR_ERROR_CALLBACK
                debug_RXISR_ERROR_Callback();
            #endif /* debug_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & debug_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = debug_RXDATA_REG;
            #if (debug_RXHW_ADDRESS_ENABLED)
                if(debug_rxAddressMode == (uint8)debug__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & debug_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & debug_RX_STS_ADDR_MATCH) != 0u)
                        {
                            debug_rxAddressDetected = 1u;
                        }
                        else
                        {
                            debug_rxAddressDetected = 0u;
                        }
                    }
                    if(debug_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        debug_rxBuffer[debug_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    debug_rxBuffer[debug_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                debug_rxBuffer[debug_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (debug_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(debug_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        debug_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    debug_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(debug_rxBufferWrite >= debug_RX_BUFFER_SIZE)
                    {
                        debug_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(debug_rxBufferWrite == debug_rxBufferRead)
                    {
                        debug_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (debug_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            debug_RXSTATUS_MASK_REG  &= (uint8)~debug_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(debug_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (debug_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & debug_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START debug_RXISR_END` */

        /* `#END` */

    #ifdef debug_RXISR_EXIT_CALLBACK
        debug_RXISR_ExitCallback();
    #endif /* debug_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (debug_RX_INTERRUPT_ENABLED && (debug_RX_ENABLED || debug_HD_ENABLED)) */


#if (debug_TX_INTERRUPT_ENABLED && debug_TX_ENABLED)
    /*******************************************************************************
    * Function Name: debug_TXISR
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
    *  debug_txBuffer - RAM buffer pointer for transmit data from.
    *  debug_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  debug_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(debug_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef debug_TXISR_ENTRY_CALLBACK
        debug_TXISR_EntryCallback();
    #endif /* debug_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START debug_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((debug_txBufferRead != debug_txBufferWrite) &&
             ((debug_TXSTATUS_REG & debug_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(debug_txBufferRead >= debug_TX_BUFFER_SIZE)
            {
                debug_txBufferRead = 0u;
            }

            debug_TXDATA_REG = debug_txBuffer[debug_txBufferRead];

            /* Set next pointer */
            debug_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START debug_TXISR_END` */

        /* `#END` */

    #ifdef debug_TXISR_EXIT_CALLBACK
        debug_TXISR_ExitCallback();
    #endif /* debug_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (debug_TX_INTERRUPT_ENABLED && debug_TX_ENABLED) */


/* [] END OF FILE */
