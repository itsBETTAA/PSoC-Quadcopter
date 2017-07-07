/*******************************************************************************
* File Name: debugSerialINT.c
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

#include "debugSerial.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (debugSerial_RX_INTERRUPT_ENABLED && (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED))
    /*******************************************************************************
    * Function Name: debugSerial_RXISR
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
    *  debugSerial_rxBuffer - RAM buffer pointer for save received data.
    *  debugSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  debugSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  debugSerial_rxBufferOverflow - software overflow flag. Set to one
    *     when debugSerial_rxBufferWrite index overtakes
    *     debugSerial_rxBufferRead index.
    *  debugSerial_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when debugSerial_rxBufferWrite is equal to
    *    debugSerial_rxBufferRead
    *  debugSerial_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  debugSerial_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(debugSerial_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef debugSerial_RXISR_ENTRY_CALLBACK
        debugSerial_RXISR_EntryCallback();
    #endif /* debugSerial_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START debugSerial_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = debugSerial_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in debugSerial_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (debugSerial_RX_STS_BREAK | 
                            debugSerial_RX_STS_PAR_ERROR |
                            debugSerial_RX_STS_STOP_ERROR | 
                            debugSerial_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                debugSerial_errorStatus |= readStatus & ( debugSerial_RX_STS_BREAK | 
                                                            debugSerial_RX_STS_PAR_ERROR | 
                                                            debugSerial_RX_STS_STOP_ERROR | 
                                                            debugSerial_RX_STS_OVERRUN);
                /* `#START debugSerial_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef debugSerial_RXISR_ERROR_CALLBACK
                debugSerial_RXISR_ERROR_Callback();
            #endif /* debugSerial_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & debugSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = debugSerial_RXDATA_REG;
            #if (debugSerial_RXHW_ADDRESS_ENABLED)
                if(debugSerial_rxAddressMode == (uint8)debugSerial__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & debugSerial_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & debugSerial_RX_STS_ADDR_MATCH) != 0u)
                        {
                            debugSerial_rxAddressDetected = 1u;
                        }
                        else
                        {
                            debugSerial_rxAddressDetected = 0u;
                        }
                    }
                    if(debugSerial_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        debugSerial_rxBuffer[debugSerial_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    debugSerial_rxBuffer[debugSerial_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                debugSerial_rxBuffer[debugSerial_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (debugSerial_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(debugSerial_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        debugSerial_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    debugSerial_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(debugSerial_rxBufferWrite >= debugSerial_RX_BUFFER_SIZE)
                    {
                        debugSerial_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(debugSerial_rxBufferWrite == debugSerial_rxBufferRead)
                    {
                        debugSerial_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (debugSerial_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            debugSerial_RXSTATUS_MASK_REG  &= (uint8)~debugSerial_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(debugSerial_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (debugSerial_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & debugSerial_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START debugSerial_RXISR_END` */

        /* `#END` */

    #ifdef debugSerial_RXISR_EXIT_CALLBACK
        debugSerial_RXISR_ExitCallback();
    #endif /* debugSerial_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (debugSerial_RX_INTERRUPT_ENABLED && (debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)) */


#if (debugSerial_TX_INTERRUPT_ENABLED && debugSerial_TX_ENABLED)
    /*******************************************************************************
    * Function Name: debugSerial_TXISR
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
    *  debugSerial_txBuffer - RAM buffer pointer for transmit data from.
    *  debugSerial_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  debugSerial_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(debugSerial_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef debugSerial_TXISR_ENTRY_CALLBACK
        debugSerial_TXISR_EntryCallback();
    #endif /* debugSerial_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START debugSerial_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((debugSerial_txBufferRead != debugSerial_txBufferWrite) &&
             ((debugSerial_TXSTATUS_REG & debugSerial_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(debugSerial_txBufferRead >= debugSerial_TX_BUFFER_SIZE)
            {
                debugSerial_txBufferRead = 0u;
            }

            debugSerial_TXDATA_REG = debugSerial_txBuffer[debugSerial_txBufferRead];

            /* Set next pointer */
            debugSerial_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START debugSerial_TXISR_END` */

        /* `#END` */

    #ifdef debugSerial_TXISR_EXIT_CALLBACK
        debugSerial_TXISR_ExitCallback();
    #endif /* debugSerial_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (debugSerial_TX_INTERRUPT_ENABLED && debugSerial_TX_ENABLED) */


/* [] END OF FILE */
