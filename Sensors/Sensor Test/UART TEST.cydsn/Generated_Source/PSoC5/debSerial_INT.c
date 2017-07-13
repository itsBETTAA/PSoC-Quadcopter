/*******************************************************************************
* File Name: debSerialINT.c
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

#include "debSerial.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (debSerial_RX_INTERRUPT_ENABLED && (debSerial_RX_ENABLED || debSerial_HD_ENABLED))
    /*******************************************************************************
    * Function Name: debSerial_RXISR
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
    *  debSerial_rxBuffer - RAM buffer pointer for save received data.
    *  debSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  debSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  debSerial_rxBufferOverflow - software overflow flag. Set to one
    *     when debSerial_rxBufferWrite index overtakes
    *     debSerial_rxBufferRead index.
    *  debSerial_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when debSerial_rxBufferWrite is equal to
    *    debSerial_rxBufferRead
    *  debSerial_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  debSerial_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(debSerial_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef debSerial_RXISR_ENTRY_CALLBACK
        debSerial_RXISR_EntryCallback();
    #endif /* debSerial_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START debSerial_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = debSerial_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in debSerial_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (debSerial_RX_STS_BREAK | 
                            debSerial_RX_STS_PAR_ERROR |
                            debSerial_RX_STS_STOP_ERROR | 
                            debSerial_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                debSerial_errorStatus |= readStatus & ( debSerial_RX_STS_BREAK | 
                                                            debSerial_RX_STS_PAR_ERROR | 
                                                            debSerial_RX_STS_STOP_ERROR | 
                                                            debSerial_RX_STS_OVERRUN);
                /* `#START debSerial_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef debSerial_RXISR_ERROR_CALLBACK
                debSerial_RXISR_ERROR_Callback();
            #endif /* debSerial_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & debSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = debSerial_RXDATA_REG;
            #if (debSerial_RXHW_ADDRESS_ENABLED)
                if(debSerial_rxAddressMode == (uint8)debSerial__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & debSerial_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & debSerial_RX_STS_ADDR_MATCH) != 0u)
                        {
                            debSerial_rxAddressDetected = 1u;
                        }
                        else
                        {
                            debSerial_rxAddressDetected = 0u;
                        }
                    }
                    if(debSerial_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        debSerial_rxBuffer[debSerial_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    debSerial_rxBuffer[debSerial_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                debSerial_rxBuffer[debSerial_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (debSerial_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(debSerial_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        debSerial_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    debSerial_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(debSerial_rxBufferWrite >= debSerial_RX_BUFFER_SIZE)
                    {
                        debSerial_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(debSerial_rxBufferWrite == debSerial_rxBufferRead)
                    {
                        debSerial_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (debSerial_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            debSerial_RXSTATUS_MASK_REG  &= (uint8)~debSerial_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(debSerial_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (debSerial_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & debSerial_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START debSerial_RXISR_END` */

        /* `#END` */

    #ifdef debSerial_RXISR_EXIT_CALLBACK
        debSerial_RXISR_ExitCallback();
    #endif /* debSerial_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (debSerial_RX_INTERRUPT_ENABLED && (debSerial_RX_ENABLED || debSerial_HD_ENABLED)) */


#if (debSerial_TX_INTERRUPT_ENABLED && debSerial_TX_ENABLED)
    /*******************************************************************************
    * Function Name: debSerial_TXISR
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
    *  debSerial_txBuffer - RAM buffer pointer for transmit data from.
    *  debSerial_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  debSerial_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(debSerial_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef debSerial_TXISR_ENTRY_CALLBACK
        debSerial_TXISR_EntryCallback();
    #endif /* debSerial_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START debSerial_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((debSerial_txBufferRead != debSerial_txBufferWrite) &&
             ((debSerial_TXSTATUS_REG & debSerial_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(debSerial_txBufferRead >= debSerial_TX_BUFFER_SIZE)
            {
                debSerial_txBufferRead = 0u;
            }

            debSerial_TXDATA_REG = debSerial_txBuffer[debSerial_txBufferRead];

            /* Set next pointer */
            debSerial_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START debSerial_TXISR_END` */

        /* `#END` */

    #ifdef debSerial_TXISR_EXIT_CALLBACK
        debSerial_TXISR_ExitCallback();
    #endif /* debSerial_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (debSerial_TX_INTERRUPT_ENABLED && debSerial_TX_ENABLED) */


/* [] END OF FILE */
