/*******************************************************************************
* File Name: serialINT.c
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

#include "serial.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (serial_RX_INTERRUPT_ENABLED && (serial_RX_ENABLED || serial_HD_ENABLED))
    /*******************************************************************************
    * Function Name: serial_RXISR
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
    *  serial_rxBuffer - RAM buffer pointer for save received data.
    *  serial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  serial_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  serial_rxBufferOverflow - software overflow flag. Set to one
    *     when serial_rxBufferWrite index overtakes
    *     serial_rxBufferRead index.
    *  serial_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when serial_rxBufferWrite is equal to
    *    serial_rxBufferRead
    *  serial_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  serial_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(serial_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef serial_RXISR_ENTRY_CALLBACK
        serial_RXISR_EntryCallback();
    #endif /* serial_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START serial_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = serial_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in serial_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (serial_RX_STS_BREAK | 
                            serial_RX_STS_PAR_ERROR |
                            serial_RX_STS_STOP_ERROR | 
                            serial_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                serial_errorStatus |= readStatus & ( serial_RX_STS_BREAK | 
                                                            serial_RX_STS_PAR_ERROR | 
                                                            serial_RX_STS_STOP_ERROR | 
                                                            serial_RX_STS_OVERRUN);
                /* `#START serial_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef serial_RXISR_ERROR_CALLBACK
                serial_RXISR_ERROR_Callback();
            #endif /* serial_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & serial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = serial_RXDATA_REG;
            #if (serial_RXHW_ADDRESS_ENABLED)
                if(serial_rxAddressMode == (uint8)serial__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & serial_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & serial_RX_STS_ADDR_MATCH) != 0u)
                        {
                            serial_rxAddressDetected = 1u;
                        }
                        else
                        {
                            serial_rxAddressDetected = 0u;
                        }
                    }
                    if(serial_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        serial_rxBuffer[serial_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    serial_rxBuffer[serial_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                serial_rxBuffer[serial_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (serial_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(serial_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        serial_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    serial_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(serial_rxBufferWrite >= serial_RX_BUFFER_SIZE)
                    {
                        serial_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(serial_rxBufferWrite == serial_rxBufferRead)
                    {
                        serial_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (serial_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            serial_RXSTATUS_MASK_REG  &= (uint8)~serial_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(serial_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (serial_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & serial_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START serial_RXISR_END` */

        /* `#END` */

    #ifdef serial_RXISR_EXIT_CALLBACK
        serial_RXISR_ExitCallback();
    #endif /* serial_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (serial_RX_INTERRUPT_ENABLED && (serial_RX_ENABLED || serial_HD_ENABLED)) */


#if (serial_TX_INTERRUPT_ENABLED && serial_TX_ENABLED)
    /*******************************************************************************
    * Function Name: serial_TXISR
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
    *  serial_txBuffer - RAM buffer pointer for transmit data from.
    *  serial_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  serial_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(serial_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef serial_TXISR_ENTRY_CALLBACK
        serial_TXISR_EntryCallback();
    #endif /* serial_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START serial_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((serial_txBufferRead != serial_txBufferWrite) &&
             ((serial_TXSTATUS_REG & serial_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(serial_txBufferRead >= serial_TX_BUFFER_SIZE)
            {
                serial_txBufferRead = 0u;
            }

            serial_TXDATA_REG = serial_txBuffer[serial_txBufferRead];

            /* Set next pointer */
            serial_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START serial_TXISR_END` */

        /* `#END` */

    #ifdef serial_TXISR_EXIT_CALLBACK
        serial_TXISR_ExitCallback();
    #endif /* serial_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (serial_TX_INTERRUPT_ENABLED && serial_TX_ENABLED) */


/* [] END OF FILE */
