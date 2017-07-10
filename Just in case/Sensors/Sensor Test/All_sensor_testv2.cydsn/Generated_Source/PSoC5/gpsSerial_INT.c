/*******************************************************************************
* File Name: gpsSerialINT.c
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

#include "gpsSerial.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (gpsSerial_RX_INTERRUPT_ENABLED && (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED))
    /*******************************************************************************
    * Function Name: gpsSerial_RXISR
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
    *  gpsSerial_rxBuffer - RAM buffer pointer for save received data.
    *  gpsSerial_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  gpsSerial_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  gpsSerial_rxBufferOverflow - software overflow flag. Set to one
    *     when gpsSerial_rxBufferWrite index overtakes
    *     gpsSerial_rxBufferRead index.
    *  gpsSerial_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when gpsSerial_rxBufferWrite is equal to
    *    gpsSerial_rxBufferRead
    *  gpsSerial_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  gpsSerial_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(gpsSerial_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef gpsSerial_RXISR_ENTRY_CALLBACK
        gpsSerial_RXISR_EntryCallback();
    #endif /* gpsSerial_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START gpsSerial_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = gpsSerial_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in gpsSerial_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (gpsSerial_RX_STS_BREAK | 
                            gpsSerial_RX_STS_PAR_ERROR |
                            gpsSerial_RX_STS_STOP_ERROR | 
                            gpsSerial_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                gpsSerial_errorStatus |= readStatus & ( gpsSerial_RX_STS_BREAK | 
                                                            gpsSerial_RX_STS_PAR_ERROR | 
                                                            gpsSerial_RX_STS_STOP_ERROR | 
                                                            gpsSerial_RX_STS_OVERRUN);
                /* `#START gpsSerial_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef gpsSerial_RXISR_ERROR_CALLBACK
                gpsSerial_RXISR_ERROR_Callback();
            #endif /* gpsSerial_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & gpsSerial_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = gpsSerial_RXDATA_REG;
            #if (gpsSerial_RXHW_ADDRESS_ENABLED)
                if(gpsSerial_rxAddressMode == (uint8)gpsSerial__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & gpsSerial_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & gpsSerial_RX_STS_ADDR_MATCH) != 0u)
                        {
                            gpsSerial_rxAddressDetected = 1u;
                        }
                        else
                        {
                            gpsSerial_rxAddressDetected = 0u;
                        }
                    }
                    if(gpsSerial_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        gpsSerial_rxBuffer[gpsSerial_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    gpsSerial_rxBuffer[gpsSerial_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                gpsSerial_rxBuffer[gpsSerial_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (gpsSerial_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(gpsSerial_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        gpsSerial_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    gpsSerial_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(gpsSerial_rxBufferWrite >= gpsSerial_RX_BUFFER_SIZE)
                    {
                        gpsSerial_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(gpsSerial_rxBufferWrite == gpsSerial_rxBufferRead)
                    {
                        gpsSerial_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (gpsSerial_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            gpsSerial_RXSTATUS_MASK_REG  &= (uint8)~gpsSerial_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(gpsSerial_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (gpsSerial_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & gpsSerial_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START gpsSerial_RXISR_END` */

        /* `#END` */

    #ifdef gpsSerial_RXISR_EXIT_CALLBACK
        gpsSerial_RXISR_ExitCallback();
    #endif /* gpsSerial_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (gpsSerial_RX_INTERRUPT_ENABLED && (gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)) */


#if (gpsSerial_TX_INTERRUPT_ENABLED && gpsSerial_TX_ENABLED)
    /*******************************************************************************
    * Function Name: gpsSerial_TXISR
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
    *  gpsSerial_txBuffer - RAM buffer pointer for transmit data from.
    *  gpsSerial_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  gpsSerial_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(gpsSerial_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef gpsSerial_TXISR_ENTRY_CALLBACK
        gpsSerial_TXISR_EntryCallback();
    #endif /* gpsSerial_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START gpsSerial_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((gpsSerial_txBufferRead != gpsSerial_txBufferWrite) &&
             ((gpsSerial_TXSTATUS_REG & gpsSerial_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(gpsSerial_txBufferRead >= gpsSerial_TX_BUFFER_SIZE)
            {
                gpsSerial_txBufferRead = 0u;
            }

            gpsSerial_TXDATA_REG = gpsSerial_txBuffer[gpsSerial_txBufferRead];

            /* Set next pointer */
            gpsSerial_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START gpsSerial_TXISR_END` */

        /* `#END` */

    #ifdef gpsSerial_TXISR_EXIT_CALLBACK
        gpsSerial_TXISR_ExitCallback();
    #endif /* gpsSerial_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (gpsSerial_TX_INTERRUPT_ENABLED && gpsSerial_TX_ENABLED) */


/* [] END OF FILE */
