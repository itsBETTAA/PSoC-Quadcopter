/***************************************************************************//**
* \file serial_SPI_UART_INT.c
* \version 3.20
*
* \brief
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in SPI and UART modes.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "serial_PVT.h"
#include "serial_SPI_UART_PVT.h"
#include "cyapicallbacks.h"

#if (serial_SCB_IRQ_INTERNAL)
/*******************************************************************************
* Function Name: serial_SPI_UART_ISR
****************************************************************************//**
*
*  Handles the Interrupt Service Routine for the SCB SPI or UART modes.
*
*******************************************************************************/
CY_ISR(serial_SPI_UART_ISR)
{
#if (serial_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
#endif /* (serial_INTERNAL_RX_SW_BUFFER_CONST) */

#if (serial_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (serial_INTERNAL_TX_SW_BUFFER_CONST) */

#ifdef serial_SPI_UART_ISR_ENTRY_CALLBACK
    serial_SPI_UART_ISR_EntryCallback();
#endif /* serial_SPI_UART_ISR_ENTRY_CALLBACK */

    if (NULL != serial_customIntrHandler)
    {
        serial_customIntrHandler();
    }

    #if(serial_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        serial_ClearSpiExtClkInterruptSource(serial_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if (serial_CHECK_RX_SW_BUFFER)
    {
        if (serial_CHECK_INTR_RX_MASKED(serial_INTR_RX_NOT_EMPTY))
        {
            do
            {
                /* Move local head index */
                locHead = (serial_rxBufferHead + 1u);

                /* Adjust local head index */
                if (serial_INTERNAL_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if (locHead == serial_rxBufferTail)
                {
                    #if (serial_CHECK_UART_RTS_CONTROL_FLOW)
                    {
                        /* There is no space in the software buffer - disable the
                        * RX Not Empty interrupt source. The data elements are
                        * still being received into the RX FIFO until the RTS signal
                        * stops the transmitter. After the data element is read from the
                        * buffer, the RX Not Empty interrupt source is enabled to
                        * move the next data element in the software buffer.
                        */
                        serial_INTR_RX_MASK_REG &= ~serial_INTR_RX_NOT_EMPTY;
                        break;
                    }
                    #else
                    {
                        /* Overflow: through away received data element */
                        (void) serial_RX_FIFO_RD_REG;
                        serial_rxBufferOverflow = (uint8) serial_INTR_RX_OVERFLOW;
                    }
                    #endif
                }
                else
                {
                    /* Store received data */
                    serial_PutWordInRxBuffer(locHead, serial_RX_FIFO_RD_REG);

                    /* Move head index */
                    serial_rxBufferHead = locHead;
                }
            }
            while(0u != serial_GET_RX_FIFO_ENTRIES);

            serial_ClearRxInterruptSource(serial_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if (serial_CHECK_TX_SW_BUFFER)
    {
        if (serial_CHECK_INTR_TX_MASKED(serial_INTR_TX_NOT_FULL))
        {
            do
            {
                /* Check for room in TX software buffer */
                if (serial_txBufferHead != serial_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (serial_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if (serial_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    serial_TX_FIFO_WR_REG = serial_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    serial_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transfer */
                    serial_DISABLE_INTR_TX(serial_INTR_TX_NOT_FULL);
                    break;
                }
            }
            while (serial_SPI_UART_FIFO_SIZE != serial_GET_TX_FIFO_ENTRIES);

            serial_ClearTxInterruptSource(serial_INTR_TX_NOT_FULL);
        }
    }
    #endif

#ifdef serial_SPI_UART_ISR_EXIT_CALLBACK
    serial_SPI_UART_ISR_ExitCallback();
#endif /* serial_SPI_UART_ISR_EXIT_CALLBACK */

}

#endif /* (serial_SCB_IRQ_INTERNAL) */


/* [] END OF FILE */
