/*******************************************************************************
* File Name: killSwitchINT.c
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

#include "killSwitch.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (killSwitch_RX_INTERRUPT_ENABLED && (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED))
    /*******************************************************************************
    * Function Name: killSwitch_RXISR
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
    *  killSwitch_rxBuffer - RAM buffer pointer for save received data.
    *  killSwitch_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  killSwitch_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  killSwitch_rxBufferOverflow - software overflow flag. Set to one
    *     when killSwitch_rxBufferWrite index overtakes
    *     killSwitch_rxBufferRead index.
    *  killSwitch_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when killSwitch_rxBufferWrite is equal to
    *    killSwitch_rxBufferRead
    *  killSwitch_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  killSwitch_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(killSwitch_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef killSwitch_RXISR_ENTRY_CALLBACK
        killSwitch_RXISR_EntryCallback();
    #endif /* killSwitch_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START killSwitch_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = killSwitch_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in killSwitch_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (killSwitch_RX_STS_BREAK | 
                            killSwitch_RX_STS_PAR_ERROR |
                            killSwitch_RX_STS_STOP_ERROR | 
                            killSwitch_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                killSwitch_errorStatus |= readStatus & ( killSwitch_RX_STS_BREAK | 
                                                            killSwitch_RX_STS_PAR_ERROR | 
                                                            killSwitch_RX_STS_STOP_ERROR | 
                                                            killSwitch_RX_STS_OVERRUN);
                /* `#START killSwitch_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef killSwitch_RXISR_ERROR_CALLBACK
                killSwitch_RXISR_ERROR_Callback();
            #endif /* killSwitch_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & killSwitch_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = killSwitch_RXDATA_REG;
            #if (killSwitch_RXHW_ADDRESS_ENABLED)
                if(killSwitch_rxAddressMode == (uint8)killSwitch__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & killSwitch_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & killSwitch_RX_STS_ADDR_MATCH) != 0u)
                        {
                            killSwitch_rxAddressDetected = 1u;
                        }
                        else
                        {
                            killSwitch_rxAddressDetected = 0u;
                        }
                    }
                    if(killSwitch_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        killSwitch_rxBuffer[killSwitch_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    killSwitch_rxBuffer[killSwitch_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                killSwitch_rxBuffer[killSwitch_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (killSwitch_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(killSwitch_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        killSwitch_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    killSwitch_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(killSwitch_rxBufferWrite >= killSwitch_RX_BUFFER_SIZE)
                    {
                        killSwitch_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(killSwitch_rxBufferWrite == killSwitch_rxBufferRead)
                    {
                        killSwitch_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (killSwitch_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            killSwitch_RXSTATUS_MASK_REG  &= (uint8)~killSwitch_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(killSwitch_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (killSwitch_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & killSwitch_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START killSwitch_RXISR_END` */

        /* `#END` */

    #ifdef killSwitch_RXISR_EXIT_CALLBACK
        killSwitch_RXISR_ExitCallback();
    #endif /* killSwitch_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (killSwitch_RX_INTERRUPT_ENABLED && (killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)) */


#if (killSwitch_TX_INTERRUPT_ENABLED && killSwitch_TX_ENABLED)
    /*******************************************************************************
    * Function Name: killSwitch_TXISR
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
    *  killSwitch_txBuffer - RAM buffer pointer for transmit data from.
    *  killSwitch_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  killSwitch_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(killSwitch_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef killSwitch_TXISR_ENTRY_CALLBACK
        killSwitch_TXISR_EntryCallback();
    #endif /* killSwitch_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START killSwitch_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((killSwitch_txBufferRead != killSwitch_txBufferWrite) &&
             ((killSwitch_TXSTATUS_REG & killSwitch_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(killSwitch_txBufferRead >= killSwitch_TX_BUFFER_SIZE)
            {
                killSwitch_txBufferRead = 0u;
            }

            killSwitch_TXDATA_REG = killSwitch_txBuffer[killSwitch_txBufferRead];

            /* Set next pointer */
            killSwitch_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START killSwitch_TXISR_END` */

        /* `#END` */

    #ifdef killSwitch_TXISR_EXIT_CALLBACK
        killSwitch_TXISR_ExitCallback();
    #endif /* killSwitch_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (killSwitch_TX_INTERRUPT_ENABLED && killSwitch_TX_ENABLED) */


/* [] END OF FILE */
