/*******************************************************************************
* File Name: sw_serial.h
* Version 1.50
*
* Description:
*  This file provides constants and parameter values for the Software Transmit
*  UART Component.
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef CY_SW_TX_UART_sw_serial_H
#define CY_SW_TX_UART_sw_serial_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"

#define sw_serial_BAUD_RATE                      (115200u)
#define sw_serial_PIN_STATIC_MODE                (0u)


/***************************************
*        Function Prototypes
***************************************/
#if(sw_serial_PIN_STATIC_MODE == 1u)
    void sw_serial_Start(void) ;
#else
    void sw_serial_StartEx(uint8 port, uint8 pin) ;
#endif /* (sw_serial_PIN_STATIC_MODE == 1u) */

void sw_serial_Stop(void) ;
void sw_serial_PutChar(uint8 txDataByte) ;
void sw_serial_PutString(const char8 string[]) ;
void sw_serial_PutArray(const uint8 array[], uint32 byteCount) ;
void sw_serial_PutHexByte(uint8 txHexByte) ;
void sw_serial_PutHexInt(uint16 txHexInt) ;
void sw_serial_PutCRLF(void) ;

#endif /* CY_SW_TX_UART_sw_serial_H */


/* [] END OF FILE */
