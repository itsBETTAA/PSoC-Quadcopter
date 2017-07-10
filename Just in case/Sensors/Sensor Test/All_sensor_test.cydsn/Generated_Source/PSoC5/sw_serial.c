/*******************************************************************************
* File Name: sw_serial.c
* Version 1.50
*
* Description:
*  This file provides the source code to the API for the Software Transmit UART.
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "sw_serial_PVT.h"

#if(sw_serial_PIN_STATIC_MODE == 1u)
    uint32 sw_serial_pinNumber = sw_serial_PIN_NUMBER;
    uint32 sw_serial_pinPortNumber = sw_serial_PIN_PORT_NUMBER;
    #if(CY_PSOC3)
        uint32 pdata sw_serial_pinDrAdress = sw_serial_PIN_DR;
    #else
        uint32 sw_serial_pinDrAdress = sw_serial_PIN_DR;
    #endif /* (CY_PSOC3) */
#else
    uint32 sw_serial_pinNumber;
    uint32 sw_serial_pinPortNumber;
    #if(CY_PSOC3)
        uint32 pdata sw_serial_pinDrAdress;
    #else
        uint32 sw_serial_pinDrAdress;
    #endif /* (CY_PSOC3) */
#endif /* (sw_serial_PIN_STATIC_MODE == 1u) */


#if(sw_serial_PIN_STATIC_MODE == 1u)
    /*******************************************************************************
    * Function Name: sw_serial_Start
    ********************************************************************************
    *
    * Summary:
    *  Empty function. Included for consistency with other
    *  components. This API is not available when PinAssignmentMethod
    *  is set to Dynamic.
    *
    * Parameters:
    *  None
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void sw_serial_Start(void) 
    {

    }
#else
    /*******************************************************************************
    * Function Name: sw_serial_StartEx
    ********************************************************************************
    *
    * Summary:
    *  Configures the SW Tx UART to use the pin specified
    *  by the parameters. This API is only available when
    *  PinAssignmentMethod is set to Dynamic.
    *
    * Parameters:
    *  port:  Port number for dynamic pin assignment
    *  pin:   Pin number for dynamic pin assignment
    *
    * Return:
    *  None
    *
    *******************************************************************************/
    void sw_serial_StartEx(uint8 port, uint8 pin) 
    {
        uint32 portConfigAddr;

        #if (CY_PSOC4)
            uint32 portDataRegAddr;
        #endif /* (CY_PSOC4) */

        if ((pin <= sw_serial_MAX_PIN_NUMBER) && (port <= sw_serial_MAX_PORT_NUMBER))
        {
            #if (!CY_PSOC4)
                portConfigAddr = sw_serial_PORT_CNF_BASE;
                portConfigAddr += ((uint32)port * (sw_serial_MAX_PIN_NUMBER + 1u)) + pin;
                CyPins_SetPinDriveMode(portConfigAddr, CY_PINS_DM_STRONG);
                CyPins_SetPin(portConfigAddr);
                sw_serial_pinDrAdress = portConfigAddr;
            #else
                portConfigAddr = sw_serial_PORT_CNF_BASE + (sw_serial_PORT_CNF_SIZE * port) +
                                                                                sw_serial_PORT_CNF_MODE_OFFSET;
                CY_SYS_PINS_SET_DRIVE_MODE(portConfigAddr, pin, CY_SYS_PINS_DM_STRONG);
                portDataRegAddr = sw_serial_PORT_CNF_BASE + (sw_serial_PORT_CNF_SIZE * port) +
                                                                                sw_serial_PORT_CNF_DR_OFFSET;
                CY_SYS_PINS_SET_PIN(portDataRegAddr, pin);
                sw_serial_pinDrAdress = portDataRegAddr;
            #endif /* (!CY_PSOC4) */
            sw_serial_pinNumber = pin;
            sw_serial_pinPortNumber = port;
        }
    }
#endif /* (sw_serial_PIN_STATIC_MODE == 1u) */


/*******************************************************************************
* Function Name: sw_serial_Stop
********************************************************************************
*
* Summary:
*  Empty function. Included for consistency with other components.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void sw_serial_Stop(void) 
{

}


/*******************************************************************************
* Function Name: sw_serial_PutString
********************************************************************************
*
* Summary:
*  Sends a NULL terminated string via the Tx pin.
*
* Parameters:
*  string: Pointer to the null terminated string to send
*
* Return:
*  None
*
*******************************************************************************/
void sw_serial_PutString(const char8 string[]) 
{
    uint8 stringIndex = 1u;
    char8 current = *string;

    /* Until null is reached, print next character */
    while((char8) '\0' != current)
    {
        sw_serial_PutChar((uint8)current);
        current = string[stringIndex];
        stringIndex++;
    }
}


/*******************************************************************************
* Function Name: sw_serial_PutArray
********************************************************************************
*
* Summary:
*  Sends byteCount bytes from a memory array via the Tx pin.
*
* Parameters:
*  data: Pointer to the memory array
*  byteCount: Number of bytes to be transmitted
*
* Return:
*  None
*
*******************************************************************************/
void sw_serial_PutArray(const uint8 array[], uint32 byteCount) 
{
    uint32 arrayIndex;

    for (arrayIndex = 0u; arrayIndex < byteCount; arrayIndex++)
    {
        sw_serial_PutChar(array[arrayIndex]);
    }
}


/*******************************************************************************
* Function Name: sw_serial_PutHexByte
********************************************************************************
*
* Summary:
*  Sends a byte in Hex representation (two characters, uppercase for A-F) via
*  the Tx pin.
*
* Parameters:
*  TxHexByte: The byte to be converted to ASCII characters and
*             sent via the Tx pin.
*
* Return:
*  None
*
*******************************************************************************/
void sw_serial_PutHexByte(uint8 txHexByte) 
{
    static char8 const CYCODE sw_serial_hex[] = "0123456789ABCDEF";

    sw_serial_PutChar((uint8) sw_serial_hex[txHexByte >> sw_serial_BYTE_UPPER_NIBBLE_SHIFT]);
    sw_serial_PutChar((uint8) sw_serial_hex[txHexByte & sw_serial_BYTE_LOWER_NIBBLE_MASK]);
}


/*******************************************************************************
* Function Name: sw_serial_PutHexInt
********************************************************************************
*
* Summary:
*  Sends a 16-bit unsigned integer in Hex representation (four characters,
*  uppercase for A-F) via the Tx pin.
*
* Parameters:
*  TxHexInt: The uint16 to be converted to ASCII characters and sent via
*            the Tx pin.
*
* Return:
*  None
*
*******************************************************************************/
void sw_serial_PutHexInt(uint16 txHexInt) 
{
    sw_serial_PutHexByte((uint8)(txHexInt >> sw_serial_U16_UPPER_BYTE_SHIFT));
    sw_serial_PutHexByte((uint8)(txHexInt & sw_serial_U16_LOWER_BYTE_MASK));
}


/*******************************************************************************
* Function Name: sw_serial_PutCRLF
********************************************************************************
*
* Summary:
*  Sends a carriage return (0x0D) and a line feed (0x0A) via the Tx pin.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void sw_serial_PutCRLF(void) 
{
    sw_serial_PutChar(0x0Du);
    sw_serial_PutChar(0x0Au);
}


/* [] END OF FILE */
