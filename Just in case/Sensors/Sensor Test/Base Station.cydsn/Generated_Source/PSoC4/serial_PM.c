/***************************************************************************//**
* \file serial_PM.c
* \version 3.20
*
* \brief
*  This file provides the source code to the Power Management support for
*  the SCB Component.
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

#include "serial.h"
#include "serial_PVT.h"

#if(serial_SCB_MODE_I2C_INC)
    #include "serial_I2C_PVT.h"
#endif /* (serial_SCB_MODE_I2C_INC) */

#if(serial_SCB_MODE_EZI2C_INC)
    #include "serial_EZI2C_PVT.h"
#endif /* (serial_SCB_MODE_EZI2C_INC) */

#if(serial_SCB_MODE_SPI_INC || serial_SCB_MODE_UART_INC)
    #include "serial_SPI_UART_PVT.h"
#endif /* (serial_SCB_MODE_SPI_INC || serial_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(serial_SCB_MODE_UNCONFIG_CONST_CFG || \
   (serial_SCB_MODE_I2C_CONST_CFG   && (!serial_I2C_WAKE_ENABLE_CONST))   || \
   (serial_SCB_MODE_EZI2C_CONST_CFG && (!serial_EZI2C_WAKE_ENABLE_CONST)) || \
   (serial_SCB_MODE_SPI_CONST_CFG   && (!serial_SPI_WAKE_ENABLE_CONST))   || \
   (serial_SCB_MODE_UART_CONST_CFG  && (!serial_UART_WAKE_ENABLE_CONST)))

    serial_BACKUP_STRUCT serial_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: serial_Sleep
****************************************************************************//**
*
*  Prepares the serial component to enter Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has an influence on this 
*  function implementation:
*  - Checked: configures the component to be wakeup source from Deep Sleep.
*  - Unchecked: stores the current component state (enabled or disabled) and 
*    disables the component. See SCB_Stop() function for details about component 
*    disabling.
*
*  Call the serial_Sleep() function before calling the 
*  CyPmSysDeepSleep() function. 
*  Refer to the PSoC Creator System Reference Guide for more information about 
*  power management functions and Low power section of this document for the 
*  selected mode.
*
*  This function should not be called before entering Sleep.
*
*******************************************************************************/
void serial_Sleep(void)
{
#if(serial_SCB_MODE_UNCONFIG_CONST_CFG)

    if(serial_SCB_WAKE_ENABLE_CHECK)
    {
        if(serial_SCB_MODE_I2C_RUNTM_CFG)
        {
            serial_I2CSaveConfig();
        }
        else if(serial_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            serial_EzI2CSaveConfig();
        }
    #if(!serial_CY_SCBIP_V1)
        else if(serial_SCB_MODE_SPI_RUNTM_CFG)
        {
            serial_SpiSaveConfig();
        }
        else if(serial_SCB_MODE_UART_RUNTM_CFG)
        {
            serial_UartSaveConfig();
        }
    #endif /* (!serial_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        serial_backup.enableState = (uint8) serial_GET_CTRL_ENABLED;

        if(0u != serial_backup.enableState)
        {
            serial_Stop();
        }
    }

#else

    #if (serial_SCB_MODE_I2C_CONST_CFG && serial_I2C_WAKE_ENABLE_CONST)
        serial_I2CSaveConfig();

    #elif (serial_SCB_MODE_EZI2C_CONST_CFG && serial_EZI2C_WAKE_ENABLE_CONST)
        serial_EzI2CSaveConfig();

    #elif (serial_SCB_MODE_SPI_CONST_CFG && serial_SPI_WAKE_ENABLE_CONST)
        serial_SpiSaveConfig();

    #elif (serial_SCB_MODE_UART_CONST_CFG && serial_UART_WAKE_ENABLE_CONST)
        serial_UartSaveConfig();

    #else

        serial_backup.enableState = (uint8) serial_GET_CTRL_ENABLED;

        if(0u != serial_backup.enableState)
        {
            serial_Stop();
        }

    #endif /* defined (serial_SCB_MODE_I2C_CONST_CFG) && (serial_I2C_WAKE_ENABLE_CONST) */

#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: serial_Wakeup
****************************************************************************//**
*
*  Prepares the serial component for Active mode operation after 
*  Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has influence on this 
*  function implementation:
*  - Checked: restores the component Active mode configuration.
*  - Unchecked: enables the component if it was enabled before enter Deep Sleep.
*
*  This function should not be called after exiting Sleep.
*
*  \sideeffect
*   Calling the serial_Wakeup() function without first calling the 
*   serial_Sleep() function may produce unexpected behavior.
*
*******************************************************************************/
void serial_Wakeup(void)
{
#if(serial_SCB_MODE_UNCONFIG_CONST_CFG)

    if(serial_SCB_WAKE_ENABLE_CHECK)
    {
        if(serial_SCB_MODE_I2C_RUNTM_CFG)
        {
            serial_I2CRestoreConfig();
        }
        else if(serial_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            serial_EzI2CRestoreConfig();
        }
    #if(!serial_CY_SCBIP_V1)
        else if(serial_SCB_MODE_SPI_RUNTM_CFG)
        {
            serial_SpiRestoreConfig();
        }
        else if(serial_SCB_MODE_UART_RUNTM_CFG)
        {
            serial_UartRestoreConfig();
        }
    #endif /* (!serial_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != serial_backup.enableState)
        {
            serial_Enable();
        }
    }

#else

    #if (serial_SCB_MODE_I2C_CONST_CFG  && serial_I2C_WAKE_ENABLE_CONST)
        serial_I2CRestoreConfig();

    #elif (serial_SCB_MODE_EZI2C_CONST_CFG && serial_EZI2C_WAKE_ENABLE_CONST)
        serial_EzI2CRestoreConfig();

    #elif (serial_SCB_MODE_SPI_CONST_CFG && serial_SPI_WAKE_ENABLE_CONST)
        serial_SpiRestoreConfig();

    #elif (serial_SCB_MODE_UART_CONST_CFG && serial_UART_WAKE_ENABLE_CONST)
        serial_UartRestoreConfig();

    #else

        if(0u != serial_backup.enableState)
        {
            serial_Enable();
        }

    #endif /* (serial_I2C_WAKE_ENABLE_CONST) */

#endif /* (serial_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
