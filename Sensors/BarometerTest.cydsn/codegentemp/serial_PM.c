/*******************************************************************************
* File Name: serial_PM.c
* Version 2.50
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "serial.h"


/***************************************
* Local data allocation
***************************************/

static serial_BACKUP_STRUCT  serial_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: serial_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the serial_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  serial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void serial_SaveConfig(void)
{
    #if(serial_CONTROL_REG_REMOVED == 0u)
        serial_backup.cr = serial_CONTROL_REG;
    #endif /* End serial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: serial_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the nonretention control register except FIFO.
*  Does not restore the FIFO which is a set of nonretention registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  serial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling serial_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void serial_RestoreConfig(void)
{
    #if(serial_CONTROL_REG_REMOVED == 0u)
        serial_CONTROL_REG = serial_backup.cr;
    #endif /* End serial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: serial_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The serial_Sleep() API saves the current component state. Then it
*  calls the serial_Stop() function and calls 
*  serial_SaveConfig() to save the hardware configuration.
*  Call the serial_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  serial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void serial_Sleep(void)
{
    #if(serial_RX_ENABLED || serial_HD_ENABLED)
        if((serial_RXSTATUS_ACTL_REG  & serial_INT_ENABLE) != 0u)
        {
            serial_backup.enableState = 1u;
        }
        else
        {
            serial_backup.enableState = 0u;
        }
    #else
        if((serial_TXSTATUS_ACTL_REG  & serial_INT_ENABLE) !=0u)
        {
            serial_backup.enableState = 1u;
        }
        else
        {
            serial_backup.enableState = 0u;
        }
    #endif /* End serial_RX_ENABLED || serial_HD_ENABLED*/

    serial_Stop();
    serial_SaveConfig();
}


/*******************************************************************************
* Function Name: serial_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  serial_Sleep() was called. The serial_Wakeup() function
*  calls the serial_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  serial_Sleep() function was called, the serial_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  serial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void serial_Wakeup(void)
{
    serial_RestoreConfig();
    #if( (serial_RX_ENABLED) || (serial_HD_ENABLED) )
        serial_ClearRxBuffer();
    #endif /* End (serial_RX_ENABLED) || (serial_HD_ENABLED) */
    #if(serial_TX_ENABLED || serial_HD_ENABLED)
        serial_ClearTxBuffer();
    #endif /* End serial_TX_ENABLED || serial_HD_ENABLED */

    if(serial_backup.enableState != 0u)
    {
        serial_Enable();
    }
}


/* [] END OF FILE */
