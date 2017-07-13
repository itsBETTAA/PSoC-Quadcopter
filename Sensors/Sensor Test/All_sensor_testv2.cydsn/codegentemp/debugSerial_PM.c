/*******************************************************************************
* File Name: debugSerial_PM.c
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

#include "debugSerial.h"


/***************************************
* Local data allocation
***************************************/

static debugSerial_BACKUP_STRUCT  debugSerial_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: debugSerial_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the debugSerial_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debugSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debugSerial_SaveConfig(void)
{
    #if(debugSerial_CONTROL_REG_REMOVED == 0u)
        debugSerial_backup.cr = debugSerial_CONTROL_REG;
    #endif /* End debugSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: debugSerial_RestoreConfig
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
*  debugSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling debugSerial_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void debugSerial_RestoreConfig(void)
{
    #if(debugSerial_CONTROL_REG_REMOVED == 0u)
        debugSerial_CONTROL_REG = debugSerial_backup.cr;
    #endif /* End debugSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: debugSerial_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The debugSerial_Sleep() API saves the current component state. Then it
*  calls the debugSerial_Stop() function and calls 
*  debugSerial_SaveConfig() to save the hardware configuration.
*  Call the debugSerial_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debugSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debugSerial_Sleep(void)
{
    #if(debugSerial_RX_ENABLED || debugSerial_HD_ENABLED)
        if((debugSerial_RXSTATUS_ACTL_REG  & debugSerial_INT_ENABLE) != 0u)
        {
            debugSerial_backup.enableState = 1u;
        }
        else
        {
            debugSerial_backup.enableState = 0u;
        }
    #else
        if((debugSerial_TXSTATUS_ACTL_REG  & debugSerial_INT_ENABLE) !=0u)
        {
            debugSerial_backup.enableState = 1u;
        }
        else
        {
            debugSerial_backup.enableState = 0u;
        }
    #endif /* End debugSerial_RX_ENABLED || debugSerial_HD_ENABLED*/

    debugSerial_Stop();
    debugSerial_SaveConfig();
}


/*******************************************************************************
* Function Name: debugSerial_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  debugSerial_Sleep() was called. The debugSerial_Wakeup() function
*  calls the debugSerial_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  debugSerial_Sleep() function was called, the debugSerial_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debugSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debugSerial_Wakeup(void)
{
    debugSerial_RestoreConfig();
    #if( (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) )
        debugSerial_ClearRxBuffer();
    #endif /* End (debugSerial_RX_ENABLED) || (debugSerial_HD_ENABLED) */
    #if(debugSerial_TX_ENABLED || debugSerial_HD_ENABLED)
        debugSerial_ClearTxBuffer();
    #endif /* End debugSerial_TX_ENABLED || debugSerial_HD_ENABLED */

    if(debugSerial_backup.enableState != 0u)
    {
        debugSerial_Enable();
    }
}


/* [] END OF FILE */
