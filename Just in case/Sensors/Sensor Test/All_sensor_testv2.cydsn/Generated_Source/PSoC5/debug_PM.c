/*******************************************************************************
* File Name: debug_PM.c
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

#include "debug.h"


/***************************************
* Local data allocation
***************************************/

static debug_BACKUP_STRUCT  debug_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: debug_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the debug_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debug_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debug_SaveConfig(void)
{
    #if(debug_CONTROL_REG_REMOVED == 0u)
        debug_backup.cr = debug_CONTROL_REG;
    #endif /* End debug_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: debug_RestoreConfig
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
*  debug_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling debug_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void debug_RestoreConfig(void)
{
    #if(debug_CONTROL_REG_REMOVED == 0u)
        debug_CONTROL_REG = debug_backup.cr;
    #endif /* End debug_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: debug_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The debug_Sleep() API saves the current component state. Then it
*  calls the debug_Stop() function and calls 
*  debug_SaveConfig() to save the hardware configuration.
*  Call the debug_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debug_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debug_Sleep(void)
{
    #if(debug_RX_ENABLED || debug_HD_ENABLED)
        if((debug_RXSTATUS_ACTL_REG  & debug_INT_ENABLE) != 0u)
        {
            debug_backup.enableState = 1u;
        }
        else
        {
            debug_backup.enableState = 0u;
        }
    #else
        if((debug_TXSTATUS_ACTL_REG  & debug_INT_ENABLE) !=0u)
        {
            debug_backup.enableState = 1u;
        }
        else
        {
            debug_backup.enableState = 0u;
        }
    #endif /* End debug_RX_ENABLED || debug_HD_ENABLED*/

    debug_Stop();
    debug_SaveConfig();
}


/*******************************************************************************
* Function Name: debug_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  debug_Sleep() was called. The debug_Wakeup() function
*  calls the debug_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  debug_Sleep() function was called, the debug_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debug_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debug_Wakeup(void)
{
    debug_RestoreConfig();
    #if( (debug_RX_ENABLED) || (debug_HD_ENABLED) )
        debug_ClearRxBuffer();
    #endif /* End (debug_RX_ENABLED) || (debug_HD_ENABLED) */
    #if(debug_TX_ENABLED || debug_HD_ENABLED)
        debug_ClearTxBuffer();
    #endif /* End debug_TX_ENABLED || debug_HD_ENABLED */

    if(debug_backup.enableState != 0u)
    {
        debug_Enable();
    }
}


/* [] END OF FILE */
