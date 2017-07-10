/*******************************************************************************
* File Name: killSwitch_PM.c
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

#include "killSwitch.h"


/***************************************
* Local data allocation
***************************************/

static killSwitch_BACKUP_STRUCT  killSwitch_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: killSwitch_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the killSwitch_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  killSwitch_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void killSwitch_SaveConfig(void)
{
    #if(killSwitch_CONTROL_REG_REMOVED == 0u)
        killSwitch_backup.cr = killSwitch_CONTROL_REG;
    #endif /* End killSwitch_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: killSwitch_RestoreConfig
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
*  killSwitch_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling killSwitch_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void killSwitch_RestoreConfig(void)
{
    #if(killSwitch_CONTROL_REG_REMOVED == 0u)
        killSwitch_CONTROL_REG = killSwitch_backup.cr;
    #endif /* End killSwitch_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: killSwitch_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The killSwitch_Sleep() API saves the current component state. Then it
*  calls the killSwitch_Stop() function and calls 
*  killSwitch_SaveConfig() to save the hardware configuration.
*  Call the killSwitch_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  killSwitch_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void killSwitch_Sleep(void)
{
    #if(killSwitch_RX_ENABLED || killSwitch_HD_ENABLED)
        if((killSwitch_RXSTATUS_ACTL_REG  & killSwitch_INT_ENABLE) != 0u)
        {
            killSwitch_backup.enableState = 1u;
        }
        else
        {
            killSwitch_backup.enableState = 0u;
        }
    #else
        if((killSwitch_TXSTATUS_ACTL_REG  & killSwitch_INT_ENABLE) !=0u)
        {
            killSwitch_backup.enableState = 1u;
        }
        else
        {
            killSwitch_backup.enableState = 0u;
        }
    #endif /* End killSwitch_RX_ENABLED || killSwitch_HD_ENABLED*/

    killSwitch_Stop();
    killSwitch_SaveConfig();
}


/*******************************************************************************
* Function Name: killSwitch_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  killSwitch_Sleep() was called. The killSwitch_Wakeup() function
*  calls the killSwitch_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  killSwitch_Sleep() function was called, the killSwitch_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  killSwitch_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void killSwitch_Wakeup(void)
{
    killSwitch_RestoreConfig();
    #if( (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) )
        killSwitch_ClearRxBuffer();
    #endif /* End (killSwitch_RX_ENABLED) || (killSwitch_HD_ENABLED) */
    #if(killSwitch_TX_ENABLED || killSwitch_HD_ENABLED)
        killSwitch_ClearTxBuffer();
    #endif /* End killSwitch_TX_ENABLED || killSwitch_HD_ENABLED */

    if(killSwitch_backup.enableState != 0u)
    {
        killSwitch_Enable();
    }
}


/* [] END OF FILE */
