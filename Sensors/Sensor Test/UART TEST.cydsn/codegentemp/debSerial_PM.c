/*******************************************************************************
* File Name: debSerial_PM.c
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

#include "debSerial.h"


/***************************************
* Local data allocation
***************************************/

static debSerial_BACKUP_STRUCT  debSerial_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: debSerial_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the debSerial_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debSerial_SaveConfig(void)
{
    #if(debSerial_CONTROL_REG_REMOVED == 0u)
        debSerial_backup.cr = debSerial_CONTROL_REG;
    #endif /* End debSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: debSerial_RestoreConfig
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
*  debSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling debSerial_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void debSerial_RestoreConfig(void)
{
    #if(debSerial_CONTROL_REG_REMOVED == 0u)
        debSerial_CONTROL_REG = debSerial_backup.cr;
    #endif /* End debSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: debSerial_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The debSerial_Sleep() API saves the current component state. Then it
*  calls the debSerial_Stop() function and calls 
*  debSerial_SaveConfig() to save the hardware configuration.
*  Call the debSerial_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debSerial_Sleep(void)
{
    #if(debSerial_RX_ENABLED || debSerial_HD_ENABLED)
        if((debSerial_RXSTATUS_ACTL_REG  & debSerial_INT_ENABLE) != 0u)
        {
            debSerial_backup.enableState = 1u;
        }
        else
        {
            debSerial_backup.enableState = 0u;
        }
    #else
        if((debSerial_TXSTATUS_ACTL_REG  & debSerial_INT_ENABLE) !=0u)
        {
            debSerial_backup.enableState = 1u;
        }
        else
        {
            debSerial_backup.enableState = 0u;
        }
    #endif /* End debSerial_RX_ENABLED || debSerial_HD_ENABLED*/

    debSerial_Stop();
    debSerial_SaveConfig();
}


/*******************************************************************************
* Function Name: debSerial_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  debSerial_Sleep() was called. The debSerial_Wakeup() function
*  calls the debSerial_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  debSerial_Sleep() function was called, the debSerial_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  debSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void debSerial_Wakeup(void)
{
    debSerial_RestoreConfig();
    #if( (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) )
        debSerial_ClearRxBuffer();
    #endif /* End (debSerial_RX_ENABLED) || (debSerial_HD_ENABLED) */
    #if(debSerial_TX_ENABLED || debSerial_HD_ENABLED)
        debSerial_ClearTxBuffer();
    #endif /* End debSerial_TX_ENABLED || debSerial_HD_ENABLED */

    if(debSerial_backup.enableState != 0u)
    {
        debSerial_Enable();
    }
}


/* [] END OF FILE */
