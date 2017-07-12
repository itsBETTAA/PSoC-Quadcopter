/*******************************************************************************
* File Name: xbeeSerial_PM.c
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

#include "xbeeSerial.h"


/***************************************
* Local data allocation
***************************************/

static xbeeSerial_BACKUP_STRUCT  xbeeSerial_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: xbeeSerial_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the xbeeSerial_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  xbeeSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void xbeeSerial_SaveConfig(void)
{
    #if(xbeeSerial_CONTROL_REG_REMOVED == 0u)
        xbeeSerial_backup.cr = xbeeSerial_CONTROL_REG;
    #endif /* End xbeeSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: xbeeSerial_RestoreConfig
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
*  xbeeSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling xbeeSerial_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void xbeeSerial_RestoreConfig(void)
{
    #if(xbeeSerial_CONTROL_REG_REMOVED == 0u)
        xbeeSerial_CONTROL_REG = xbeeSerial_backup.cr;
    #endif /* End xbeeSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: xbeeSerial_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The xbeeSerial_Sleep() API saves the current component state. Then it
*  calls the xbeeSerial_Stop() function and calls 
*  xbeeSerial_SaveConfig() to save the hardware configuration.
*  Call the xbeeSerial_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  xbeeSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void xbeeSerial_Sleep(void)
{
    #if(xbeeSerial_RX_ENABLED || xbeeSerial_HD_ENABLED)
        if((xbeeSerial_RXSTATUS_ACTL_REG  & xbeeSerial_INT_ENABLE) != 0u)
        {
            xbeeSerial_backup.enableState = 1u;
        }
        else
        {
            xbeeSerial_backup.enableState = 0u;
        }
    #else
        if((xbeeSerial_TXSTATUS_ACTL_REG  & xbeeSerial_INT_ENABLE) !=0u)
        {
            xbeeSerial_backup.enableState = 1u;
        }
        else
        {
            xbeeSerial_backup.enableState = 0u;
        }
    #endif /* End xbeeSerial_RX_ENABLED || xbeeSerial_HD_ENABLED*/

    xbeeSerial_Stop();
    xbeeSerial_SaveConfig();
}


/*******************************************************************************
* Function Name: xbeeSerial_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  xbeeSerial_Sleep() was called. The xbeeSerial_Wakeup() function
*  calls the xbeeSerial_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  xbeeSerial_Sleep() function was called, the xbeeSerial_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  xbeeSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void xbeeSerial_Wakeup(void)
{
    xbeeSerial_RestoreConfig();
    #if( (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) )
        xbeeSerial_ClearRxBuffer();
    #endif /* End (xbeeSerial_RX_ENABLED) || (xbeeSerial_HD_ENABLED) */
    #if(xbeeSerial_TX_ENABLED || xbeeSerial_HD_ENABLED)
        xbeeSerial_ClearTxBuffer();
    #endif /* End xbeeSerial_TX_ENABLED || xbeeSerial_HD_ENABLED */

    if(xbeeSerial_backup.enableState != 0u)
    {
        xbeeSerial_Enable();
    }
}


/* [] END OF FILE */
