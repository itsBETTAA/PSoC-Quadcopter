/*******************************************************************************
* File Name: gpsSerial_PM.c
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

#include "gpsSerial.h"


/***************************************
* Local data allocation
***************************************/

static gpsSerial_BACKUP_STRUCT  gpsSerial_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: gpsSerial_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the gpsSerial_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  gpsSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void gpsSerial_SaveConfig(void)
{
    #if(gpsSerial_CONTROL_REG_REMOVED == 0u)
        gpsSerial_backup.cr = gpsSerial_CONTROL_REG;
    #endif /* End gpsSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: gpsSerial_RestoreConfig
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
*  gpsSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling gpsSerial_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void gpsSerial_RestoreConfig(void)
{
    #if(gpsSerial_CONTROL_REG_REMOVED == 0u)
        gpsSerial_CONTROL_REG = gpsSerial_backup.cr;
    #endif /* End gpsSerial_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: gpsSerial_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The gpsSerial_Sleep() API saves the current component state. Then it
*  calls the gpsSerial_Stop() function and calls 
*  gpsSerial_SaveConfig() to save the hardware configuration.
*  Call the gpsSerial_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  gpsSerial_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void gpsSerial_Sleep(void)
{
    #if(gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED)
        if((gpsSerial_RXSTATUS_ACTL_REG  & gpsSerial_INT_ENABLE) != 0u)
        {
            gpsSerial_backup.enableState = 1u;
        }
        else
        {
            gpsSerial_backup.enableState = 0u;
        }
    #else
        if((gpsSerial_TXSTATUS_ACTL_REG  & gpsSerial_INT_ENABLE) !=0u)
        {
            gpsSerial_backup.enableState = 1u;
        }
        else
        {
            gpsSerial_backup.enableState = 0u;
        }
    #endif /* End gpsSerial_RX_ENABLED || gpsSerial_HD_ENABLED*/

    gpsSerial_Stop();
    gpsSerial_SaveConfig();
}


/*******************************************************************************
* Function Name: gpsSerial_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  gpsSerial_Sleep() was called. The gpsSerial_Wakeup() function
*  calls the gpsSerial_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  gpsSerial_Sleep() function was called, the gpsSerial_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  gpsSerial_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void gpsSerial_Wakeup(void)
{
    gpsSerial_RestoreConfig();
    #if( (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) )
        gpsSerial_ClearRxBuffer();
    #endif /* End (gpsSerial_RX_ENABLED) || (gpsSerial_HD_ENABLED) */
    #if(gpsSerial_TX_ENABLED || gpsSerial_HD_ENABLED)
        gpsSerial_ClearTxBuffer();
    #endif /* End gpsSerial_TX_ENABLED || gpsSerial_HD_ENABLED */

    if(gpsSerial_backup.enableState != 0u)
    {
        gpsSerial_Enable();
    }
}


/* [] END OF FILE */
