/*******************************************************************************
* File Name: deltaTimer_PM.c
* Version 2.70
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "deltaTimer.h"

static deltaTimer_backupStruct deltaTimer_backup;


/*******************************************************************************
* Function Name: deltaTimer_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  deltaTimer_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void deltaTimer_SaveConfig(void) 
{
    #if (!deltaTimer_UsingFixedFunction)
        deltaTimer_backup.TimerUdb = deltaTimer_ReadCounter();
        deltaTimer_backup.InterruptMaskValue = deltaTimer_STATUS_MASK;
        #if (deltaTimer_UsingHWCaptureCounter)
            deltaTimer_backup.TimerCaptureCounter = deltaTimer_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!deltaTimer_UDB_CONTROL_REG_REMOVED)
            deltaTimer_backup.TimerControlRegister = deltaTimer_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: deltaTimer_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  deltaTimer_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void deltaTimer_RestoreConfig(void) 
{   
    #if (!deltaTimer_UsingFixedFunction)

        deltaTimer_WriteCounter(deltaTimer_backup.TimerUdb);
        deltaTimer_STATUS_MASK =deltaTimer_backup.InterruptMaskValue;
        #if (deltaTimer_UsingHWCaptureCounter)
            deltaTimer_SetCaptureCount(deltaTimer_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!deltaTimer_UDB_CONTROL_REG_REMOVED)
            deltaTimer_WriteControlRegister(deltaTimer_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: deltaTimer_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  deltaTimer_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void deltaTimer_Sleep(void) 
{
    #if(!deltaTimer_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(deltaTimer_CTRL_ENABLE == (deltaTimer_CONTROL & deltaTimer_CTRL_ENABLE))
        {
            /* Timer is enabled */
            deltaTimer_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            deltaTimer_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    deltaTimer_Stop();
    deltaTimer_SaveConfig();
}


/*******************************************************************************
* Function Name: deltaTimer_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  deltaTimer_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void deltaTimer_Wakeup(void) 
{
    deltaTimer_RestoreConfig();
    #if(!deltaTimer_UDB_CONTROL_REG_REMOVED)
        if(deltaTimer_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                deltaTimer_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
