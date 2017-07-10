/*******************************************************************************
* File Name: serial_SCBCLK.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_serial_SCBCLK_H)
#define CY_CLOCK_serial_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void serial_SCBCLK_StartEx(uint32 alignClkDiv);
#define serial_SCBCLK_Start() \
    serial_SCBCLK_StartEx(serial_SCBCLK__PA_DIV_ID)

#else

void serial_SCBCLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void serial_SCBCLK_Stop(void);

void serial_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 serial_SCBCLK_GetDividerRegister(void);
uint8  serial_SCBCLK_GetFractionalDividerRegister(void);

#define serial_SCBCLK_Enable()                         serial_SCBCLK_Start()
#define serial_SCBCLK_Disable()                        serial_SCBCLK_Stop()
#define serial_SCBCLK_SetDividerRegister(clkDivider, reset)  \
    serial_SCBCLK_SetFractionalDividerRegister((clkDivider), 0u)
#define serial_SCBCLK_SetDivider(clkDivider)           serial_SCBCLK_SetDividerRegister((clkDivider), 1u)
#define serial_SCBCLK_SetDividerValue(clkDivider)      serial_SCBCLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define serial_SCBCLK_DIV_ID     serial_SCBCLK__DIV_ID

#define serial_SCBCLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define serial_SCBCLK_CTRL_REG   (*(reg32 *)serial_SCBCLK__CTRL_REGISTER)
#define serial_SCBCLK_DIV_REG    (*(reg32 *)serial_SCBCLK__DIV_REGISTER)

#define serial_SCBCLK_CMD_DIV_SHIFT          (0u)
#define serial_SCBCLK_CMD_PA_DIV_SHIFT       (8u)
#define serial_SCBCLK_CMD_DISABLE_SHIFT      (30u)
#define serial_SCBCLK_CMD_ENABLE_SHIFT       (31u)

#define serial_SCBCLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << serial_SCBCLK_CMD_DISABLE_SHIFT))
#define serial_SCBCLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << serial_SCBCLK_CMD_ENABLE_SHIFT))

#define serial_SCBCLK_DIV_FRAC_MASK  (0x000000F8u)
#define serial_SCBCLK_DIV_FRAC_SHIFT (3u)
#define serial_SCBCLK_DIV_INT_MASK   (0xFFFFFF00u)
#define serial_SCBCLK_DIV_INT_SHIFT  (8u)

#else 

#define serial_SCBCLK_DIV_REG        (*(reg32 *)serial_SCBCLK__REGISTER)
#define serial_SCBCLK_ENABLE_REG     serial_SCBCLK_DIV_REG
#define serial_SCBCLK_DIV_FRAC_MASK  serial_SCBCLK__FRAC_MASK
#define serial_SCBCLK_DIV_FRAC_SHIFT (16u)
#define serial_SCBCLK_DIV_INT_MASK   serial_SCBCLK__DIVIDER_MASK
#define serial_SCBCLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_serial_SCBCLK_H) */

/* [] END OF FILE */
