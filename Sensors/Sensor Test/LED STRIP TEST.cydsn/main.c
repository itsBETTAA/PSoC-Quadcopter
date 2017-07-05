/* =======================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdint.h>

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    StripLights_Start();
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    uint32_t _col = 0;
    for(;;)
    {
        /* Place your application code here. */
        StripLights_WriteColor(_col);
        LED_Write(~ LED_Read());
        _col += 10;
        CyDelay(100);
    }
}

/* [] END OF FILE */
