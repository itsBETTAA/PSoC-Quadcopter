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
#include <stripLights.h>
#include <stripLights.c>

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    StripLights_Start();
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    //uint32_t _col = 0;
    for(;;)
    {
        /* Place your application code here. */
        //StripLights_WriteColor(_col);
        //StripLights_DisplayClear(0b0010101001010111);
        StripLights_Pixel(2, 1, 0xF0F0);
        StripLights_Dim(4);
        //StripLights_Trigger(0);
    }
}

/* [] END OF FILE */
