#include "project.h"
#include "stdlib.h"
#include "math.h"
#include "millis.h"
#include <stdint.h>

static volatile uint32_t mscount = 0;

/* Declares the interrupt service routine. Every time 1 ms elapsed the mscount will count up 1
 * there are 8.64x10^7 = 86,400,000 milliseconds in a day
 * the mscount variable is 32 bit which means that it can represent 0 - 4,294,967,295 numbers
 * Which means that the counter will overflow and wrap around every 49.71 days which is way longer
 * than our purpose which means that we don't have to take care of the things that could be caused due to overflow
*/
CY_ISR(timerISR);

CY_ISR(timerISR){
    mscount++;    
    deltaTimer_STATUS;
}

void millis_init(){
    deltaTimer_Start(); //configure and enable the timer
    deltTime_StartEx(timerISR);
}

uint32_t millis(){
    return mscount;
}