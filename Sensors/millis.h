#ifndef MILLIS_H
#define MILLIS_H

#include <stdint.h>
    
/*Initializes the ISR vector*/
void millis_init();

/*returns the delta amount of time elapsed since the program has started*/
uint32_t millis();

#endif