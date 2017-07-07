/*

*/
#include "project.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <debug_Serial.h>
#include <math.h>

char str[100];

void serial_begin(){
    debugSerial_Start();    //initialize the UART
    /*
    Description for PSoC datasheet:
        This is the preferred method to begin component operation.
        UART_Start() sets the initVar variable, calls the UART_Init()
        function, and then calls the UART_Enable() function.
    */
}

void serial_print(char *string)
{
    debugSerial_PutString(string);
}

void serial_println(char *string)
{
    debugSerial_PutString(string);
    debugSerial_PutCRLF(' ');
}

void serial_printDouble(double num){
    char str[20];

    sprintf(str,"%f", num); 
    serial_print(str);
}

void serial_printDoubleln(double num){
    char str[20];

    sprintf(str,"%f", num); 
    serial_println(str);
}

void serial_printFloat(float num){
    char str[20];

    sprintf(str,"%f", num); 
    serial_print(str);
}

void serial_printFloatln(float num){
    char str[20];

    sprintf(str,"%f", num); 
    serial_println(str);
}

void serial_printInt(int num){
    char buffer[20];
    serial_print(itoa((num),buffer,10));
}

void serial_printIntln(int num){
    char buffer[20];
    //The 10 means that the number will be represented in the decimal base
    //if we wanted it in binary we would've just specified it as 2
    //16 for hex and 8 for octal
    serial_println(itoa((num),buffer,10));
}
