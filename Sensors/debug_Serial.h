#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

void serial_begin();
void serial_print(char *string);
void serial_println(char *string);
void serial_printDouble(double num);
void serial_printDoubleln(double num);
void serial_printFloat(float num);
void serial_printFloatln(float num);
void serial_printInt(int num);
void serial_printIntln(int num);

#endif