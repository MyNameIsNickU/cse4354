/*
 * utilities.c
 *
 *  Created on: Sep 1, 2022
 *      Author: insti
 */

#include "utilities.h"
#include "uart0.h"

#define MAX_BUFFER_SIZE 255

void emb_vprintf(const char * str, va_list vaArgP)
{
    uint32_t ui32Value, index, bufferPos, base, negNeeded = 0, convertNeeded = 0;
    char buffer[MAX_BUFFER_SIZE];
    char *outputStr;

    // map for converting int to char based on index
    // can only print caps hex as of right now
    static const char * const map = "0123456789ABCDEF";

    if(str[0] == '\0')
        return;

    //main loop
    while(*str != '\0')
    {
        // find first char that isn't '%' or '\0'
        for(index = 0; (str[index] != '%') && (str[index] != '\0'); index++) { }

        // handles normal strings and ones before '%' formatting
        putsUart0L(str, index);

        // skip over what was just printed
        str += index;

        // deals with '%' formatting
        if(*str == '%')
        {
            // skip the '%'
            str++;

            switch(*str++)
            {
                // strings
                //TODO
                case 's':
                {

                }
                // chars
                case 'c':
                {
                    // pulls the variable char variable
                    ui32Value = va_arg(vaArgP, uint32_t);
                    putcUart0((char)ui32Value);
                    break;
                }
                // unsigned int
                case 'u':
                {
                    // pull the variable unsigned int value
                    ui32Value = va_arg(vaArgP, uint32_t);
                    // reset the writing buffer index to 0
                    bufferPos = 0;
                    // indicates base 10
                    base = 10;
                    // it is unsigned and cannot be negative
                    negNeeded = 0;
                    // indicates that a conversion from int to string is needed
                    convertNeeded = 1;
                    break;
                }
                // hex
                // both cases will print upper-case
                case 'X':
                case 'x':
                {
                    // pull the variable unsigned int value
                    ui32Value = va_arg(vaArgP, uint32_t);
                    // reset the writing buffer index to 0
                    bufferPos = 0;
                    // indicates base 16
                    base = 16;
                    // it is unsigned and cannot be negative
                    negNeeded = 0;
                    // indicates that a conversion from int to string (in hex) is needed
                    convertNeeded = 1;
                    break;
                }
            }

            if(convertNeeded)
            {
                // counts the number of digits in the string based on the base
                for(index = 1; (((index * base) <= ui32Value) && (((index * base) / base) == index)); )
                {
                    index *= base;
                }

                // uses map to convert int into string and put into buffer
                for(; index; index /= base)
                {
                    buffer[bufferPos++] = map[(ui32Value / index) % base];
                }

                putsUart0L(buffer, bufferPos);

                convertNeeded = 0;
            }
        }
    }
}

void emb_printf(const char * str, ...)
{
    va_list vaArgP;

    va_start(vaArgP, str);

    emb_vprintf(str, vaArgP);

    va_end(vaArgP);
}
