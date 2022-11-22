/*
 * utilities.c
 *
 *  Created on: Sep 1, 2022
 *      Author: insti
 */

#include "utilities.h"
#include "uart0.h"

#define MAX_BUFFER_SIZE 50

// Reentrant safe (multi-threaded safe) printf function
// NOTE: does not handle negative integers
void emb_vprintf(const char * str, va_list vaArgP)
{
    uint32_t ui32Value, index, bufferPos, base, convertNeeded = 0;
    char buffer[MAX_BUFFER_SIZE];
    char *outputStr;

    // map for converting int to char based on index
    // can only print caps hex as of right now
    static const char * const map = "0123456789ABCDEF";

    // handles NULL case
    if(str[0] == '\0')
        return;

    //main loop
    // stops when the end of the string is reached
    while(*str != '\0')
    {
        // find first char that isn't '%' or '\0'
        for(index = 0; (str[index] != '%') && (str[index] != '\0'); index++) { }

        // handles normal strings and chars before '%' formatting
        putsUart0L(str, index);

        // skip over what was just printed (if anything)
        str += index;

        // deals with '%' formatting
        if(*str == '%')
        {
            // skip the '%'
            str++;

            switch(*str++)
            {
                // strings
                case 's':
                {
                    // gets the char *
                    outputStr = va_arg(vaArgP, char *);
                    // should already stop at the NULL character
                    putsUart0(outputStr);
                    //for(index = 0; outputStr[index] != '\0'; index++) { }
                    break;
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
                    //negNeeded = 0;
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
                    //negNeeded = 0;
                    // indicates that a conversion from int to string (in hex) is needed
                    convertNeeded = 1;
                    break;
                }
                // binary?
                case 'b':
                {
                    // pull the variable unsigned int value
                    ui32Value = va_arg(vaArgP, uint32_t);
                    // reset the writing buffer index to 0
                    bufferPos = 0;
                    // indicates base 16
                    base = 2;
                    // it is unsigned and cannot be negative
                    //negNeeded = 0;
                    // indicates that a conversion from int to string (in hex) is needed
                    convertNeeded = 1;
                    break;
                }
            }

            // handles flag for a conversion from an integer value to a string
            if(convertNeeded)
            {
                // counts the number of digits in the string based on the base
                for(index = 1; (((index * base) <= ui32Value) && (((index * base) / base) == index)); )
                {
                    index *= base;
                }

                // uses map to convert int into char version and put into buffer
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

// actual function that is called and handles the variable paramters
void emb_printf(const char * str, ...)
{
    // init the handler for the variable arguments
    va_list vaArgP;

    // start handling variable arguments after this parameter
    // str is the "last known fixed argument"
    va_start(vaArgP, str);

    // run the variable function that does the actual thing
    emb_vprintf(str, vaArgP);

    // stop handling variable arguments
    va_end(vaArgP);
}

// assumes length of dest can handle the source
void emb_strcpy(const char * source, char * dest)
{
    uint16_t index = 0;

    for(; source[index] != '\0'; index++)
    {
        dest[index] = source[index];
    }
    dest[index] = '\0';
}

void emb_memcpy(void *dest, const void *src, uint16_t bytes_to_copy)
{
    uint8_t *dest_B = (uint8_t*)dest, *src_B = (uint8_t*)src;
    while(bytes_to_copy != 0)
    {
        *dest_B = *(src_B++);
        dest_B++;
        bytes_to_copy--;
    }
}

uint32_t emb_strlen(const char * s)
{
    uint32_t count = 0;
    while(s[count] != '\0')
        count++;
    return count;
}

uint8_t emb_log2(uint64_t value)
{
    uint8_t count = 0;
    while(value > 1)
    {
        value >>= 1;
        count++;
    }
    return count;
}

uint32_t emb_pow2(int8_t value)
{
    int8_t loops = value;
    uint32_t return_value = 1;

    while(loops-- > 0)
        return_value *= 2;

    return return_value;
}
