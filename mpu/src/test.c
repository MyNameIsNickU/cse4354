/*
 * test.c
 *
 *  Created on: Oct 12, 2022
 *      Author: insti
 */


#include "test.h"
#include "rtos.h"
#include "board.h"

void sram_test(void)
{
    uint8_t * p = (uint8_t*)0x20007C00;
    bool doUnpriv = false;

    setSramAccessWindow((uint32_t)p, 1024);

    while(1)
    {
        if( isPressed(PB0) )
            doUnpriv = true;
        else if( isPressed(PB1) )
        {
            if(doUnpriv)
                setUnprivileged();
            *p = 100;
        }
        else if( isPressed(PB2) )
        {
            if(doUnpriv)
                setUnprivileged();
            *(p-4) = 100;
        }
        else if( isPressed(PB3) )
        {
            if(doUnpriv)
                setUnprivileged();
            *(p+1024) = 100;
        }

    }
}

void heap_test(void)
{
    //extern uint8_t * heap;
    uint8_t * p;
    uint8_t i;

    for(i = 0; i < 32; i++)
        p = malloc_from_heap(1024);
}
