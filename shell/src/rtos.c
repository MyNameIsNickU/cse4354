/*
 * rtos.c
 *
 *  Created on: Aug 30, 2022
 *      Author: insti
 */

#include "rtos.h"
#include "uart0.h"
#include "cmd.h"

void ps(void)
{
    putsUart0("PS Called");
}

void ipcs(void)
{
    putsUart0("IPCS Called");
}

void kill(uint32_t pid)
{
    char pid_str[20];
    int_tostr(pid, pid_str);
    putsUart0(pid_str);
    putsUart0(" Killed");
}

void pmap(uint32_t pid)
{
    char pid_str[20];
    int_tostr(pid, pid_str);
    putsUart0("Memory usage by ");
    putsUart0(pid_str);
}

void preempt(bool on)
{
    switch(on)
    {
    case true:
        putsUart0("preempt ON");
        break;
    case false:
        putsUart0("preempt OFF");
        break;
    default:
        break;
    }
}
