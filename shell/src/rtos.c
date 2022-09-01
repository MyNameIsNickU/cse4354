/*
 * rtos.c
 *
 *  Created on: Aug 30, 2022
 *      Author: insti
 */

#include "rtos.h"
#include "uart0.h"
#include "cmd.h"
#include "gpio.h"
#include "utilities.h"

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
    //char pid_str[20];
    //int_tostr(pid, pid_str);
    //putsUart0(pid_str);
    //putsUart0(" Killed");
    emb_printf("%u Killed", pid);
}

void pmap(uint32_t pid)
{
    /*char pid_str[20];
    int_tostr(pid, pid_str);
    putsUart0("Memory usage by ");
    putsUart0(pid_str);*/
    emb_printf("Memory usage by %u", pid);
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

void sched(bool prio_on)
{
    switch(prio_on)
    {
    case true:
        putsUart0("sched prio");
        break;
    case false:
        putsUart0("sched rr");
        break;
    default:
        break;
    }
}

void pidof(const char name[])
{
    putsUart0(name);
    putsUart0(" launched");
}

void runProg(const char name[])
{
    putsUart0("Running: ");
    putsUart0(name);

    if( strcomp(name, "red") )
        setPinValue(RED_LED,!getPinValue(RED_LED));
}
