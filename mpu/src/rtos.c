/*
 * rtos.c
 *
 *  Created on: Aug 30, 2022
 *      Author: insti
 */

#include "tm4c123gh6pm.h"
#include "rtos.h"
#include "uart0.h"
#include "cmd.h"
#include "gpio.h"
#include "utilities.h"

void reboot(void)
{
    // resets core and microcontroller
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ; // resets perphs and regs

    // just a core reset, does not reset perphs and regs
    //NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_VECT_RESET;
}

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
        putsUart0("preempt on");
        break;
    case false:
        putsUart0("preempt off");
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
    /*putsUart0(name);
    putsUart0(" launched");*/
    emb_printf("%s launched", name);
}

void runProcess(const char name[])
{
    /*putsUart0("Running: ");
    putsUart0(name);*/
    emb_printf("Running: %s", name);

    // when the user types 'run red', toggle the RED_LED
    if( strcomp(name, "red") )
        setPinValue(RED_LED,!getPinValue(RED_LED));
}
