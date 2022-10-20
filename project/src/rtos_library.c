/*
 * rtos.c
 *
 *  Created on: Aug 30, 2022
 *      Author: Nicholas Untrecht
 */

#include <rtos_library.h>
#include "tm4c123gh6pm.h"
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
    putsUart0("PS Called\n");
}

void ipcs(void)
{
    putsUart0("IPCS Called\n");
}

void kill(uint32_t pid)
{
    emb_printf("%u Killed\n", pid);
}

void pmap(uint32_t pid)
{
    emb_printf("Memory usage by %u\n", pid);
}

void preempt(bool on)
{
    switch(on)
    {
    case true:
        putsUart0("preempt on\n");
        break;
    case false:
        putsUart0("preempt off\n");
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
        putsUart0("sched prio\n");
        break;
    case false:
        putsUart0("sched rr\n");
        break;
    default:
        break;
    }
}

void pidof(const char name[])
{
    emb_printf("%s launched\n", name);
}

void runProcess(const char name[])
{
    emb_printf("Running: %s\n", name);


}
