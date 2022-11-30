/*
 * rtos.c
 *
 *  Created on: Aug 30, 2022
 *      Author: Nicholas Untrecht
 */

#include "rtos_library.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "cmd.h"
#include "gpio.h"
#include "utilities.h"

#define SYSTICK_CTRL_R (*((volatile uint32_t*)0xE000E000 + 4))
#define SYSTICK_LOAD_R (*((volatile uint32_t*)0xE000E000 + 5))
#define SYSTICK_VAL_R  (*((volatile uint32_t*)0xE000E000 + 6))

void initSysTick(void)
{
	// datasheet says for 100 ticks, put value of 99
	
    NVIC_ST_RELOAD_R = 0x0;

    NVIC_ST_RELOAD_R = 39999;
    // reset value is not 1 for clock source, must set the bit
    // TI lied!
	NVIC_ST_CTRL_R |= (NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN);
}

#define WTIMER_CFG_32_BIT TIMER_CFG_16_BIT
void initWTimer1(void)
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1; // Wide Timer 1
    _delay_cycles(3);

    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    WTIMER1_CFG_R = WTIMER_CFG_32_BIT; // sets WTimer to 32b mode
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR; // count time and count up from 0
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG; // count time between negative edges
    WTIMER1_TAV_R = 0;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void initWTimer2(void)
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2; // Wide Timer 1
    _delay_cycles(3);

    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    WTIMER2_CFG_R = WTIMER_CFG_32_BIT; // sets WTimer to 32b mode
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR; // count time and count up from 0
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_NEG; // count time between negative edges
    WTIMER2_TAV_R = 0;
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;
}
/*void ps(void)
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
}*/
