// Timer Service Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Timer 4

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "timer.h"
#include "nvic.h"
#include "gpio.h"

// Offsets from General Purpose Timer (GPTM) to its register 
#define OFS_CFG_TO_TAMR 1*4*8
#define OFS_CFG_TO_TBMR 2*4*8

#define OFS_CFG_TO_CTL  3*4*8
#define OFS_CTL_TO_TAEN 0
#define OFS_CTL_TO_TBEN 8

#define OFS_CFG_TO_IMR  5*4*8
#define OFS_CFG_TO_ICR  8*4*8
#define OFS_CFG_TO_TAILR 10*4*8
#define OFS_CFG_TO_TBILR 11*4*8

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void enableTimerModule(TIMER timer)
{
	switch(timer)
	{
		case TIMER0:
		{
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
			break;
		}
		case TIMER1:
		{
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
			break;
		}
		case TIMER2:
		{
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
			break;
		}
		case TIMER3:
		{
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
			break;
		}
		case TIMER4:
		{
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;
			break;
		}
		case TIMER5:
		{
			SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R5;
			break;
		}
	}
	_delay_cycles(3);
}

void enableTimer(TIMER timer, SUBTIMER block)
{
	volatile uint32_t *p;
	if(block == A)
	{
		p = (uint32_t*)timer + OFS_CFG_TO_CTL + OFS_CTL_TO_TAEN;
		*p = 1;
	}
	else if(block == B)
	{
		p = (uint32_t*)timer + OFS_CFG_TO_CTL + OFS_CTL_TO_TBEN;
		*p = 1;
	}
}

void disableTimer(TIMER timer, SUBTIMER block)
{
	volatile uint32_t *p;
	if(block == A)
	{
		p = (uint32_t*)timer + OFS_CFG_TO_CTL + OFS_CTL_TO_TAEN;
		*p = 0;
	}
	else if(block == B)
	{
		p = (uint32_t*)timer + OFS_CFG_TO_CTL + OFS_CTL_TO_TBEN;
		*p = 0;
	}
}

// DOESNT WORK
void loadIntervalTimer(TIMER timer, SUBTIMER block, uint32_t value)
{
	volatile uint32_t *p;
	if(block == A)
	{
		p = (uint32_t*)timer + OFS_CFG_TO_TAILR;
		*p = value;
	}
	else if(block == B)
	{
		p = (uint32_t*)timer + OFS_CFG_TO_TBILR;
		*p = value;
	}
}

void initTimer1()
{
    // Enable clocks
    //SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
	enableTimerModule(TIMER1);
	
    // Configure Timer 4 for 1 sec tick
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 80000000;                       // 1 Hz interrupt rate, called every 1 second
	//loadIntervalTimer(TIMER1,A,80000000);
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;                // turn-on interrupt
    enableNvicInterrupt(INT_TIMER1A);             // turn-on interrupt 86 (TIMER1A)
}

void enableTimer1A()
{
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void clearIntT1A(void)
{
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void initTimer2()
{
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    _delay_cycles(3);
	
	TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    //TIMER2_TAILR_R = 40000;                          // set load value (1000 Hz rate)
    TIMER2_TAILR_R = 20000000;                          // set load value (2 Hz rate)
    TIMER2_IMR_R |= TIMER_IMR_TATOIM;                // turn-on interrupt
    enableNvicInterrupt(INT_TIMER2A);
}

void initCountTimers()
{
    enableTimerModule(TIMER3);

    TIMER3_CTL_R &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
    TIMER3_CFG_R = TIMER_CFG_16_BIT;
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER3_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
    TIMER3_TAILR_R = 40; // 40 MHz / 40 = 1 MHz
    TIMER3_TBILR_R = 40; // 40 MHz / 40 = 1 MHz
    TIMER3_IMR_R |= (TIMER_IMR_TATOIM | TIMER_IMR_TBTOIM);
    enableNvicInterrupt(INT_TIMER3A);
    enableNvicInterrupt(INT_TIMER3B);
}

void timer1Atick(void)
{
    //setPinValue(BLUE_LED, !getPinValue(BLUE_LED));
    clearIntT1A();
}

// Placeholder random number function
uint32_t random32()
{
    return TIMER4_TAV_R;
}
