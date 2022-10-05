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

#ifndef TIMER_H_
#define TIMER_H_

typedef void (*_callback)();

// check pg. 98 for bit-band section


// 0x42000000 is the start of the bit-band alias section
// the (0x400x.xxxx - 0x4000.0000) gets the offset of the specific periph.
typedef enum _TIMER
{
    TIMER0 = 0x42000000 + (0x40030000-0x40000000)*32,
    TIMER1 = 0x42000000 + (0x40031000-0x40000000)*32,
    TIMER2 = 0x42000000 + (0x40032000-0x40000000)*32,
    TIMER3 = 0x42000000 + (0x40033000-0x40000000)*32,
    TIMER4 = 0x42000000 + (0x40034000-0x40000000)*32,
    TIMER5 = 0x42000000 + (0x40035000-0x40000000)*32
} TIMER;

typedef enum _SUBTIMER
{
	A = 1,
	B = 2
} SUBTIMER;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void enableTimer(TIMER timer, SUBTIMER block);
void disableTimer(TIMER timer, SUBTIMER block);

void initTimer1(void);
void enableTimer1A(void);
void clearIntT1A(void);
void initTimer2(void);
//void tickIsr();
void initCountTimers(void);
uint32_t random32();

#endif
