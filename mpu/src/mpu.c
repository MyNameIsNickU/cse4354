#include <stdint.h>
#include "clock.h"
#include "gpio.h"
#include "cmd.h"
#include "uart0.h"
#include "wait.h"
#include "utilities.h"
#include "timer.h"
#include "rtos.h"
#include "isr.h"
#include "board.h"

void initHw()
{
    //initSystemClockTo40Mhz();
    initSystemClockTo80Mhz();

    initBoard();

    initUart0();
    //setUart0BaudRate(115200, 80e6);
    setUart0BaudRate(460800, 80e6);

    initTimer1();
}

//#define TOGGLE_PIN (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*1))) // PE1

extern void setPSP(uint32_t * address);

extern void setASP(void);

extern void setUnprivileged(void);

extern void setPrivileged(void);

//TODO: check if data section is setup properly in .cmd file
//uint8_t heap[1024 * 28]; // 28 KiB Heap
uint8_t * heap = (uint8_t *)0x20001000;

int main()
{
    initHw();

    // Init startup flash
    uint8_t i;
    setPinValue(GREEN_LED, 0);
    for(i = 0; i < 6; i++)
    {
        setPinValue(GREEN_LED, !getPinValue(GREEN_LED));
        waitMicrosecond(100000);
    }
    emb_printf("Initial setup complete!\n");


    setPSP((uint32_t*)heap);
    setASP();

    initFaults();

    uint32_t varA = 25;
    uint32_t varB = 50;

    allowFlashAccess();
    allowPeripheralAccess();
    setupSramAccess();
    setSramAccessWindow(0x20000000, 0x8000);
    enableMPU();

    uint8_t * my_stack_pointer = malloc_from_heap(2079);

    //setUnprivileged();

    //setPrivileged();

    /*__asm("TEST_ADDR: .field 0x20000000");
    __asm(" LDR R0, TEST_ADDR");
    __asm(" STR R1, [R0]");*/
	
    //enableTimer(TIMER1,A);

    shell();
    //buttonShell();

    while(1);
}
