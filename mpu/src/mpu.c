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
#include "test.h"

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



//TODO: check if data section is setup properly in .cmd file
//uint8_t heap[1024 * 28]; // 28 KiB Heap
uint8_t * heap = (uint8_t *)0x20001000;
//uint8_t * psp = (uint8_t*)0x20008000;
uint32_t * psp = (uint32_t*)0x20008000;

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


    // i think this needs to be divided out more
    //setPSP((uint32_t*)psp);
    setPSP(psp);
    setASP();

    initFaults();

    //buttonShell();

    setupBackgroundRegion();
    allowFlashAccess();
    setupSramAccess();
    allowPeripheralAccess();
    enableMPU();

    //heap_test();
    sram_test();

    //shell();

    while(1);
}
