#include <stdint.h>
#include "clock.h"
#include "gpio.h"
#include "cmd.h"
#include "uart0.h"
#include "wait.h"
#include "utilities.h"

void initHw()
{
    initSystemClockTo40Mhz();

    enablePort(PORTF);
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(GREEN_LED);

    initUart0();
    setUart0BaudRate(115200, 40e6);
}


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
    putsUart0("Initial setup complete!\n");


    //emb_printf("here is a long string: %s\there is an unsigned int: %u\there is a char: %c\n", "AYOOOOOOOOOOo", 135807128, '&');
    //emb_printf("\nhere is a hex value: %x\there is another: %x\n", 0xFFFF, 0x1234ABCD);

    //char buffer[MAX_CHARS + 1];

    shell();
}
