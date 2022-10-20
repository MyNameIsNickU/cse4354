/*
 * board.c
 *
 *  Created on: Sep 16, 2022
 *      Author: insti
 */

#include <rtos_library.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "board.h"
#include "gpio.h"
#include "wait.h"
#include "utilities.h"


#define BLUE_LED   PORTF,2 // on-board blue LED
#define RED_LED    PORTE,0 // off-board red LED
#define ORANGE_LED PORTA,2 // off-board orange LED
#define YELLOW_LED PORTA,3 // off-board yellow LED
#define GREEN_LED  PORTA,4 // off-board green LED

#define ORANGE_PIN PORTA,2
#define ORANGE (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // PA2
#define RED_PIN PORTE,0
#define RED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // PE0
#define GREEN_PIN PORTA,4
#define GREEN  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // PA4
#define YELLOW_PIN PORTA,3
#define YELLOW (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // PA3

#define PB0_PIN PORTD,6
#define PB0_BB (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4))) // PD6
#define PB1_PIN PORTD,7
#define PB1_BB (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4))) // PD7
#define PB2_PIN PORTC,4
#define PB2_BB (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) // PC4
#define PB3_PIN PORTC,5
#define PB3_BB (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) // PC5
#define PB4_PIN PORTC,6
#define PB4_BB (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) // PC6
#define PB5_PIN PORTC,7
#define PB5_BB (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) // PC7

void initBoard(void)
{
    enablePort(PORTA);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

    // PORT F on-board LEDs
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(GREEN_LED);

    // Output LEDs
    selectPinPushPullOutput(ORANGE_PIN); // PA2
    selectPinPushPullOutput(RED_PIN); // PE0
    selectPinPushPullOutput(YELLOW_PIN); // PA3
    selectPinPushPullOutput(GREEN_PIN);  // PA4

    // Input Push Buttons
    enablePinPullup(PB0_PIN);
    selectPinDigitalInput(PB0_PIN); // PD6
    setPinCommitControl(PB1_PIN);
    enablePinPullup(PB1_PIN);
    selectPinDigitalInput(PB1_PIN); // PD7
    enablePinPullup(PB2_PIN);
    selectPinDigitalInput(PB2_PIN);
    enablePinPullup(PB3_PIN);
    selectPinDigitalInput(PB3_PIN);
    enablePinPullup(PB4_PIN);
    selectPinDigitalInput(PB4_PIN);
    enablePinPullup(PB5_PIN);
    selectPinDigitalInput(PB5_PIN);
}

void testLEDs(void)
{
    RED = 1;
    waitMicrosecond(500000);
    GREEN = 1;
    waitMicrosecond(500000);
    YELLOW = 1;
    waitMicrosecond(500000);
    ORANGE = 1;
    waitMicrosecond(500000);

    uint8_t i;
    for(i = 0; i < 7; i++)
    {
        RED ^= 1;
        GREEN ^= 1;
        YELLOW ^= 1;
        ORANGE ^= 1;
        waitMicrosecond(250000);
    }
}

uint8_t getButtons(void)
{
    uint8_t return_value = 0;

    if(PB0_BB)
    {
        return_value += PB0;
    }
    if(PB1_BB)
    {
        return_value += PB1;
    }
    if(PB2_BB)
    {
        return_value += PB2;
    }
    if(PB3_BB)
    {
        return_value += PB3;
    }
    if(PB4_BB)
    {
        return_value += PB4;
    }
    if(PB5_BB)
    {
        return_value += PB5;
    }

    return return_value;
}

bool isPressed(uint8_t mask)
{
    return (63 - getButtons()) == mask;
}

void testButtons(void)
{
    uint8_t i;
    for(i = 0; i < 120; i++)
    {
        emb_printf("Button Value: %b\n", getButtons());
        waitMicrosecond(250000);
    }
}

void buttonShell(void)
{
    uint32_t * bus_fault = (uint32_t*)0x30007FFF;
    uint8_t x = 5;
    uint8_t y = 0;
    uint8_t z;

    while(1)
    {
        if( isPressed(PB1) )
        {
            *bus_fault = 1;
        }

        else if( isPressed(PB0) )
        {
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }

        else if( isPressed(PB2) )
        {
            z = x/y;
            //return -1;
            //exit;
        }

        else if( isPressed(PB3) )
        {
            NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_MEM);
            __asm(" MOV R0, #1");
            __asm(" MOV R1, #2");
            __asm(" MOV R2, #3");
            __asm(" MOV R3, #4");
            *bus_fault = 1; // R0 is addr, R1 is value
        }

        else if( isPressed(PB4) )
        {

            uint32_t x;
            x = 123;
        }
    }

}
