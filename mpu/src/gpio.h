// GPIO Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// GPIO APB ports A-F

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#include <stdbool.h>

// Enum values set to bitband address of bit 0 of the GPIO_PORTx_DATA_R register
typedef enum _PORT
{
    PORTA = 0x42000000 + (0x400043FC-0x40000000)*32,
    PORTB = 0x42000000 + (0x400053FC-0x40000000)*32,
    PORTC = 0x42000000 + (0x400063FC-0x40000000)*32,
    PORTD = 0x42000000 + (0x400073FC-0x40000000)*32,
    PORTE = 0x42000000 + (0x400243FC-0x40000000)*32,
    PORTF = 0x42000000 + (0x400253FC-0x40000000)*32
} PORT;

#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3

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

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void enablePort(PORT port);
void disablePort(PORT port);

void selectPinPushPullOutput(PORT port, uint8_t pin);
void selectPinOpenDrainOutput(PORT port, uint8_t pin);
void selectPinDigitalInput(PORT port, uint8_t pin);
void selectPinAnalogInput(PORT port, uint8_t pin);
void setPinCommitControl(PORT port, uint8_t pin);

void enablePinPullup(PORT port, uint8_t pin);
void disablePinPullup(PORT port, uint8_t pin);
void enablePinPulldown(PORT port, uint8_t pin);
void disablePinPulldown(PORT port, uint8_t pin);

void setPinAuxFunction(PORT port, uint8_t pin, uint32_t fn);

void selectPinInterruptRisingEdge(PORT port, uint8_t pin);
void selectPinInterruptFallingEdge(PORT port, uint8_t pin);
void selectPinInterruptBothEdges(PORT port, uint8_t pin);
void selectPinInterruptHighLevel(PORT port, uint8_t pin);
void selectPinInterruptLowLevel(PORT port, uint8_t pin);
void enablePinInterrupt(PORT port, uint8_t pin);
void disablePinInterrupt(PORT port, uint8_t pin);
void clearPinInterrupt(PORT port, uint8_t pin);

void setPinValue(PORT port, uint8_t pin, bool value);
bool getPinValue(PORT port, uint8_t pin);
void setPortValue(PORT port, uint8_t value);
uint8_t getPortValue(PORT port);

#endif
