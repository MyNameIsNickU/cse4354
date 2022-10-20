/*
 * board.h
 *
 *  Created on: Sep 16, 2022
 *      Author: insti
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>

typedef enum _PB_WEIGHT
{
    PB5 = 1, PB4 = 2, PB3 = 4, PB2 = 8, PB1 = 16, PB0 = 32
} PB_WEIGHT;

void initBoard(void);

void buttonShell(void);

uint8_t getButtons(void);

void testLEDs(void);
void testButtons(void);
//uint8_t getButtons(void);
bool isPressed(uint8_t mask);

#endif /* BOARD_H_ */
