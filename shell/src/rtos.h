/*
 * rtos.h
 *
 *  Created on: Aug 30, 2022
 *      Author: insti
 */

#ifndef _RTOS_H_
#define _RTOS_H_

#include <stdint.h>
#include <stdbool.h>

void ps(void);
void ipcs(void);
void kill(uint32_t pid);
void pmap(uint32_t pid);
void preempt(bool on);

#endif /* _RTOS_H_ */
