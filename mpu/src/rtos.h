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

void reboot(void);
void ps(void);
void ipcs(void);
void kill(uint32_t pid);
void pmap(uint32_t pid);
void preempt(bool on);
void sched(bool prio_on);
void pidof(const char name[]);
void runProcess(const char name[]);

#endif /* _RTOS_H_ */
