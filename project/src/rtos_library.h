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

void initSysTick(void);

// assembly function defines
extern void setPSP(uint32_t * address);
extern void setASP(void);
extern void setUnprivileged(void);
extern void setPrivileged(void);
extern uint32_t * getPSP(void);
extern uint32_t * getMSP(void);
extern void pushCore(void);
extern void popCore(void);
extern void setupUnrun(uint32_t function_pointer);
extern uint32_t extractR0(void);


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
