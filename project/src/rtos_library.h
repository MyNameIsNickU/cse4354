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

// assembly function defines
extern void setPSP(uint32_t * address);
extern void setASP(void);
extern void setUnprivileged(void);
extern void setPrivileged(void);
extern uint32_t * getPSP(void);
extern uint32_t * getMSP(void);

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
