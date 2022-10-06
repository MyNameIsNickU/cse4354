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


void * malloc_from_heap(uint32_t size_in_bytes);

void initRTOS(void);

void setupBackgroundRegion(void);
void allowFlashAccess(void);
void allowPeripheralAccess(void);
void setupSramAccess(void);

void setSramAccessWindow(uint32_t base_addr, uint32_t size_to_allocate);

//void setMPUFields(uint8_t region, uint32_t base_addr, uint32_t region_size);
void enableMPU(void);

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
