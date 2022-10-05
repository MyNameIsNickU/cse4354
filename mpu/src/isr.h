/*
 * isr.h
 *
 *  Created on: Sep 22, 2022
 *      Author: insti
 */

#ifndef ISR_H_
#define ISR_H_

void initFaults(void);

void mpuFault(void);
void busFault(void);
void usageFault(void);
void hardFault(void);
void pendsvFault(void);

#endif /* ISR_H_ */
