/*
 * isr.c
 *
 *  Created on: Sep 22, 2022
 *      Author: insti
 */


#include "isr.h"
#include "utilities.h"
#include "tm4c123gh6pm.h"

extern uint32_t * getPSP(void);
extern uint32_t * getMSP(void);

void initFaults(void)
{
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_MEM;
    NVIC_CFG_CTRL_R |= NVIC_CFG_CTRL_DIV0;
}

void mpuFault(void)
{
    uint32_t * msp_pointer = getMSP();
    uint32_t * psp_pointer = getPSP();
    uint8_t mem_fault_bool = 0;

    emb_printf("MPU fault in process N\n");

    emb_printf(" MSP: 0x%x\n", msp_pointer);


    emb_printf(" PSP: 0x%x\n", psp_pointer);

    emb_printf("  R0: 0x%x\n", *psp_pointer);
    emb_printf("  R1: 0x%x\n", *(psp_pointer + 1));
    emb_printf("  R2: 0x%x\n", *(psp_pointer + 2));
    emb_printf("  R3: 0x%x\n", *(psp_pointer + 3));
    emb_printf(" R12: 0x%x\n", *(psp_pointer + 4));
    emb_printf("  LR: 0x%x\n", *(psp_pointer + 5));
    emb_printf("  PC: 0x%x\n", *(psp_pointer + 6));

    uint32_t bad_data_addr = NVIC_MM_ADDR_R;
    uint32_t bad_instruct = *(psp_pointer+6); // dereferncing PC
    uint32_t mfault_stat = NVIC_FAULT_STAT_R;

    // if MM Fault Address is valid
    mem_fault_bool = NVIC_FAULT_STAT_R && NVIC_FAULT_STAT_MMARV;
    if(mem_fault_bool)
	{
		bad_instruct = *(uint32_t*)bad_instruct;
        emb_printf("Bad Data Addr: 0x%X\n", bad_data_addr);
		emb_printf("Bad Instruct: 0x%X\n", bad_instruct);
	}
    
    emb_printf("MFault Flags: 0x%X\n", mfault_stat);

    // clears pending MPU fault
    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_MEMP;

    // turns on a pending SV call
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    //while(1) { }
}

void busFault(void)
{
    emb_printf("Bus fault in process N\n");
    while(1) { }
}

void usageFault(void)
{
    emb_printf("Usage fault in process N\n");

    while(1) { }
}

void hardFault(void)
{
    emb_printf("Hard fault in process N\n");

    uint32_t * msp_pointer = getMSP();
    emb_printf(" MSP: 0x%x\n", msp_pointer);

    uint32_t * psp_pointer = getPSP();
    emb_printf(" PSP: 0x%x\n", psp_pointer);

    /*emb_printf("  R0: 0x%x\n", *psp_pointer);
    emb_printf("  R1: 0x%x\n", *(psp_pointer + 1));
    emb_printf("  R2: 0x%x\n", *(psp_pointer + 2));
    emb_printf("  R3: 0x%x\n", *(psp_pointer + 3));
    emb_printf(" R12: 0x%x\n", *(psp_pointer + 4));
    emb_printf("  LR: 0x%x\n", *(psp_pointer + 5));
    emb_printf("  PC: 0x%x\n", *(psp_pointer + 6));*/
    emb_printf("xPSR: 0x%x\n", *(psp_pointer + 7));

    while(1) { }
}

void pendsvFault(void)
{
    emb_printf("Pendsv in process N\n");
    if(NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_DERR == NVIC_FAULT_STAT_DERR || NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_IERR == NVIC_FAULT_STAT_IERR)
        emb_printf("called from MPU\n");

    while(1) { }
    // If the MPU DERR or IERR bits are set
    // clear them and display the message “called from MPU”
}
