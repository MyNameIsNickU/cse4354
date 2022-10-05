/*
 * rtos.c
 *
 *  Created on: Aug 30, 2022
 *      Author: Nicholas Untrecht
 */

#include "tm4c123gh6pm.h"
#include "rtos.h"
#include "uart0.h"
#include "cmd.h"
#include "gpio.h"
#include "utilities.h"

void * malloc_from_heap(uint32_t size_in_bytes)
{
    void * temp = NULL;
    extern uint8_t * heap;
    uint32_t size_to_allocate = 0;

    // rounds up to the nearest 1 KiB
    size_to_allocate = ((size_in_bytes + 1023) / 1024) * 1024;

    if(heap <= (uint8_t*)0x20008000)
    {
        temp = heap;
        heap += size_to_allocate;
    }

    emb_printf("Size wanted: %u\n", size_in_bytes);
    emb_printf("Size to allocate: %u\n", size_to_allocate);
    emb_printf("Address Allocated: %x\n", temp);
    emb_printf("New Heap Pointer: %x\n", heap);

    return temp;
}

#define REGION_SIZE 1024
#define REGION_SIZE_lg2 10

#define PERM_FA 0x3000000
#define PERM_PRIV 0x1000000

void setMPUFields(uint8_t region, uint32_t base_addr, uint32_t region_size)
{
    uint32_t log2_value = emb_log2(region_size);
    NVIC_MPU_NUMBER_R = region;
    NVIC_MPU_ATTR_R |= ((log2_value-1) << 1); // log2(size) - 1 shifted right one
    NVIC_MPU_ATTR_R &= ~NVIC_MPU_ATTR_TEX_M;
    NVIC_MPU_BASE_R |= (base_addr >> 5) << 5; // removes the bottom 5 bits, then shifts value back
}

void allowFlashAccess(void)
{
    setMPUFields(0, 0x00000000, 0x40000);
    NVIC_MPU_ATTR_R |= PERM_FA | NVIC_MPU_ATTR_CACHEABLE;
    NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_ENABLE;
}

void allowPeripheralAccess(void)
{
    setMPUFields(1, 0x40000000, 0x4000000);
	NVIC_MPU_ATTR_R |= PERM_FA | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_BUFFRABLE | NVIC_MPU_ATTR_XN;
	NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_ENABLE;
}

void setupSramAccess(void)
{
    uint8_t i, region;
    uint32_t sram_base_addr, region_size;
    region_size = 0x2000; // 8 KiB
    for(i = 1; i <= 4; i++)
    {
        sram_base_addr = 0x20000000 + (i-1)*region_size; // Starting at SRAM, each chunk 8 KiB
        region = 2 + (i-1); // regions 2-5
        setMPUFields(region, sram_base_addr, region_size);
        NVIC_MPU_ATTR_R |= PERM_PRIV | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_XN;
        NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_SRD_M; // disable all of the Subregions
        NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_ENABLE; // turn on Region
    }
}


// TODO: Change this, task/MPU will only be configured to one thing at any time.
// store what the SRD mask will be for all regions for each program/task.
void setSramAccessWindow(uint32_t base_addr, uint32_t size_in_bytes)
{
    uint32_t size_to_allocate = ((size_in_bytes + 1023) / 1024) * 1024; // rounds up to nearest 1 KiB
    uint8_t subregions_to_touch = size_to_allocate / 1024; // how many subregions needed for the size requested
    uint8_t start_region = (base_addr - 0x20000000) / 0x2000; // which of the 4 SRAM regions are we starting

    uint16_t srd_mask;

    start_region += 2; // where the SRAM regions start
    while(subregions_to_touch > 0)
    {
        NVIC_MPU_NUMBER_R = start_region;
        if(subregions_to_touch - 8 >= 0)
        {
            NVIC_MPU_ATTR_R &= ~(0xFF << 8);
            subregions_to_touch -= 8;
        }
        else
        {
            srd_mask = ((emb_pow2(subregions_to_touch) - 1) << 8);
            NVIC_MPU_ATTR_R &= ~srd_mask;
            subregions_to_touch = 0;
        }
        start_region += 1;
    }
}

void enableMPU(void)
{
    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE;
}

void initRTOS(void)
{
    //setPSP();
}

void reboot(void)
{
    // resets core and microcontroller
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ; // resets perphs and regs

    // just a core reset, does not reset perphs and regs
    //NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_VECT_RESET;
}

void ps(void)
{
    putsUart0("PS Called\n");
}

void ipcs(void)
{
    putsUart0("IPCS Called\n");
}

void kill(uint32_t pid)
{
    //char pid_str[20];
    //int_tostr(pid, pid_str);
    //putsUart0(pid_str);
    //putsUart0(" Killed");
    emb_printf("%u Killed\n", pid);
}

void pmap(uint32_t pid)
{
    /*char pid_str[20];
    int_tostr(pid, pid_str);
    putsUart0("Memory usage by ");
    putsUart0(pid_str);*/
    emb_printf("Memory usage by %u\n", pid);
}

void preempt(bool on)
{
    switch(on)
    {
    case true:
        putsUart0("preempt on\n");
        break;
    case false:
        putsUart0("preempt off\n");
        break;
    default:
        break;
    }
}

void sched(bool prio_on)
{
    switch(prio_on)
    {
    case true:
        putsUart0("sched prio\n");
        break;
    case false:
        putsUart0("sched rr\n");
        break;
    default:
        break;
    }
}

void pidof(const char name[])
{
    /*putsUart0(name);
    putsUart0(" launched");*/
    emb_printf("%s launched\n", name);
}

void runProcess(const char name[])
{
    /*putsUart0("Running: ");
    putsUart0(name);*/
    emb_printf("Running: %s\n", name);

    // when the user types 'run red', toggle the RED_LED
    if( strcomp(name, "red") )
        setPinValue(RED_LED,!getPinValue(RED_LED));
}
