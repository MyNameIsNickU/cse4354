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

#define REGION_SIZE 1024

#define PERM_FA 0x3000000
#define PERM_PRIV 0x1000000

void setMPUFields(uint8_t region, uint32_t base_addr, uint64_t region_size)
{
    uint32_t log2_value = emb_log2(region_size);
    NVIC_MPU_NUMBER_R = region;
    NVIC_MPU_BASE_R = (base_addr >> 5) << 5; // removes the bottom 5 bits, then shifts value back
    NVIC_MPU_ATTR_R |= ((log2_value-1) << 1); // log2(size) - 1 shifted left one
    NVIC_MPU_ATTR_R &= ~NVIC_MPU_ATTR_TEX_M;   
}

void setupBackgroundRegion(void)
{
    setMPUFields(0, 0x00000000, 0x100000000); // (0xFFFFFFFF + 1)
    NVIC_MPU_ATTR_R |= PERM_FA | NVIC_MPU_ATTR_XN;
    NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_ENABLE;
}

void allowFlashAccess(void)
{
    setMPUFields(1, 0x00000000, 0x40000);
    NVIC_MPU_ATTR_R |= PERM_FA | NVIC_MPU_ATTR_CACHEABLE;
    NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_ENABLE;
}

void allowPeripheralAccess(void)
{
    setMPUFields(2, 0x40000000, 0x4000000);
	NVIC_MPU_ATTR_R |= PERM_FA | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_BUFFRABLE | NVIC_MPU_ATTR_XN;
	NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_ENABLE;
}

#define SRAM_STARTING_REGION 3
void setupSramAccess(void)
{
    uint8_t i, region;
    uint32_t sram_base_addr, region_size;
    region_size = 0x2000; // 8 KiB
    for(i = 1; i <= 4; i++)
    {
        sram_base_addr = 0x20000000 + (i-1)*region_size; // Starting at SRAM, each chunk 8 KiB
        region = SRAM_STARTING_REGION + (i-1); // regions 3-6
        setMPUFields(region, sram_base_addr, region_size);
        NVIC_MPU_ATTR_R |= PERM_PRIV | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE | NVIC_MPU_ATTR_XN;
        //NVIC_MPU_ATTR_R &= ~NVIC_MPU_ATTR_SRD_M; // disable all of the Subregions
        NVIC_MPU_ATTR_R |= NVIC_MPU_ATTR_ENABLE; // turn on Region
    }
}


// TODO: Change this, task/MPU will only be configured to one thing at any time.
// store what the SRD mask will be for all regions for each program/task.
// 0x20000000
void setSramAccessWindow(uint32_t base_addr, uint32_t size_to_allocate)
{
    size_to_allocate = ((size_to_allocate + 1023) / 1024) * 1024; // rounds up to nearest 1 KiB
    int8_t subregions_to_touch = size_to_allocate >> 10; // how many subregions needed for the size requested
	uint8_t subregion_index = (base_addr - 0x20000000) >> 10;
	uint8_t start_region = ((base_addr - 0x20000000) >> 13) + SRAM_STARTING_REGION; // which of the 4 SRAM regions are we starting

    uint32_t srd_mask = 0;
	
	while(subregions_to_touch > 0)
	{
		srd_mask += (1 << subregion_index++);
		subregions_to_touch--;
	}

	emb_printf("SRD_Mask: 0x%X\n", srd_mask);

	uint32_t srd_area = 0x000000FF;
	subregions_to_touch = size_to_allocate >> 10;

	uint8_t i;
	for(i = 0; i <= SRAM_STARTING_REGION; i++)
	{
	    NVIC_MPU_NUMBER_R = SRAM_STARTING_REGION + i;
	    // get the 8 bits of SRD mask, shift them down by which 8bits it's in
	    // and then shift it left 8bits to get to SRD bits
        NVIC_MPU_ATTR_R |= (((srd_mask & srd_area) >> i*8) << 8);
        srd_area <<= 8;
	}
	//TODO: store srd_mask in TCB
}


// TODO: might return 0x20008000 as a valid value, THIS IS WRONG
void * malloc_from_heap(uint32_t size_in_bytes)
{
    void * temp = NULL;
    extern uint8_t * heap;
    uint32_t size_to_allocate = 0;

    // rounds up to the nearest 1 KiB
    size_to_allocate = ((size_in_bytes + 1023) / 1024) * 1024;

    if(heap < (uint8_t*)0x20008000)
    {
        temp = heap;
        heap += size_to_allocate;
    }

    setSramAccessWindow((uint32_t)temp, size_to_allocate);

    emb_printf("Size wanted: %u\n", size_in_bytes);
    emb_printf("Size to allocate: %u\n", size_to_allocate);
    emb_printf("Address Allocated: 0x%x\n", temp);
    emb_printf("New Heap Pointer: 0x%x\n", heap);

    return temp;
}

void enableMPU(void)
{
    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE;
}

void initRTOS(void)
{

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
    emb_printf("%u Killed\n", pid);
}

void pmap(uint32_t pid)
{
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
    emb_printf("%s launched\n", name);
}

void runProcess(const char name[])
{
    emb_printf("Running: %s\n", name);

    // when the user types 'run red', toggle the RED_LED
    if( strcomp(name, "red") )
        setPinValue(RED_LED,!getPinValue(RED_LED));
}
