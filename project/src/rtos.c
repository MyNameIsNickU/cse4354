// RTOS Framework - Fall 2022
// J Losh

// Student Name:
// TO DO: Add your name(s) on this line.
//        Do not include your ID number(s) in the file.

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PE0 (lengthy and general)
// Orange: PA2 (idle)
// Yellow: PA3 (oneshot and general)
// Green:  PA4 (flash4hz)
// PBs on these pins
// PB0:    PD6 (set red, toggle yellow)
// PB1:    PD7 (clear red, post flash_request semaphore)
// PB2:    PC4 (restart flash4hz)
// PB3:    PC5 (stop flash4hz, uncoop)
// PB4:    PC6 (lengthy priority increase)
// PB5:    PC7 (errant)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Memory Protection Unit (MPU):
//   Region to allow peripheral access (RW) or a general all memory access (RW)
//   Region to allow flash access (XRW)
//   Regions to allow 32 1KiB SRAM access (RW or none)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "cmd.h"
#include "wait.h"
#include "utilities.h"
#include "board.h"
#include "rtos_library.h"

#define BLUE_LED   PORTF,2 // on-board blue LED
#define RED_LED    PORTE,0 // off-board red LED
#define ORANGE_LED PORTA,2 // off-board orange LED
#define YELLOW_LED PORTA,3 // off-board yellow LED
#define GREEN_LED  PORTA,4 // off-board green LED

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

bool preemptionActive = true;
bool priorityActive = true;

enum _svcNum{
    SVC_RESTART = 66,
    SVC_STOP = 86,
    SVC_SET_TASK_PRIO = 33,
    SVC_YIELD = 21,
    SVC_SLEEP = 32,
    SVC_WAIT = 43,
    SVC_POST = 54,
    SVC_MALLOC = 13,
    SVC_KERNEL_DATA = 97,
    SVC_REBOOT = 101,
    SVC_PREEMP = 112,
    SVC_SCHED = 123
};

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16]; // name of semaphore
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_KILLED     5 // has run, but now has been killed

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

#define MIN_PRIORITY     7 // the lowest priority a thread can have

uint8_t * heap_bottom = (uint8_t *)0x20001000;
uint8_t * heap_top = (uint8_t *)0x20008000;
//uint8_t * psp = (uint8_t*)0x20008000;

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    uint16_t stackSize;            // size of original stack allocated
    int8_t priority;               // 0=highest to 7=lowest
    int8_t priorityInit;           // Store the original priority here
    uint32_t ticks;                // ticks until sleep complete
    uint32_t time_ping;            // for CPU time for 'ps' command
    uint32_t time_pong;            // for CPU time for 'ps' command
    uint32_t srd;                  // MPU subregion disable bits (one per 1 KiB)
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS] = {0};

bool wrPing_rdPong = true;
uint32_t time_running_sum = 0;
//-----------------------------------------------------------------------------
// Memory Manager and MPU Funcitons
//-----------------------------------------------------------------------------

#define SRAM_STARTING_REGION 3

uint32_t getSRD(uint32_t base_addr, uint32_t size_to_allocate)
{
    size_to_allocate = ((size_to_allocate + 1023) / 1024) * 1024; // rounds up to nearest 1 KiB
    int8_t subregions_to_touch = size_to_allocate >> 10; // how many subregions needed for the size requested
    // add one to the base addr to get to the 1K divisible boundary
    uint8_t subregion_index = (base_addr+1 - 0x20000000) >> 10;
    //uint8_t start_region = ((base_addr - 0x20000000) >> 13) + SRAM_STARTING_REGION; // which of the 4 SRAM regions are we starting

    uint32_t srd_mask = 0;

    while(subregions_to_touch > 0)
    {
        srd_mask += (1 << subregion_index++);
        subregions_to_touch--;
    }
#ifdef DEBUG
    emb_printf("SRD_Mask: 0x%X\n", srd_mask);
#endif

    return srd_mask;
}

void setSRD(uint32_t srd_mask)
{
    uint32_t srd_area = 0x000000FF;
    static uint8_t i;

    for(i = 0; i <= 3; i++)
    {
        NVIC_MPU_NUMBER_R = SRAM_STARTING_REGION + i;
        // get the 8 bits of SRD mask, shift them down by which 8bits it's in
        // and then shift it left 8bits to get to SRD bits region of the ATTR register
        NVIC_MPU_ATTR_R |= (((srd_mask & srd_area) >> i*8) << 8);
        srd_area <<= 8;
    }
}

void * mallocKernal(uint32_t size_in_bytes)
{
    void * temp = NULL;

    // ASSUME: should already be rounded up

    if(heap_bottom+size_in_bytes < heap_top)
    {
        temp = heap_bottom;
        heap_bottom += size_in_bytes;
    }

    //setSRD((uint32_t)temp, size_to_allocate);

    //emb_printf("Size wanted: %u\n", size_in_bytes);
#ifdef DEBUG
    emb_printf("Size to allocate: %u\n", size_in_bytes);
    emb_printf("Address Allocated: 0x%x\n", temp);
    emb_printf("New Heap Pointer: 0x%x\n", heap_bottom);
#endif

    return temp;
}

void * svcMalloc(uint32_t size_in_bytes)
{

    // ASSUME: should already be rounded up

    if(heap_top-size_in_bytes > (uint8_t*)heap_bottom)
    {
        heap_top -= size_in_bytes;
    }

    //setSRD((uint32_t)temp, size_to_allocate);

    //emb_printf("Size wanted: %u\n", size_in_bytes);
#ifdef DEBUG
    emb_printf("Size to allocate: %u\n", size_in_bytes);
    emb_printf("Address Allocated: 0x%x\n", temp);
    emb_printf("New Heap Pointer: 0x%x\n", heap_bottom);
#endif

    return heap_top;
}

// NOTE: this is just a SVC call
// svcMalloc is what actually runs now
void * mallocFromHeap(uint32_t size_in_bytes)
{
    __asm(" SVC #13");
    // how do I return something here to satisfy compiler?

}

#define REGION_SIZE 1024

#define PERM_FA 0x3000000
#define PERM_PRIV 0x1000000


void setMPUFields(uint8_t region, uint32_t base_addr, uint64_t region_size)
{
    uint32_t log2_value = emb_log2(region_size);
    NVIC_MPU_NUMBER_R = region;
    NVIC_MPU_BASE_R = (base_addr >> 5) << 5; // removes the bottom 5 bits, then shifts value back
    NVIC_MPU_ATTR_R |= ((log2_value-1) << 1); // ( log2(size) - 1 ) shifted left one
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

void initMpu(void)
{
    setupBackgroundRegion();
    allowFlashAccess();
    allowPeripheralAccess();
    setupSramAccess();

    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE; // enable MPU
}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void initRtos()
{
    // SysTick set to 1 kHz or 1ms
    initSysTick();
    initWTimer1();
    initWTimer2();

    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

int rtosScheduler()
{
    static uint8_t task = 0xFF;
    static int8_t lastTaskRan[MIN_PRIORITY + 1] = {0};
    uint8_t prio_i, task_i, loop_count = 0;
    bool foundTask = false, ready2run_l = false;

    // TODO: check this
    // we have to keep track of each task run for each priority separately?
    // N number of circular buffer layers for each priority?
    // ...seems like a lot, will take more time, but prio 0 will run quickly still
    // should just loop around once and find lowest prio but keep track of index
    // this is very complicated apparently
    if(priorityActive)
    {
        // loops for each priority value
        for(prio_i = 0; prio_i <= MIN_PRIORITY && !foundTask; prio_i++)
        {
            loop_count = 0;
            // goes through each task and find outer loop prio
            // start at the last task ran + 1, will either find same prio task or circularly loop around back to last task
            // has to go through each of the tasks to go to the next lowest priority
            for(task_i = (lastTaskRan[prio_i] + 1) % (MAX_TASKS); loop_count != MAX_TASKS/*task_i < MAX_TASKS*/; task_i=(task_i+1)%MAX_TASKS, loop_count++)
            {
                ready2run_l = tcb[task_i].state == STATE_READY || tcb[task_i].state == STATE_UNRUN;
                if( tcb[task_i].priority == prio_i && ready2run_l )
                {
                    lastTaskRan[prio_i] = task_i;
                    task = task_i;
                    foundTask = true;
                    break;
                }
            }
        }
    }
    else
    {
        while (!foundTask)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            foundTask = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }

    //putcUart0(task + 48);
    return task;
}

// could also stuff the PSP stack here when we create the thread
// ...for the case when the thread hasn't been run yet
bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    stackBytes = ((stackBytes + 1023) / 1024) * 1024;
    bool ok = false;
    uint8_t i = 0;
    bool found = false;

    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}

            // store the thread name
            emb_strcpy(name, tcb[i].name);

            // init state of task to UNRUN but ready to run
            tcb[i].state = STATE_UNRUN;
            // pid is the function address
            tcb[i].pid = fn;
            // allocate stack space and store top of stack in sp and spInit
            // -1 as our allocated space is (base -> top - 1) e.g. 0x2000 + 0x27FF, we don't get access to 0x2800
            tcb[i].sp = ((uint8_t*)mallocKernal(stackBytes) + stackBytes - 1);
            tcb[i].spInit = tcb[i].sp;
            tcb[i].stackSize = stackBytes;
            tcb[i].priorityInit = priority;
            tcb[i].priority = priority;
            // calculates SRD mask based off of initial base address and size of stack requested
            tcb[i].srd = getSRD( (uint32_t)((uint8_t*)tcb[i].spInit - stackBytes), stackBytes);
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    // ????
    return ok;
}

void restartThread(_fn fn)
{
    __asm(" SVC #66");
}

// NOTE: see notes in class for strategies on whether stack is freed or not
void stopThread(_fn fn)
{
    __asm(" SVC #86");
}

void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm(" SVC #33");
}

bool createSemaphore(uint8_t semaphore, uint8_t count, const char name[])
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    if(ok)
    {
        semaphores[semaphore].count = count;
        emb_strcpy(name, semaphores[semaphore].name);
    }
    return ok;
}

// calls scheduler, sets PSP, ASP bit, TMPL bit, and PC
void startRtos()
{
    // get index of task to run from tcb
    taskCurrent = rtosScheduler();

    // update MPU based on the Subregion Disabled bits
    setSRD(tcb[taskCurrent].srd);
    // set SP of task to malloc'd memory
    setPSP(tcb[taskCurrent].spInit);
    // use PSP
    setASP();

    // NOTE: this is defined on the new PSP so...
    // ...the task will always lose 4 bytes of data in its stack
    // for this method
    // load the function call into variable
    _fn task = (_fn)tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;


    // turn on systick timer
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;

    // must access the tcb struct before getting set to unpriv
    setUnprivileged();

    // set the PC to the beginning of the given process
    //((_fn)tcb[taskCurrent].pid)();
    task();
}

void yield()
{
    // indicates a needed service call for service "21"
    // the svCallIsr will handle the yield service
    __asm(" SVC #21");
}

// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    // indicates a needed service call for service "32"
    // the svCallIsr will handle the sleep service
    __asm(" SVC #32");
}

void wait(int8_t semaphore)
{
    // indicates a needed service call for service "43"
    // the svCallIsr will handle the wait for semaphore service
    __asm(" SVC #43");
}

void post(int8_t semaphore)
{
    // indicates a needed service call for service "54"
    // the svCallIsr will handle the post semaphore service
    __asm(" SVC #54");
}

void reboot(void)
{
    __asm(" SVC #101");
}

void setPreemption(bool value)
{
    __asm(" SVC #112");
}

void setPriority(bool value)
{
    __asm(" SVC #123");
}

enum _shell_cmd
{
    SHELL_PMAP = 2,
    SHELL_PIDOF = 3,
    SHELL_IPCS = 4,
    SHELL_PS = 5
};

typedef struct _SHELL_PMAP_S
{
    bool found;
    uint32_t stackTop;
    uint32_t stackBottom;
    uint32_t stackSize;
    uint32_t heapBottom;
    uint32_t heapTop;
    uint32_t heapSize;
    uint32_t srd;
    uint32_t heap_srd;
    bool usedMalloc;
} SHELL_PMAP_S;

typedef struct _SHELL_PS_S
{
    bool valid;
    uint32_t time;
    char name[16];
    uint8_t state;
    uint16_t k_time;
    uint32_t pid;
} SHELL_PS_S;

// type will be the type of data requested
// data_space will be a pointer to the space that data will be returned
void getKernelData(uint8_t type, void * data_space)
{
    __asm(" SVC #97");
}


#define INTEGRATION_T_MS 2000

void systickIsr()
{
    uint8_t i;
    static uint16_t ms_passed = 0;
    ms_passed++;

	// clear pending systick
	NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PENDSTCLR;

	// do I search the entire tcb for a sleeping task?
	// yes, i do
	for(i = 0; i < MAX_TASKS; i++)
	{
	    if(ms_passed == INTEGRATION_T_MS)
	    {
	        if(!wrPing_rdPong)
	            tcb[i].time_ping = 0;
	        else
	            tcb[i].time_pong = 0;
	    }

	    if(tcb[i].state == STATE_DELAYED)
	    {
	        if(tcb[i].ticks == 0)
	            tcb[i].state = STATE_READY;
	        else
	            tcb[i].ticks--;
	    }
	}

	if(ms_passed == INTEGRATION_T_MS)
	{
	    time_running_sum = WTIMER2_TAV_R;
        // clear out timers for CPU time in tcb
        WTIMER1_TAV_R = 0;
        //WTIMER1_CTL_R |= TIMER_CTL_TAEN;
        WTIMER2_TAV_R = 0;
        //WTIMER2_CTL_R |= TIMER_CTL_TAEN;
        if(wrPing_rdPong)
            wrPing_rdPong = false;
        else
            wrPing_rdPong = true;
        ms_passed = 0;
        //putcUart0('p');
	}

	// if preemption is active, trigger a task switch
	if(preemptionActive)
	    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// this interrupt cannot be called during another interrupt
// can only be called from Thread mode
// if called from an interrupt, the interrupt left will never finish

// TODO: add timers to count the amount of time spent running threads and/or task switching
void pendSvIsr()
{
    // stop CPU Timers
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    // aren't the core resgisters already on the PSP at this point by h/w?
    // yes, but R4-R11 aren't
    pushCore();
    uint32_t *psp = getPSP();

    // save the PSP
    tcb[taskCurrent].sp = psp;

    if(wrPing_rdPong)
    {
        /*putcUart0('w');
        putcUart0('p');
        putcUart0('i');*/
        tcb[taskCurrent].time_ping += WTIMER1_TAV_R;
    }
    else
    {
        /*putcUart0('w');
        putcUart0('p');
        putcUart0('o');*/
        tcb[taskCurrent].time_pong += WTIMER1_TAV_R;
    }

    // get a new task
    taskCurrent = rtosScheduler();

	// if task state is ready, then restore the last known PSP value and pop the core registers
    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSP( (uint32_t*)tcb[taskCurrent].sp );
        // pop the registers from this new PSP
        popCore();
    }
	// if task state is unrun, set PSP to initial value and load core registers with default values
    else if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        // Set psp to initial value
        setPSP( (uint32_t*)tcb[taskCurrent].spInit );

		// this should load in all R4-R11
        // similar to popCore, but loads in set defined values
        setupUnrun( (uint32_t)tcb[taskCurrent].pid );

        // set the state ready
        tcb[taskCurrent].state = STATE_READY;
    }

    // update the MPU with the prev stored SRD mask bits
    setSRD(tcb[taskCurrent].srd);

	// clear pending sv flag
	NVIC_INT_CTRL_R |= NVIC_INT_CTRL_UNPEND_SV;
    //while(1) { }
	WTIMER1_TAV_R = 0;
	WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void svCallIsr()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    // get the PSP in order to get pushed value of PC
    uint32_t *psp = getPSP();

    // get R0
    // check if there is a quicker way to do this...there was -_-
    uint32_t r0_value = *psp;
    uint32_t r1_value = *(psp+1);


    // to get the specific service requested...
    // gets the PC from PSP
    // thumb instructions are 16 b, so cast as 16b pointer
    // PC points at the NEXT instruction to execute
    // so go back one instruction to get the SVC instruction
    // the lower 8b are the number used in the SVC.
    uint16_t sv_instruct_value = *(((uint16_t*)(*(psp+6))) - 1);
    uint8_t sv_num = sv_instruct_value & 0xFF;

    semaphore *sem;

    uint8_t i,j;
    uint32_t allocated_memory;

    char * arg;
    uint32_t arg_int;

    SHELL_PMAP_S *pmap_data;
    SHELL_PS_S *ps_data;

    bool found = false;

    static uint32_t sum_task_time = 0;

    switch(sv_num)
    {
	// yield
    case SVC_YIELD:
        // set bit for pending SV call to trigger task switch
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
	// sleep
    case SVC_SLEEP:
        // set initial value of ticks to what was passed into R0
        tcb[taskCurrent].ticks = r0_value;

        // state is now delayed due to waiting for sleep to complete
        tcb[taskCurrent].state = STATE_DELAYED;

        // i don't think this will work. the codes runs for a little bit before switching
        // set bit for pending SV call to trigger task switch
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
	// wait
    case SVC_WAIT:

		// if the semaphore count is 0, then the task must wait until the semp. posts
        // this is when a task goes inside the queue
        // don't put it in queue if queue size has hit max
		if(semaphores[r0_value].count == 0 && semaphores[r0_value].queueSize < MAX_QUEUE_SIZE )
		{
		    // load task with the semaphore address
            tcb[taskCurrent].semaphore = &semaphores[r0_value];
            // put the task index in the semaphores queue and increment the queue size (index)
            semaphores[r0_value].processQueue[semaphores[r0_value].queueSize++] = taskCurrent;
		    // task is now waiting on a semaphore to post
		    tcb[taskCurrent].state = STATE_BLOCKED;

		    // set bit for pending SV call to trigger task switch
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
		}
		else
		    semaphores[r0_value].count--;
        break;
    //post
    case SVC_POST:

        // increment the semaphore value
        semaphores[r0_value].count++;

        // the semaphore went from 0 -> 1, this means a process was likely waiting on it
        // there also must be something in the queue to clear
        if(semaphores[r0_value].count == 1 && semaphores[r0_value].queueSize > 0)
        {
            tcb[semaphores[r0_value].processQueue[0]].state = STATE_READY;
            tcb[semaphores[r0_value].processQueue[0]].semaphore = NULL;

            // if a task was waiting on this semaphore, it then takes the value just incremented for itself
            semaphores[r0_value].count--;

            // When do I remove a task from the queue?
           // do I leave it in there when I post, only removing when the task is destroyed?
           // NO, remove the task from the queue
            //shift the process queue over left one
            for(i = 0; i < semaphores[r0_value].queueSize - 1; i++ )
            {
                semaphores[r0_value].processQueue[i] = semaphores[r0_value].processQueue[i+1];
            }
            semaphores[r0_value].queueSize--;


            // TODO: task switch to the task that is now able to run
        }
        break;
    case SVC_RESTART:
        for(i = 0; i < MAX_TASKS; i++)
        {
            // found the requested thread
            if((uint32_t)tcb[i].pid == r0_value && (tcb[i].state == STATE_KILLED) )
            {
                tcb[i].priority = tcb[i].priorityInit;
                tcb[i].sp = tcb[i].spInit;
                tcb[i].state = STATE_UNRUN;
            }
        }
        break;
    case SVC_STOP:
        for(i = 0; i < MAX_TASKS; i++)
        {
            // found the requested thread
            if((uint32_t)tcb[i].pid == r0_value)
            {
                tcb[i].state = STATE_KILLED;

                // TODO: actually free the malloc memory


                // this will clear out the SRD and just allocate for the thread's stack size
                tcb[i].srd = 0;
                tcb[i].srd = getSRD((uint32_t)tcb[i].spInit + 1 - tcb[i].stackSize, tcb[i].stackSize);

                sem = (semaphore*)tcb[i].semaphore;
                // if the process is waiting on a semaphore
                // ...then remove that task from the process queue
                // ...and shift the process queue over
                if( sem != NULL && sem->queueSize != 0 )
                {
                    // find the task inside the process queue
                    for(j = 0; j < sem->queueSize - 1 && i != sem->processQueue[j]; j++) { }
                    // shift the queue from the found task
                    for(; j < sem->queueSize - 1; j++)
                        sem->processQueue[j] = sem->processQueue[j+1];
                    // decrement the size of the queue
                    sem->queueSize--;
                }
            }
        }
        break;
    // void setThreadPriority(_fn fn, uint8_t priority)
    case SVC_SET_TASK_PRIO:
        for(i = 0; i < MAX_TASKS; i++)
        {
            // found the requested thread
            if((uint32_t)tcb[i].pid == r0_value)
            {
                tcb[i].priority = r1_value;
            }
        }
        break;
    case SVC_MALLOC:
        // allocate function, basically what malloc from heap does now
        allocated_memory = (uint32_t)svcMalloc(r0_value);
        // *psp = void * from malloc
        // this will store the void * at R0
        *psp = allocated_memory;
        // update SRD mask with the newly allocated memory
        tcb[taskCurrent].srd |= getSRD(allocated_memory, r0_value);
        setSRD(tcb[taskCurrent].srd);
        break;
    case SVC_REBOOT:
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        break;
    case SVC_PREEMP:
        preemptionActive = r0_value;
        break;
    case SVC_SCHED:
        priorityActive = r0_value;
        break;
    // handles getting kernel data for output for shell
    case SVC_KERNEL_DATA:
        switch(r0_value)
        {
        // 'pidof TASK_NAME'
        // output: PID
        case SHELL_PIDOF:
            arg = (char*)(r1_value); // arg should have string name
            for(i = 0; i < MAX_TASKS && !found; i++)
            {
                if(strcomp(arg, tcb[i].name))
                {
                    emb_memcpy((void*)r1_value, &tcb[i].pid, sizeof(uint32_t));
//                    break;
                    found = true;
                }
            }

            if(!found)
                emb_memcpy((void*)r1_value, "NOTFOUND", 9);
            // now arg should have the
            break;
        case SHELL_IPCS:
            i = *(uint8_t*)(r1_value); // what index
            emb_memcpy((void*)r1_value, &semaphores[i], sizeof(semaphore));
            break;
        case SHELL_PMAP:
            arg_int = *(uint32_t*)(r1_value); // arg should have PID #
            for(i = 0; i < MAX_TASKS; i++)
            {
                if(arg_int == (uint32_t)tcb[i].pid)
                    break;
            }
            pmap_data = (SHELL_PMAP_S*)(r1_value);
            if(i == MAX_TASKS)
            {
                pmap_data->found = false;
                break;
            }

            pmap_data->found = true;
            pmap_data->srd = tcb[i].srd;
            pmap_data->stackTop = (uint32_t)tcb[i].spInit;
            pmap_data->stackSize = tcb[i].stackSize;
            pmap_data->stackBottom = pmap_data->stackTop + 1 - tcb[i].stackSize;
            pmap_data->heapBottom = 0;
            pmap_data->heapTop = 0;
            pmap_data->heapSize = 0;
            arg_int = tcb[i].srd - getSRD(pmap_data->stackBottom, pmap_data->stackSize);

            if(arg_int > 0)
            {
                pmap_data->usedMalloc = true;
                pmap_data->heapBottom = 0x20000000;
                while((arg_int & 1) == 0)
                {
                    pmap_data->heapBottom += 1024;
                    arg_int >>= 1;
                }
                pmap_data->heapTop = pmap_data->heapBottom + 1024 - 1;
                pmap_data->heapSize = 1024;

                arg_int >>= 1;
                while((arg_int & 1) == 1)
                {
                    pmap_data->heapTop += 1024;
                    pmap_data->heapSize += 1024;
                    arg_int >>= 1;
                }
            }
            else
                pmap_data->usedMalloc = false;
            break;
        case SHELL_PS:
            i = *(uint8_t*)(r1_value); // what index
            ps_data = (SHELL_PS_S*)(r1_value);

            // requested kernel time
            if(i == MAX_TASKS)
            {
                ps_data->k_time = (INTEGRATION_T_MS * 40000-sum_task_time) / (INTEGRATION_T_MS * 40000 / 10000);
                ps_data->valid = true;
                sum_task_time = 0;
                break;
            }

            ps_data->valid = tcb[i].state != 0;
            if(!ps_data->valid)
                break;

            emb_strcpy(tcb[i].name, ps_data->name);
            ps_data->state = tcb[i].state;
            ps_data->pid = (uint32_t)tcb[i].pid;
            if(wrPing_rdPong)
            {
                //emb_printf("Pong[%u]: %u\n", i, tcb[i].time_pong);
                ps_data->time = tcb[i].time_pong / (INTEGRATION_T_MS * 40000 / 10000);
                sum_task_time += tcb[i].time_pong;
            }
            else
            {
                //emb_printf("Ping[%u]: %u\n", i, tcb[i].time_ping);
                //ps_data->time = tcb[i].time_ping / (time_running_sum / 10000);
                ps_data->time = tcb[i].time_ping / (INTEGRATION_T_MS * 40000 / 10000);
                sum_task_time += tcb[i].time_ping;
            }

            break;
        }
    }

    // Restart task timers
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void mpuFaultIsr()
{
    uint32_t * msp_pointer = getMSP();
    uint32_t * psp_pointer = getPSP();
    uint8_t mem_fault_bool = NVIC_FAULT_STAT_R && NVIC_FAULT_STAT_MMARV;

    emb_printf("MPU fault in process %u\n", tcb[taskCurrent].pid);

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
    if(mem_fault_bool)
    {
        bad_instruct = *(uint32_t*)bad_instruct;
        emb_printf("Bad Data Addr: 0x%X\n", bad_data_addr);
        emb_printf("Bad Instruct: 0x%X\n", bad_instruct);
    }

    emb_printf("MFault Flags: 0x%X\n", mfault_stat);

    // if called from MPU fault, kill the current thread
    if(NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_DERR || NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_IERR)
    {

        //stopThread((_fn)tcb[taskCurrent].pid);
        semaphore *sem;
        uint8_t j;

        NVIC_FAULT_STAT_R |= NVIC_FAULT_STAT_DERR | NVIC_FAULT_STAT_IERR;
        //emb_printf("...called from MPU\n");
        tcb[taskCurrent].state = STATE_KILLED;
//
//        //TODO: clear out SRD bits
//
        // remove task from process queue
        sem = (semaphore*)tcb[taskCurrent].semaphore;
        // if the process is waiting on a semaphore
        // ...then remove that task from the process queue
        if( sem != NULL && sem->queueSize != 0 )
        {
            for(j = 0; j < sem->queueSize - 1; j++)
                sem->processQueue[j] = sem->processQueue[j+1];
            sem->queueSize--;
        }

    }

    // clears pending MPU fault
    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_MEMP;

    // turns on a pending SV call
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    //while(1) { }
}

void hardFaultIsr()
{
    emb_printf("Hard fault in process %u\n", taskCurrent);

    uint32_t * msp_pointer = getMSP();
    emb_printf(" MSP: 0x%x\n", msp_pointer);

    uint32_t * psp_pointer = getPSP();
    emb_printf(" PSP: 0x%x\n", psp_pointer);

    emb_printf("xPSR: 0x%x\n", *(psp_pointer + 7));

    while(1) { }
}

void busFaultIsr()
{
    emb_printf("Bus fault in process %u\n", taskCurrent);
    while(1) { }
}

void usageFaultIsr()
{
    emb_printf("Usage fault in process %u\n", taskCurrent);
    while(1) { }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // init all LEDs and PBs
    initBoard();

    // initialize the faults
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_MEM;
    // trap on divide by 0
    //NVIC_CFG_CTRL_R |= NVIC_CFG_CTRL_DIV0;
}

uint8_t readPbs()
{
    return getButtons();
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{

    __asm(" MOV R4, #1");
    __asm(" MOV R5, #2");
    __asm(" MOV R6, #3");
    __asm(" MOV R7, #4");
    __asm(" MOV R8, #5");
    __asm(" MOV R9, #6");
    __asm(" MOV R10, #7");
    __asm(" MOV R11, #8");

    while(true)
    {
        //putcUart0('\n');
        //putcUart0('i');
        //putcUart0('o');
        setPinValue(ORANGE_LED, 1);
        waitMicrosecond(1000);
        setPinValue(ORANGE_LED, 0);
        //waitMicrosecond(1000000);
        yield();
    }
}

void idle_another()
{
    __asm(" MOV R4, #8");
    __asm(" MOV R5, #7");
    __asm(" MOV R6, #6");
    __asm(" MOV R7, #5");
    __asm(" MOV R8, #4");
    __asm(" MOV R9, #3");
    __asm(" MOV R10, #2");
    __asm(" MOV R11, #1");

    while(true)
    {
        //putcUart0('\n');
        //putcUart0('i');
        //putcUart0('b');
        setPinValue(BLUE_LED, 1);
        waitMicrosecond(1000);
        setPinValue(BLUE_LED, 0);
        //waitMicrosecond(1000000);
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        setPinValue(GREEN_LED, !getPinValue(GREEN_LED));
        sleep(125);
    }
}

void flash2Hz()
{
    while(true)
    {
        setPinValue(RED_LED, !getPinValue(RED_LED));
        sleep(250);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        setPinValue(YELLOW_LED, 1);
        sleep(1000);
        setPinValue(YELLOW_LED, 0);
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    uint8_t *p;

    // Example of allocating memory from stack
    // This will show up in the pmap command for this thread

    p = mallocFromHeap(1024);
    *p = 0;

    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        setPinValue(RED_LED, !getPinValue(RED_LED));
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            setPinValue(YELLOW_LED, !getPinValue(YELLOW_LED));
            setPinValue(RED_LED, 1);
        }
        if ((buttons & 2) != 0)
        {
            //putcUart0('f');
            post(flashReq);
            setPinValue(RED_LED, 0);
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            stopThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        setPinValue(BLUE_LED, 1);
        sleep(1000);
        setPinValue(BLUE_LED, 0);
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
//inline void shell() could name my shell somehting else and call it here
// this copies the contents of the inline function to the locatino of the function call

// could give svcCall a pointer and ask it to fill it out with needed info
// ...this is BAD

// svcCall for getData(type, &data)
// test to make sure that data svcCall is filling in is inside the range of the stack

// make structs for all of the commands needed

// good to have functions for getting # of tasks or # of semaphores

// use different svc nums for preempt, sched, reboot
void shell()
{
    USER_DATA data;
    SHELL_PMAP_S *pmap_data;
    SHELL_PS_S *ps_data;
    void * kernel_data_ptr = mallocFromHeap(2048);
    bool valid_input = false;
    int8_t valid_integer = -1;
    uint8_t i, j;
    char *taskName, *arg_str;
    uint32_t arg_int = 0;

    putcUart0('\n');
    putcUart0('>');
    while (true)
    {
        getsUart0(&data);
        parseFields(&data);

        // always pass data, "what string do you want to check", how many args
        if( isCommand(&data, "help", 0) )
        {
            putsUart0("\nPossible commands:\n");
            // commands go here
            putsUart0("'preemp [ON|OFF]'\n");
            putsUart0("'sched [PRIO|RR]'\n");
            putsUart0("'pidof [NAME]'\n");
            putsUart0("'pmap [PID#]'\n");
            putsUart0("'ipcs'\n");
            putsUart0("'ps'\n");
            putsUart0("'run [NAME]'\n");
            putsUart0("'kill [PID#]'\n");
            putsUart0("'reboot'\n");
            putsUart0("'help' (this command :0)\n");
            putcUart0('\n');
            valid_input = true;
        }

        else if( isCommand(&data, "reboot", 0) )
        {
            reboot();
            valid_input = true;
        }

        else if( isCommand(&data, "clear", 0) || isCommand(&data, "cls", 0))
        {
            for(i = 0; i < 50; i++)
                putcUart0('\n');
            valid_input = true;
        }

        // Process Status
        // show PID, array index in the tcb array
        //time running would be cool
        else if( isCommand(&data, "ps", 0) )
        {
            putsUart0("\nNAME\t\t\tPID\t\tTIME\n");

            for(i = 0; i < MAX_TASKS; i++)
            {
                emb_memcpy(kernel_data_ptr, &i, 1); // give svc the index
                getKernelData(SHELL_PS, kernel_data_ptr);
                ps_data = (SHELL_PS_S*)kernel_data_ptr;
                if(ps_data->valid)
                {
                    if(emb_strlen(ps_data->name) <= 7)
                        emb_printf("%s\t\t\t%u\t\t%u.%u %%\n", ps_data->name, ps_data->pid, ps_data->time/100, ps_data->time%100);
                    else
                        emb_printf("%s\t\t%u\t\t%u.%u %%\n", ps_data->name, ps_data->pid, ps_data->time/100, ps_data->time%100);
                }
            }
            // i is now = to MAX_TASKS
            emb_memcpy(kernel_data_ptr, &i, 1); // give svc the index
            getKernelData(SHELL_PS, kernel_data_ptr);
            if(ps_data->valid)
            {
                emb_printf("*Kernel Time\t\txxxx\t\t%u.%u %%\n", ps_data->k_time/100, ps_data->k_time%100);
            }

            valid_input = true;
        }

        // Process Memory Map
        // show stack allocation for requested process
        // and malloc memory if any
        else if( isCommand(&data, "pmap", 1) )
        {
            arg_int = getFieldInteger(&data, 1, &valid_integer);

            if(valid_integer)
            {
                emb_memcpy(kernel_data_ptr, &arg_int, sizeof(uint32_t));
                getKernelData(SHELL_PMAP, kernel_data_ptr);
                pmap_data = (SHELL_PMAP_S*)kernel_data_ptr;

                if(pmap_data->found)
                {
                    putsUart0("\nADDR\t\t\t\tSIZE\tTYPE\n");
                    emb_printf("0x%X - 0x%X\t\t%u\tSTACK\n", pmap_data->stackTop, pmap_data->stackBottom, pmap_data->stackSize);

                    if(pmap_data->usedMalloc)
                    {
                        emb_printf("0x%X - 0x%X\t\t%u\tMALLOC\n", pmap_data->heapTop, pmap_data->heapBottom, pmap_data->heapSize);
                    }

                    putcUart0('\n');
                }
                else
                    putsUart0("PID not found for 'pmap'.\n");
            }
            else
                putsUart0("Invalid argument for 'pmap'.\n");

            valid_input = true;
        }

        // Inter-process Communication
        // Show semaphores
        else if( isCommand(&data, "ipcs", 0) )
        {
            semaphore *output_sem;
            putsUart0("\nNAME\t\tCOUNT\t\tQUEUE\n");
            for(i = 0; i < MAX_SEMAPHORES; i++)
            {
                emb_memcpy(kernel_data_ptr, &i, 1);
                getKernelData(SHELL_IPCS, kernel_data_ptr);
                output_sem = (semaphore*)kernel_data_ptr;
                if(output_sem->name[0] != '\0')
                    emb_printf("%s\t%u\t\t[", output_sem->name, output_sem->count);
                for(j = 0; j < output_sem->queueSize; j++)
                {
                    if(j + 1 != output_sem->queueSize)
                        emb_printf("%u, ", output_sem->processQueue[j]);
                    else
                        emb_printf("%u", output_sem->processQueue[j]);

                }
                if(output_sem->name[0] != '\0')
                    putsUart0("]\n");
            }
            putcUart0('\n');
            valid_input = true;
        }

        // Kill Process
        // compares the passed in name and finds it in the tcb array
        else if( isCommand(&data, "kill", 1) )
        {
            arg_int = getFieldInteger(&data, 1, &valid_integer);

            if(valid_integer)
            {
                stopThread((_fn)arg_int);
            }
            else
                putsUart0("Invalid argument for 'kill'.\n");

            valid_input = true;
        }

        // PID_OF
        // checks the name passed in and returns the pid (also fn pointer)
        else if( isCommand(&data, "pidof", 1) )
        {
            taskName = getFieldString(&data, 1);

            // fill kernel_data_ptr with the name of the task we want PID of
            emb_memcpy(kernel_data_ptr, taskName, emb_strlen(taskName) + 1);
            // kernel will take the name and overwrite with PID #
            getKernelData(SHELL_PIDOF, kernel_data_ptr);
            // data_ptr should have pid # now, so print it out

            if( !strcomp( (char*)kernel_data_ptr, "NOTFOUND") )
            {
                emb_printf("PID: %u\n", *(uint32_t*)kernel_data_ptr);
            }
            else
                putsUart0("Invalid process name for 'pmap'.\n");

            valid_input = true;
        }

        // RUN
        // checks the name passed in and returns the pid (also fn pointer)
        else if( isCommand(&data, "run", 1) )
        {
            taskName = getFieldString(&data, 1);
            // fill kernel_data_ptr with the name of the task we want PID of
            emb_memcpy(kernel_data_ptr, taskName, emb_strlen(taskName) + 1);
            // kernel will take the name and overwrite with PID #
            getKernelData(SHELL_PIDOF, kernel_data_ptr);

            if( !strcomp( (char*)kernel_data_ptr, "NOTFOUND") )
            {
                restartThread((_fn)*(uint32_t*)kernel_data_ptr);
            }
            else
                putsUart0("Invalid process name for 'run'.\n");

            valid_input = true;
        }

        // PREEMP
        // toggle it based on input
        else if( isCommand(&data, "preemp", 1) )
        {
            arg_str = getFieldString(&data, 1);
            if( strcomp(arg_str, "on") )
            {
                setPreemption(true);
                putsUart0("Preemption active.\n");
            }
            else if( strcomp(arg_str, "off") )
            {
                setPreemption(false);
                putsUart0("Preemption disabled.\n");
            }
            else
                putsUart0("Invalid argument for 'preemp'.\n");
            valid_input = true;
        }

        // SCHED
        // toggle it based on input
        else if( isCommand(&data, "sched", 1) )
        {
            arg_str = getFieldString(&data, 1);
            if( strcomp(arg_str, "prio") )
            {
                setPriority(true);
                putsUart0("Priority scheduling active.\n");
            }
            else if( strcomp(arg_str, "rr") )
            {
                setPriority(false);
                putsUart0("Round robin scheduling active.\n");
            }
            else
                putsUart0("Invalid argument for 'sched'.\n");
            valid_input = true;
        }

        else if( data.buffer[0] == '\0' || data.buffer[0] == '\n' )
        {
            valid_input = true;
        }

        if(!valid_input)
        {
            emb_printf("Invalid input: '%s'\n", &data.buffer[0]);
        }
        valid_input = false;
        putcUart0('>');
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initUart0();
    initMpu();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(250000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(250000);


    // Initialize semaphores
    createSemaphore(keyPressed, 1, "keyPressed");
    createSemaphore(keyReleased, 0, "keyReleased");
    createSemaphore(flashReq, 5, "flashReq");
    createSemaphore(resource, 1, "resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);
    //ok &=  createThread(idle_another, "IdleAnth", 7, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    //ok &= createThread(flash2Hz, "Flash2Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
    ok &= createThread(errant, "Errant", 6, 1024);
    ok &= createThread(shell, "Shell", 6, 2048);


    putsUart0("\nRTOS Initialized.\n");
    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        setPinValue(RED_LED, 1);

    return 0;
}
