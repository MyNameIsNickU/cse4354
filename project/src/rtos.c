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

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly
uint8_t * heap = (uint8_t *)0x20001000;
uint8_t * psp = (uint8_t*)0x20008000;

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 7=lowest
    uint32_t ticks;                // ticks until sleep complete
    uint32_t srd;                  // MPU subregion disable bits (one per 1 KiB)
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];


//-----------------------------------------------------------------------------
// Memory Manager and MPU Funcitons
//-----------------------------------------------------------------------------

#define SRAM_STARTING_REGION 3

uint32_t getSRD(uint32_t base_addr, uint32_t size_to_allocate)
{
    size_to_allocate = ((size_to_allocate + 1023) / 1024) * 1024; // rounds up to nearest 1 KiB
    int8_t subregions_to_touch = size_to_allocate >> 10; // how many subregions needed for the size requested
    uint8_t subregion_index = (base_addr - 0x20000000) >> 10;
    //uint8_t start_region = ((base_addr - 0x20000000) >> 13) + SRAM_STARTING_REGION; // which of the 4 SRAM regions are we starting

    uint32_t srd_mask = 0;

    while(subregions_to_touch > 0)
    {
        srd_mask += (1 << subregion_index++);
        subregions_to_touch--;
    }

    emb_printf("SRD_Mask: 0x%X\n", srd_mask);

    return srd_mask;


    // TODO: take this part and put it into its own setSRD() function
    /*uint32_t srd_area = 0x000000FF;
    subregions_to_touch = size_to_allocate >> 10;

    uint8_t i;
    for(i = 0; i <= SRAM_STARTING_REGION; i++)
    {
        NVIC_MPU_NUMBER_R = SRAM_STARTING_REGION + i;
        // get the 8 bits of SRD mask, shift them down by which 8bits it's in
        // and then shift it left 8bits to get to SRD bits
        NVIC_MPU_ATTR_R |= (((srd_mask & srd_area) >> i*8) << 8);
        srd_area <<= 8;
    }*/
}


// TODO: add your malloc code here and update the SRD bits for the current thread
void * mallocFromHeap(uint32_t size_in_bytes)
{
    void * temp = NULL;
    //extern uint8_t * heap;
    uint32_t size_to_allocate = size_in_bytes;

    // rounds up to the nearest 1 KiB
    // ASSUME: should already be rounded up
    //size_to_allocate = ((size_in_bytes + 1023) / 1024) * 1024;

    if(heap < (uint8_t*)0x20008000)
    {
        temp = heap;
        heap += size_to_allocate;
    }

    //setSRD((uint32_t)temp, size_to_allocate);

    //emb_printf("Size wanted: %u\n", size_in_bytes);
    emb_printf("Size to allocate: %u\n", size_to_allocate);
    emb_printf("Address Allocated: 0x%x\n", temp);
    emb_printf("New Heap Pointer: 0x%x\n", heap);

    return temp;
}

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
    // REQUIRED: call your MPU functions here
    setupBackgroundRegion();
    allowFlashAccess();
    allowPeripheralAccess();
    //setupSramAccess();

    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE; // enable MPU
}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void initRtos()
{
    // SysTick set to 1 kHz or 1ms
    initSysTick();

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

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
}

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
            tcb[i].sp = ((uint8_t*)mallocFromHeap(stackBytes) + stackBytes);
            tcb[i].spInit = tcb[i].sp;
            tcb[i].priority = priority;
            // calculates SRD mask based off of initial base address and size of stack requested
            tcb[i].srd = getSRD( (uint32_t)((uint8_t*)tcb[i].spInit - stackBytes), stackBytes);
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
}

// REQUIRED: modify this function to stop a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void stopThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, TMPL bit, and PC
void startRtos()
{
    // get index of task to run from tcb
    taskCurrent = rtosScheduler();
    // set SP of task to malloc'd memory
    setPSP(tcb[taskCurrent].spInit);
    // use PSP
    setASP();
    setUnprivileged();

    // load the function call into variable

    // NOTE: this is defined on the new PSP so...
    // ...the task will always lose 4 bytes of data in its stack
    // for this method
    tcb[taskCurrent].state = STATE_READY;
    //_fn task = *((_fn)tcb[taskCurrent].pid)();
    //(*task)();


    ((_fn)tcb[taskCurrent].pid)();
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

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t semaphore)
{
    // indicates a needed service call for service "43"
    // the svCallIsr will handle the wait for semaphore service
    __asm(" SVC #43");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    // indicates a needed service call for service "54"
    // the svCallIsr will handle the post semaphore service
    __asm(" SVC #54");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
	// clear pending systick
	NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PENDSTCLR;
	
	// TODO: do I search the entire tcb for a sleeping task?
	// yes, i do

	for(i = 0; i < MAX_TASKS; i++)
	{
	    if(tcb[i].state == STATE_DELAYED)
	    {
	        if(tcb[i].ticks == 0)
	            tcb[i].state = STATE_READY;
	        else
	            tcb[i].ticks--;
	    }
	}
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    //emb_printf("Pendsv in process N");
    if(NVIC_FAULT_STAT_R && NVIC_FAULT_STAT_DERR || NVIC_FAULT_STAT_R && NVIC_FAULT_STAT_IERR)
    {
        //emb_printf("...called from MPU\n");
    }
    //putcUart0('\n');


    //putcUart0('p');
	
    //TODO: push stuff onto stack
    // aren't the core resgisters already on the PSP at this point?
    pushCore();
    uint32_t *psp = getPSP();

    // save the PSP
    tcb[taskCurrent].sp = psp;

    // get a new task
    taskCurrent = rtosScheduler();

	// if task state is ready, then get the last known PSP value and pop the core registers
    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSP( (uint32_t*)tcb[taskCurrent].sp );
        // TODO: pop the registers from this new PSP
        popCore();
    }
	// if task state is unrun, set PSP to initial value and load core registers with default values
    else if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        // Set psp
        setPSP( (uint32_t*)tcb[taskCurrent].spInit );

		// this should load a new LR and PC
        setupUnrun( (uint32_t)tcb[taskCurrent].pid );


        tcb[taskCurrent].state = STATE_READY;
    }

	// clear pending sv flag
	NVIC_INT_CTRL_R |= NVIC_INT_CTRL_UNPEND_SV;
    //while(1) { }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    // get R0
    uint32_t r0_value = extractR0();

    //putcUart0('s');

    // get the specific service requested
    uint32_t *psp = getPSP();

    // gets the PC from PSP
    // thumb instructions are 16 b, so cast as 16b pointer
    // PC points at the NEXT instruction to execute
    // so go back one instruction to get the SVC instruction
    // the lower 8b are the number used in the SVC.
    uint16_t sv_instruct_value = *((uint16_t*)(*(psp+6)) - 1);
    uint8_t sv_num = sv_instruct_value & 0xFF;
	
	if(sv_num == 21)
	{
		//putsUart0("YIELD.");
	}

    if(sv_num == 32)
    {
        //emb_printf("SLEEP.");
        // set initial value of ticks to what was passed into R0
        tcb[taskCurrent].ticks = r0_value;

        // do I set the state to DELAYED?
        tcb[taskCurrent].state = STATE_DELAYED;
    }

    // set bit for pending SV call
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

void mpuFaultIsr()
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
    NVIC_CFG_CTRL_R |= NVIC_CFG_CTRL_DIV0;
}

// TODO: test LEDs and PBs
uint8_t readPbs()
{
    return getButtons();
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{

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
void shell()
{
    USER_DATA data;
    while (true)
    {
        putcUart0('>');
        getsUart0(&data);
        parseFields(&data);

        if( handleCommand(&data) ) { }
        else
        {
            emb_printf("Invalid input: '%s'\n", &data.buffer[0]);
        }
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
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);
    ok &=  createThread(idle_another, "IdleAnth", 7, 1024);

    // Add other processes
//    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
//    ok &= createThread(oneshot, "OneShot", 2, 1024);
//    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
//    ok &= createThread(debounce, "Debounce", 6, 1024);
//    ok &= createThread(important, "Important", 0, 1024);
//    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
//    ok &= createThread(errant, "Errant", 6, 1024);
//    ok &= createThread(shell, "Shell", 6, 2048);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        setPinValue(RED_LED, 1);

    return 0;
}
