/**
 ********************************************************************************
 * @file kernel.c
 * @author Daniel Luchian
 * @brief The kernel file for this RTOS, responsible for kernel
 * initialization, starting, and handling interrupts.
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <stdlib.h>

#include "lk_kernel.h"
#include "scheduler.h"
#include "thread_queue.h"
#include "lk_itc.h"
#include "rtos_arm_m4.h"

/************************************
 * EXTERN VARIABLES AND FUNCTIONS
 ************************************/
extern void runFirstThread(void); //Function to start the first thread
extern CHAR rtos_itcInit(void);

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
#define SVC_ARGUMENT_RETURN_ADDRESS_INDEX  6
#define SVC_NUMBER_OFFSET                 -2
#define PENDSV_TRIGGER_BIT                28

#define START_THREAD_ID                    5

/************************************
 * STATIC VARIABLES
 ************************************/
static UINT* minthread_stack_ptr = NULL; //Lowest valid thread stack pointer
static UINT* last_allocated_thread = NULL; //Last allocated thread stack pointer

/************************************
 * GLOBAL VARIABLES
 ************************************/
thread_queue_t* g_rtos_queue = NULL; //RTOS thread queue
UINT g_kernel_status_flag = 0;

/************************************
 * STATIC FUNCTIONS
 ************************************/
static UINT* allocateThread(void); //Allocates a new thread stack pointer
static void configExceptionPrios(void);
static thread_id_t assignThreadId(void);

//-----------------------------------------------------------------------
static UINT*  //A pointer to a valid stack address for a new thread, or if no valid address exists; NULL
allocateThread() //Get a thread pointer
{
	UINT* potential_thread = (UINT*)((UINT)last_allocated_thread - THREAD_STACK_SIZE);

	//If the potential new thread address fits in the stack return it, otherwise return NULL
	if (potential_thread >= minthread_stack_ptr)
	{
		return potential_thread;
	}
	else
	{
		return NULL; //Not enough space for a new thread
	}
}

//-----------------------------------------------------------------------
static void
configExceptionPrios()
{
	//int x = SVC_PRIO;
	//int y = PENSV_PRIO;
	//int z = SYSTICK_PRIO;
}

//-----------------------------------------------------------------------
static thread_id_t
assignThreadId()
{
	static thread_id_t prev_thread_id = START_THREAD_ID;
	return prev_thread_id++;
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void
svcHandlerMain( //SVC Handler function
		UINT* svc_args) //A pointer to an unsigned integer containing the SVC argument
{
	UINT svc_num = 0;

	/*
	 * Stack contains:
	 * r0, r1, r2, r3, r12, r24, the return address and xPSR
	 * First argument (r0) is svc_args[0]
	 */
	svc_num = ((CHAR*)svc_args[SVC_ARGUMENT_RETURN_ADDRESS_INDEX])[SVC_NUMBER_OFFSET];

	switch(svc_num)
	{

	//Run the first thread
	case RUN_FIRST_THREAD:
		g_thread_manager.current_thread_ = rtos_peekQueue(g_rtos_queue)->thread_data_;
		__set_PSP ((UINT)rtos_peekQueue(g_rtos_queue)->thread_data_->thread_stack_ptr_);
		runFirstThread();
		break;

	//Pend an interrupt to do a context switch
	case RTOS_YIELD:
		_ICSR |= (1 << PENDSV_TRIGGER_BIT); // Trigger PendSV
		__ISB();
		break;

	default:
		break;
	}
}

//-----------------------------------------------------------------------
BOOLE //True if the kernel was initialized with not errors
lk_kernelInit() //Function to initialize kernel related information
{
	// 1) Configure Exception Priorities
	configExceptionPrios();

	//Set PendSV priority to weakest
	SHPR3 |= PENDSV_INTERRUPT_PRIO << 16; //Shift the constant 0xFE 16 bits to set PendSV priority
	SHPR2 |= 0xFDU << 24; //Set the priority of SVC higher than PendSV

	//Getting the value of the MSP pointer
	UINT* msp_val = *(UINT**)0x0;

	//Setting the last allocated thread to the MSP
	last_allocated_thread = msp_val;

	//Calculating the maximum address of a thread stack pointer
	minthread_stack_ptr = (UINT*)(((UINT)msp_val - STACK_SIZE) + THREAD_STACK_SIZE);

	//Initializing threads circular queue
	g_rtos_queue = rtos_initQueue(STACK_SIZE / THREAD_STACK_SIZE - 1);
	if (g_rtos_queue == NULL)
	{

		g_kernel_status_flag |= KERNEL_NOT_INITIALIZED;
		return FALSE; //Failed to create kernel queue
	}

	// Initialize the message queue
	if (rtos_itcInit() != 0)
	{
		g_kernel_status_flag |= KERNEL_NOT_INITIALIZED;
		return FALSE;
	}

	//Kernel successfully created
	g_kernel_status_flag |= KERNEL_INITIALIZED;
	return TRUE;
}

//-----------------------------------------------------------------------
void
lk_kernelStart() //Function to start the kernel
{
	g_kernel_status_flag |= KERNEL_STARTED;
	__ASM("SVC #0");
}

//-----------------------------------------------------------------------
BOOLE //Return whether a thread was succesfully created
lk_createThread( //A function that creates a thread
		void (*thread_fn)(void*), //A pointer to the thread's function
		void *parameters) //A pointer to the thread's arguments
{
	//Allocate a new thread pointer
	UINT* new_thread_ptr = allocateThread();
	if (new_thread_ptr == NULL)
	{

		return FALSE; //New thread stack allocation failed
	}

	//Update last allocated thread information
	last_allocated_thread = new_thread_ptr;

	//Set the thread context
	*(--new_thread_ptr) =  1<<24; //Set the XPSR register
	*(--new_thread_ptr) = (UINT)thread_fn; //Set the PC register
	for (int i=0; i<5; ++i)
	{
		*(--new_thread_ptr) = 0xA; //Dummy values to fill thread context stack
	}

	*(--new_thread_ptr) = (UINT)parameters;

	for (int i=0; i<8; i++)
	{
		*(--new_thread_ptr) = 0xA;
	}

	//Create a new thread context structure and check if malloc was successful
	tcb_t* new_thread_context = (tcb_t*)malloc(sizeof(tcb_t));
	if (new_thread_context == NULL)
	{

		return FALSE; //Memory allocation failed
	}

	//Setup the thread contexts
	new_thread_context->thread_stack_ptr_ = new_thread_ptr;
	new_thread_context->thread_fn_ = thread_fn;
	new_thread_context->thread_runtime_ms_ = ROUND_ROBIN_TIMEOUT_MS;
	new_thread_context->thread_timeslice_ms_ = ROUND_ROBIN_TIMEOUT_MS;
	new_thread_context->state_ = THREAD_READY;
	new_thread_context->thread_id_ = assignThreadId();

	//Enqueue the new thread
	if (!rtos_enQueue(g_rtos_queue, new_thread_context))
	{
		free(new_thread_context); //Free context memory if enqueue failed
		return FALSE;
	}

	return TRUE;
}

//-----------------------------------------------------------------------
BOOLE //Return whether a thread was successfully created
lk_createThreadWithDeadline( //A function that creates a thread
		void (*thread_fn)(void*), //A pointer to the thread's function
		void *parameters, //A pointer to the thread's arguments
		UINT deadline)
{
	//Create a standard thread
	if (!lk_createThread(thread_fn, parameters))
	{
		//some debug log
		return FALSE;
	}

	//Set the threads deadline
	rtos_peekQueue(g_rtos_queue)->thread_data_->thread_runtime_ms_ = deadline;
	rtos_peekQueue(g_rtos_queue)->thread_data_->thread_timeslice_ms_ = deadline;

	return TRUE;
}

//-----------------------------------------------------------------------
void
lk_threadYield() //Function that yields a thread
{
	__ASM("SVC #1");
}
