/**
 ********************************************************************************
 * @file kernel.c
 * @author Daniel Luchian
 * @brief The kernel file for this RTOS, responsible for kernel
 * initialization, starting, and handling interupts.
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <stdlib.h>

#include "kernel.h"
#include "main.h"
#include "scheduler.h"
#include "queue.h"
#include "itc.h"

/************************************
 * EXTERN VARIABLES AND FUNCTIONS
 ************************************/
extern void runFirstThread(void); //Function to start the first thread

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
#define SVC_ARGUMENT_RETURN_ADDRESS_INDEX  6
#define SVC_NUMBER_OFFSET                 -2
#define PENDSV_TRIGGER_BIT                28

/************************************
 * STATIC VARIABLES
 ************************************/
static UINT* pui_minthread_stack_ptr = NULL; //Lowest valid thread stack pointer
static UINT* pui_last_allocated_thread = NULL; //Last allocated thread stack pointer

/************************************
 * GLOBAL VARIABLES
 ************************************/
thread_queue_t* g_ps_rtos_queue = NULL; //RTOS thread queue
UINT g_ui_kernel_status_flag = 0;

/************************************
 * STATIC FUNCTIONS
 ************************************/
UINT* allocateThread(void); //Allocates a new thread stack pointer
void svcHandlerMain(UINT* pui_svcargs); //SVC Handler

//-----------------------------------------------------------------------
UINT*  //A pointer to a valid stack address for a new thread, or if no valid address exists; NULL
allocateThread() //Get a thread pointer
{
	UINT* pui_potential_thread = (UINT*)((UINT)pui_last_allocated_thread - THREAD_STACK_SIZE);

	//If the potential new thread address fits in the stack return it, otherwise return NULL
	if (pui_potential_thread >= pui_minthread_stack_ptr)
		return pui_potential_thread;
	else
		return NULL; //Not enough space for a new thread
}

//-----------------------------------------------------------------------
void
svcHandlerMain( //SVC Handler function
		UINT* pui_svc_args) //A pointer to an unsigned integer containing the SVC argument
{
	UINT ui_svc_num = 0;

	/*
	 * Stack contains:
	 * r0, r1, r2, r3, r12, r24, the return address and xPSR
	 * First argument (r0) is svc_args[0]
	 */
	ui_svc_num = ((CHAR*)pui_svc_args[SVC_ARGUMENT_RETURN_ADDRESS_INDEX])[SVC_NUMBER_OFFSET];

	switch(ui_svc_num)
	{

	//Run the first thread
	case RUN_FIRST_THREAD:
		__set_PSP ((UINT)
				rtos_peekQueue(g_ps_rtos_queue)->ps_thread_data_->pui_thread_stack_ptr_);
		runFirstThread();
		break;

	//Pend an interrupt to do a context switch
	case RTOS_YIELD:
		_ICSR |= (1 << PENDSV_TRIGGER_BIT); // Trigger PendSV
		__asm("isb");
		break;

	default:
		break;
	}
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
BOOLE //True if the kernel was initialized with not errors
rtos_kernelInit() //Function to initialize kernel related information
{
	//Set PendSV priority to weakest
	SHPR3 |= 0xFE << 16; //Shift the constant 0xFE 16 bits to set PendSV priority
	SHPR2 |= 0xFDU << 24; //Set the priority of SVC higher than PendSV

	//Getting the value of the MSP pointer
	UINT* pui_msp_val = *(UINT**)0x0;

	//Setting the last allocated thread to the MSP
	pui_last_allocated_thread = pui_msp_val;

	//Calculating the maximum address of a thread stack pointer
	pui_minthread_stack_ptr = (UINT*)(((UINT)pui_msp_val - STACK_SIZE) + THREAD_STACK_SIZE);

	//Initializing threads circular queue
	g_ps_rtos_queue = rtos_initQueue(STACK_SIZE / THREAD_STACK_SIZE - 1);
	if (!g_ps_rtos_queue)
	{
		//TODO add some debug log
		g_ui_kernel_status_flag |= KERNEL_NOT_INITIALIZED;
		return FALSE; //Failed to create kernel queue
	}

	// Initialize the message queue


	//Kernel successfully created
	g_ui_kernel_status_flag |= KERNEL_INITIALIZED;
	return TRUE;
}

//-----------------------------------------------------------------------
void
rtos_kernelStart() //Function to start the kernel
{
	g_ui_kernel_status_flag |= KERNEL_STARTED;
	__asm("SVC #0");
}

//-----------------------------------------------------------------------
BOOLE //Return whether a thread was succesfully created
rtos_createThread( //A function that creates a thread
		void (*pfn_thread_fn)(void*), //A pointer to the thread's function
		void *pv_parameters) //A pointer to the thread's arguments
{
	//Allocate a new thread pointer
	UINT* pui_new_thread_ptr = allocateThread();
	if (pui_new_thread_ptr == NULL)
	{
		//TODO add some debug log
		return FALSE; //New thread stack allocation failed
	}

	//Update last allocated thread information
	pui_last_allocated_thread = pui_new_thread_ptr;

	//Set the thread context
	*(--pui_new_thread_ptr) =  1<<24; //Set the XPSR register
	*(--pui_new_thread_ptr) = (UINT)pfn_thread_fn; //Set the PC register
	for (int i=0; i<5; ++i)
	{
		*(--pui_new_thread_ptr) = 0xA; //Dummy values to fill thread context stack
	}

	*(--pui_new_thread_ptr) = (UINT)pv_parameters;

	for (int i=0; i<8; i++)
	{
		*(--pui_new_thread_ptr) = 0xA;
	}

	//Create a new thread context structure and check if malloc was succesful
	thread_context_struct_t* ps_new_thread_context = (thread_context_struct_t*)malloc(sizeof(thread_context_struct_t));
	if (ps_new_thread_context == NULL)
	{
		//TODO add some debug log
		return FALSE; //Memory allocatio failed
	}

	//Setup the thread contexts
	ps_new_thread_context->pui_thread_stack_ptr_ = pui_new_thread_ptr;
	ps_new_thread_context->pfn_thread_fn_ = pfn_thread_fn;
	ps_new_thread_context->ui_thread_runtime_ms_ = ROUND_ROBIN_TIMEOUT_MS;
	ps_new_thread_context->ui_thread_timeslice_ms_ = ROUND_ROBIN_TIMEOUT_MS;

	//Enqueue the new thread
	if (!rtos_enQueue(g_ps_rtos_queue, ps_new_thread_context))
	{
		free(ps_new_thread_context); //Free context memory if enqueue failed
		//TODO add some debug log
		return FALSE;
	}

	return TRUE;
}

//-----------------------------------------------------------------------
BOOLE //Return whether a thread was succesfully created
rtos_createThreadWithDeadline( //A function that creates a thread
		void (*pfn_thread_fn)(void*), //A pointer to the thread's function
		void *pv_parameters, //A pointer to the thread's arguments
		UINT ui_deadline)
{
	//Create a standard thread
	if (!rtos_createThread(pfn_thread_fn, pv_parameters))
	{
		//some debug log
		return FALSE;
	}

	//Set the threads deadline
	rtos_peekQueue(g_ps_rtos_queue)->ps_thread_data_->ui_thread_runtime_ms_ = ui_deadline;
	rtos_peekQueue(g_ps_rtos_queue)->ps_thread_data_->ui_thread_timeslice_ms_ = ui_deadline;

	return TRUE;
}

