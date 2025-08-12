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
#define SVC_ARGUMENT_RETURN_ADDRESS_INDEX 6
#define SVC_NUMBER_OFFSET                -2
#define PENDSV_TRIGGER_BIT               28

/************************************
 * STATIC VARIABLES
 ************************************/
static UINT* puiMinThreadStackPointer = NULL; //Lowest valid thread stack pointer
static UINT* puiLastAllocatedThread = NULL; //Last allocated thread stack pointer

/************************************
 * GLOBAL VARIABLES
 ************************************/
RTOSQueue* g_psRTOSQueue = NULL; //RTOS thread queue
UINT g_uiKernelStatusFlag = 0;

/************************************
 * STATIC FUNCTIONS
 ************************************/
UINT* allocateThread(void); //Allocates a new thread stack pointer
void svcHandlerMain(UINT* puiSvcArgs); //SVC Handler

//-----------------------------------------------------------------------
UINT*  //A pointer to a valid stack address for a new thread, or if no valid address exists; NULL
allocateThread() //Get a thread pointer
{
	UINT* puiPotentialThread = (UINT*)((UINT)puiLastAllocatedThread - THREAD_STACK_SIZE);

	//If the potential new thread address fits in the stack return it, otherwise return NULL
	if (puiPotentialThread >= puiMinThreadStackPointer)
		return puiPotentialThread;
	else
		return NULL; //Not enough space for a new thread
}

//-----------------------------------------------------------------------
void
svcHandlerMain( //SVC Handler function
		UINT* puiSvcArgs) //A pointer to an unsigned integer containing the SVC argument
{
	UINT uiSvcNumber;

	/*
	 * Stack contains:
	 * r0, r1, r2, r3, r12, r24, the return address and xPSR
	 * First argument (r0) is svc_args[0]
	 */
	uiSvcNumber = ((CHAR*)puiSvcArgs[SVC_ARGUMENT_RETURN_ADDRESS_INDEX])[SVC_NUMBER_OFFSET];

	switch(uiSvcNumber)
	{

	//Run the first thread
	case RUN_FIRST_THREAD:
		__set_PSP ((UINT)
				rtos_peekQueue(g_psRTOSQueue)->psThreadData->puiMyThreadStackPointer);
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
	UINT* puiMspVal = *(UINT**)0x0;

	//Setting the last allocated thread to the MSP
	puiLastAllocatedThread = puiMspVal;

	//Calculating the maximum address of a thread stack pointer
	puiMinThreadStackPointer = (UINT*)(((UINT)puiMspVal - STACK_SIZE) + THREAD_STACK_SIZE);

	//Initializing threads circular queue
	g_psRTOSQueue = rtos_initQueue(STACK_SIZE / THREAD_STACK_SIZE - 1);
	if (!g_psRTOSQueue)
	{
		//TODO add some debug log
		g_uiKernelStatusFlag |= KERNEL_NOT_INITIALIZED;
		return FALSE; //Failed to create kernel queue
	}

	//Kernel successfully created
	g_uiKernelStatusFlag |= KERNEL_INITIALIZED;
	return TRUE;
}

//-----------------------------------------------------------------------
void
rtos_kernelStart() //Function to start the kernel
{
	g_uiKernelStatusFlag |= KERNEL_STARTED;
	__asm("SVC #0");
}

//-----------------------------------------------------------------------
BOOLE //Return whether a thread was succesfully created
rtos_createThread( //A function that creates a thread
		void (*pfnThreadFunction_)(void*), //A pointer to the thread's function
		void *pvParameters_) //A pointer to the thread's arguments
{
	//Allocate a new thread pointer
	UINT* puiNewThreadPointer = allocateThread();
	if (puiNewThreadPointer == NULL)
	{
		//TODO add some debug log
		return FALSE; //New thread stack allocation failed
	}

	//Update last allocated thread information
	puiLastAllocatedThread = puiNewThreadPointer;

	//Set the thread context
	*(--puiNewThreadPointer) =  1<<24; //Set the XPSR register
	*(--puiNewThreadPointer) = (UINT)pfnThreadFunction_; //Set the PC register
	for (int i=0; i<5; ++i)
	{
		*(--puiNewThreadPointer) = 0xA; //Dummy values to fill thread context stack
	}

	*(--puiNewThreadPointer) = (UINT)pvParameters_;

	for (int i=0; i<8; i++)
	{
		*(--puiNewThreadPointer) = 0xA;
	}

	//Create a new thread context structure and check if malloc was succesful
	ThreadContextStruct* psNewThreadContext = (ThreadContextStruct*)malloc(sizeof(ThreadContextStruct));
	if (psNewThreadContext == NULL)
	{
		//TODO add some debug log
		return FALSE; //Memory allocatio failed
	}

	//Setup the thread contexts
	psNewThreadContext->puiMyThreadStackPointer = puiNewThreadPointer;
	psNewThreadContext->pfnMyThreadFunction = pfnThreadFunction_;
	psNewThreadContext->uiMyThreadRuntimeMs = ROUND_ROBIN_TIMEOUT_MS;
	psNewThreadContext->uiMyThreadTimesliceMs = ROUND_ROBIN_TIMEOUT_MS;

	//Enqueue the new thread
	if (!rtos_enQueue(g_psRTOSQueue, psNewThreadContext))
	{
		free(psNewThreadContext); //Free context memory if enqueue failed
		//TODO add some debug log
		return FALSE;
	}

	return TRUE;
}

//-----------------------------------------------------------------------
BOOLE //Return whether a thread was succesfully created
rtos_createThreadWithDeadline( //A function that creates a thread
		void (*pfnThreadFunction_)(void*), //A pointer to the thread's function
		void *pvParameters_, //A pointer to the thread's arguments
		UINT uiDeadline_)
{
	//Create a standard thread
	if (!rtos_createThread(pfnThreadFunction_, pvParameters_))
	{
		//some debug log
		return FALSE;
	}

	//Set the threads deadline
	rtos_peekQueue(g_psRTOSQueue)->psThreadData->uiMyThreadRuntimeMs = uiDeadline_;
	rtos_peekQueue(g_psRTOSQueue)->psThreadData->uiMyThreadTimesliceMs = uiDeadline_;

	return TRUE;
}

