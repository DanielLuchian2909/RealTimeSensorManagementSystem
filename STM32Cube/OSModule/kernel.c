/**
 ********************************************************************************
 * @file kernel.c
 * @author
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <stdlib.h>
#include <stddef.h>

#include "kernel.h"
#include "main.h"

/************************************
 * EXTERN VARIABLES AND FUNCTIONS
 ************************************/
extern void runFirstThread(void);

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
#define THREAD_CONTEXT_OVERHEAD 0x40

/************************************
 * STATIC VARIABLES
 ************************************/
static UINT* puiMaxThreadStackPointer = NULL;

static UINT* puiLastAllocatedThread = NULL;

static ThreadContextStruct stMyThread1;

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

//-----------------------------------------------------------------------
UINT*  //A pointer to a valid stack address for a new thread, or if no valid address exists, NULL
AllocateThread( //Get a thread pointer
		void)
{
	UINT* puiPotentialThread = (UINT*)((UINT)puiLastAllocatedThread - THREAD_STACK_SIZE);

	//If the potential new thread address fits in the stack return it, otherwise return NULL
	if (puiPotentialThread >= puiMaxThreadStackPointer)
		return puiPotentialThread;
	else
		return NULL;
}

//-----------------------------------------------------------------------
void
SVC_Handler_Main( //SVC Handler function
		UINT* puiSvcArgs) //A pointer to an unsigned integer containin the SVC argument
{
	UINT uiSvcNumber;

	/*
	 * Stack contains:
	 * r0, r1, r2, r3, r12, r24, the return address and xPSR
	 * First argument (r0) is svc_args[0]
	 */
	uiSvcNumber = ((CHAR*)puiSvcArgs[6])[-2];

	switch(uiSvcNumber)
	{
	case RUN_FIRST_THREAD:
		__set_PSP((UINT)stMyThread1.puiMyThreadStackPointer - THREAD_CONTEXT_OVERHEAD);
		runFirstThread();
		break;

	default:
		break;
	}
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/

//-----------------------------------------------------------------------
void
os_KernelInit( //Function to initiliaze kernel related information
		void)
{
	//Getting the value of the MSP pointer
	UINT* puiMspVal = *(UINT**)0x0;

	//Setting the last allocated thread to the MSP
	puiLastAllocatedThread = puiMspVal;

	//Calculating the maximum address of a thread stack pointer
	puiMaxThreadStackPointer = (UINT*)(((UINT)puiMspVal - STACK_SIZE) + THREAD_STACK_SIZE);

}

//-----------------------------------------------------------------------
void
os_KernelStart( //Function to start the kernel
		void)
{
	__asm("SVC #0");
}

//-----------------------------------------------------------------------
BOOLE //Return whether a thread was succesfully created
os_CreateThread( //A function that creates a thread
		void (*pfnThreadFunction)(void*)) //A pointer to the thread fucntion
{
	UINT* puiNewThreadPointer = AllocateThread();

	//Check a valid stack pointer was returned
	if (puiNewThreadPointer == NULL)
		return FALSE;

	UINT* puiTempThreadPointer = puiNewThreadPointer;

	//Set the thread context
	*(--puiTempThreadPointer) =  1<<24;
	*(--puiTempThreadPointer) = (UINT)pfnThreadFunction;
	for (int i=0; i<14; ++i)
	{
		*(--puiTempThreadPointer) = 0xA; //Dummy values to fill thread context stack
	}

	//Update thread information
	stMyThread1.puiMyThreadStackPointer = puiNewThreadPointer;
	stMyThread1.pfnMyThreadFunction = pfnThreadFunction;

	//Update last allocated thread information
	puiLastAllocatedThread = puiNewThreadPointer;

	return TRUE;
}




