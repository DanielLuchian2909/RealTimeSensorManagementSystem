/**
 ********************************************************************************
 * @file scheduler.c
 * @author Daniel Luchian
 * @brief RTOS scheduler implementation for managing thread execution
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <stdlib.h>

#include "scheduler.h"
#include "main.h"
#include "kernel.h"
#include "queue.h"

/************************************
 * EXTERN VARIABLES
 ************************************/
extern RTOSQueue* g_psRTOSQueue; //Pointer to a global RTOSQueue

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
#define GET_PSP_OFFSET 0x32

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
BOOLE Sched(void); //RTOS scheduler

/************************************
 * STATIC FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
BOOLE //Return true if the schedular was run without exceptions
Sched() //RTOS Scheduler, full implementation tbd
{
    //Dequeue the current thread and TODO checks if a thread context was dequeued
    ThreadContextStruct sDeQueuedThread = rtos_DeQueue(g_psRTOSQueue);

    //TODO: Make this whole block conditional on the OS behaviour
    if (TRUE)
    {
    	//Save the current thread's stack pointer
    	sDeQueuedThread.puiMyThreadStackPointer = (UINT*)(__get_PSP() - 8*4);

		//Allocate a new ThreadContextStructure for the dequeued threadcheck if malloc was succesful
		ThreadContextStruct* psDeQueuedThread = (ThreadContextStruct*)malloc(sizeof(ThreadContextStruct));
		if (psDeQueuedThread == NULL)
		{
			//TODO add debug log
			return FALSE; //Memory allocation failed
		}

		//Copy over the dequeued thread data into the new ThreadContextStructure
		psDeQueuedThread->pfnMyThreadFunction = sDeQueuedThread.pfnMyThreadFunction;
		psDeQueuedThread->puiMyThreadStackPointer = sDeQueuedThread.puiMyThreadStackPointer;

		//Re-enqueue the current thread to the rear of the queue
		if (!rtos_EnQueue(g_psRTOSQueue, psDeQueuedThread))
		{
			free(psDeQueuedThread); //Ensure failed enqueue memeory is freeed
			//TODO add to debug log
			return FALSE; //Check if thread is successfully enqueued
		}
    }

    //Set PSP to the next thread's stack pointer
    __set_PSP((UINT)rtos_PeekQueue(g_psRTOSQueue)->psThreadData->puiMyThreadStackPointer); //used to be psmynextthread

    return TRUE;
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void
rtos_Yield() //Function that yields a thread
{
	__asm("SVC #1");
}

