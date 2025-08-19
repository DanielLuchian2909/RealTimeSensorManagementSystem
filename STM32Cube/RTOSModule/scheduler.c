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
extern thread_queue_t* g_ps_rtos_queue; //Pointer to a global RTOSQueue

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
BOOLE sched(void); //RTOS scheduler

/************************************
 * STATIC FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
BOOLE //Return true if the schedular was run without exceptions
sched() //RTOS Scheduler, full implementation tbd
{
    //Dequeue the current thread and TODO checks if a thread context was dequeued
    thread_context_struct_t s_dequeued_thread = rtos_deQueue(g_ps_rtos_queue);

    //TODO: Make this whole block conditional on the OS behaviour
    if (TRUE)
    {
    	//Save the current thread's stack pointer
    	s_dequeued_thread.pui_thread_stack_ptr_ = (UINT*)(__get_PSP() - 8*4);

		//Allocate a new thread_context_struct_ture for the dequeued threadcheck if malloc was succesful
		thread_context_struct_t* ps_dequeued_thread = (thread_context_struct_t*)malloc(sizeof(thread_context_struct_t));
		if (ps_dequeued_thread == NULL)
		{
			//TODO add debug log
			return FALSE; //Memory allocation failed
		}

		//Copy over the dequeued thread data into the new thread_context_struct
		ps_dequeued_thread->pfn_thread_fn_ = s_dequeued_thread.pfn_thread_fn_;
		ps_dequeued_thread->pui_thread_stack_ptr_ = s_dequeued_thread.pui_thread_stack_ptr_;

		//Re-enqueue the current thread to the rear of the queue
		if (!rtos_enQueue(g_ps_rtos_queue, ps_dequeued_thread))
		{
			free(ps_dequeued_thread); //Ensure failed enqueue memeory is freeed
			//TODO add to debug log
			return FALSE; //Check if thread is successfully enqueued
		}
    }

    //Set PSP to the next thread's stack pointer
    __set_PSP((UINT)rtos_peekQueue(g_ps_rtos_queue)->ps_thread_data_->pui_thread_stack_ptr_); //used to be psmynextthread

    return TRUE;
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void
rtos_yield() //Function that yields a thread
{
	__asm("SVC #1");
}

