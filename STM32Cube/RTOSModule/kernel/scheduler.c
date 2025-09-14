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
#include "rtos_arm_m4.h"
#include "lk_kernel.h"
#include "thread_queue.h"

/************************************
 * EXTERN VARIABLES
 ************************************/
extern thread_queue_t* g_rtos_queue; //Pointer to a global RTOSQueue
extern volatile UINT g_kernel_status_flag;

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
thread_manager_t g_thread_manager;

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
BOOLE //Return true if the scheduler was run without exceptions
sched() //RTOS Scheduler, full implementation tbd
{
    //Dequeue the current thread and TODO checks if a thread context was dequeued
    tcb_t dequeued_thread = rtos_deQueue(g_rtos_queue);

    //TODO: Make this whole block conditional on the OS behavior
    if (TRUE)
    {
    	//Save the current thread's stack pointer
    	dequeued_thread.thread_stack_ptr_ = (UINT*)(__get_PSP() - 8*4);

		//Allocate a new thread_context_struct_ture for the dequeued thread check if malloc was succesful
		tcb_t* new_thread = (tcb_t*)malloc(sizeof(tcb_t));
		if (new_thread == NULL)
		{
			//TODO add debug log
			return FALSE; //Memory allocation failed
		}

		//Copy over the dequeued thread data into the new thread_context_struct
		new_thread->thread_fn_ = dequeued_thread.thread_fn_;
		new_thread->thread_stack_ptr_ = dequeued_thread.thread_stack_ptr_;

		//Re-enqueue the current thread to the rear of the queue
		if (!rtos_enQueue(g_rtos_queue, new_thread))
		{
			free(new_thread); //Ensure failed enqueue memory is freed
			//TODO add to debug log
			return FALSE; //Check if thread is successfully enqueued
		}
    }

    //Set PSP to the next thread's stack pointer
    __set_PSP((UINT)rtos_peekQueue(g_rtos_queue)->thread_data_->thread_stack_ptr_);

    // Hack for mutex implementation
    g_thread_manager.current_thread_ = rtos_peekQueue(g_rtos_queue)->thread_data_;

    g_kernel_status_flag &= ~(KERNEL_SCHEDULING);

    return TRUE;
}
