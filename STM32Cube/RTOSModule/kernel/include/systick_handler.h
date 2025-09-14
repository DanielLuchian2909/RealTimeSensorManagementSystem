/**
 ********************************************************************************
 * @file systick_handler.h
 * @author Daniel Luchian
 * @brief
 ********************************************************************************
 */

#ifndef __SYSTICK_HANDLER_H__
#define __SYSTICK_HANDLER_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "thread_queue.h"
#include "lk_kernel.h"
#include "rtos_arm_m4.h"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/
extern thread_queue_t* g_rtos_queue; //Pointer to a global RTOSQueue
extern UINT g_kernel_status_flag;

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
static inline void // Static inline for speed and internal linkage to stm32f4xx_it.c
rtos_handleSystick(void) // Function to handle systick firing
{
	// Run or update scheduler if kernel is working but not currently scheduling
	if ( (g_kernel_status_flag & KERNEL_STARTED) &&
		 !(g_kernel_status_flag & KERNEL_SCHEDULING))
	{
		//If the thread is not done running, decrement its runtime, otherwise yield
		if( rtos_peekQueue(g_rtos_queue)->thread_data_->thread_runtime_ms_ > 0)
		{
			--(rtos_peekQueue(g_rtos_queue)->thread_data_->thread_runtime_ms_);
		}
		else
		{
			rtos_peekQueue(g_rtos_queue)->thread_data_->thread_runtime_ms_ = rtos_peekQueue(g_rtos_queue)->thread_data_->thread_timeslice_ms_;

			_ICSR |= 1<<28; // Fire PendSV (interrupt responsible for scheduler)

			__ISB(); // Flush processor pipeline - make sure state change is visible

			/* Explanation for self: You might think why does it matter if I flush the pipeline if PendSV just fires
			 * before ISB happens? But here interrupt priority context matters. Since setting ICSR -> PendSV is lower
			 * priority than systick, even after setting that interrupt systick will finish completion first so
			 * the ISB will make sure the processor sees the new state of ICSR, if for some reason (which should never happen)
			 * systick becomes lower priority than PendSV the ISB becomes useless.
			 */

			g_kernel_status_flag |= KERNEL_SCHEDULING;
		}
	}
}

#ifdef __cplusplus
}
#endif

#endif // __SYSTICK_HANDLER_H__
