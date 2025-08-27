/**
 ********************************************************************************
 * @file scheduler.h
 * @author Daniel Luchian
 * @brief Header file for the scheduler
 ********************************************************************************
 */

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "base_types.h" //Basic type definitions and utilities
#include "thread_queue.h"

/************************************
 * MACROS AND DEFINES
 ************************************/
#define MAX_THREAD_PRIO_LEVEL  5
#define ROUND_ROBIN_TIMEOUT_MS 1000; //The default value for a threads timeslice in ms

/************************************
 * TYPEDEFS
 ************************************/
typedef struct thread_manager_t
{
	// Thread state data structures
	thread_queue_t* ready_queue_[MAX_THREAD_PRIO_LEVEL];
	thread_queue_t* blocked_queue_;
	thread_queue_t* sleeeping_queue_;
	thread_queue_t* deleted_queue_;
	thread_queue_t* error_queue_;

	// Current execution data
	tcb_t* current_thread_;

	// Meta data
} thread_manager_t;

/************************************
 * EXPORTED VARIABLES
 ************************************/
extern thread_manager_t g_thread_manager;

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

#ifdef __cplusplus
}
#endif

#endif
