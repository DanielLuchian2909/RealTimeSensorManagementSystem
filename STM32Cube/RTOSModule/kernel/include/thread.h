/**
 ********************************************************************************
 * @file thread.h
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __THREAD_H__
#define __THREAD_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "base_types.h"

/************************************
 * MACROS AND DEFINES
 ************************************/
#define THREAD_STACK_SIZE 0x400 //Stack size for individual threads

/************************************
 * TYPEDEFS
 ************************************/
typedef UINT thread_id_t;

/* Thread States */
typedef enum
{
	THREAD_READY = 0, // Ready to run but not currently executing
	THREAD_RUNNING,   // Currently running
	THREAD_BLOCKED,   // Block by external factor (mutex, sem, ...)
	THREAD_SLEEPING,  // Timed sleep
	THREAD_DELETED,   // De-init
	THREAD_ERROR      // Generic error
} thread_state_e;

/* Thread Control Block */
typedef struct tcb_t
{
	// Thread address info
    UINT* thread_stack_ptr_;   // Stack pointer for the thread
    void (*thread_fn_)(void*); // Thread function pointer

    thread_state_e state_; // State of thread

    thread_id_t thread_id_; // ID of thread

    // Timing Control
    UINT thread_runtime_ms_;   // Runtime in milliseconds
    UINT thread_timeslice_ms_; // Time slice for the thread in milliseconds
    // UINT ui_sleep_time_; // Time thread have been sleeping

} tcb_t;

/* Thread Node Structure */
typedef struct thread_node_t
{
	tcb_t* thread_data_; //Pointer to thread-specific data
	struct thread_node_t* next_; //Pointer to the next node in the queue
} thread_node_t;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

#ifdef __cplusplus
}
#endif

#endif // __THREAD_H__
