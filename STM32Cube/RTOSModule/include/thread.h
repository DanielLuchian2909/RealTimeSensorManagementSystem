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

/************************************
 * TYPEDEFS
 ************************************/
/* Thread Context Structure */
typedef struct thread_context_struct_t
{
    UINT* pui_thread_stack_ptr_;  // Stack pointer for the thread
    void (*pfn_thread_fn_)(void*); // Thread function pointer
    UINT ui_thread_runtime_ms_; // Runtime in milliseconds
    UINT ui_thread_timeslice_ms_; // Time slice for the thread in milliseconds
} thread_context_struct_t;

/* Thread Node Structure */
typedef struct thread_node_t
{
	thread_context_struct_t* ps_thread_data_; //Pointer to thread-specific data
	struct thread_node_t* ps_next_; //Pointer to the next node in the queue
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
