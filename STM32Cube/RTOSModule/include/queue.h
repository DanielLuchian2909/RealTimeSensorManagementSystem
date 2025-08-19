/**
 ********************************************************************************
 * @file queue.h
 * @author Daniel Luchian
 * @brief Header for the queue file of the RTOS
 ********************************************************************************
 */

#ifndef __QUEUE_H__
#define __QUEUE_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "base_types.h" //Basic type definitions and utilities
#include "thread.h"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/
//RTOS Queue Structure
typedef struct thread_queue_t
{
	thread_node_t* ps_front_; //Pointer to the front of the queue
	thread_node_t* ps_rear_; //Poiner to the rear of the queue
	UINT ui_num_threads_; //Current number of threads in the queue
	UINT ui_max_num_threads_; //Maxium number of threads the queue can hold
} thread_queue_t;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
/* RTOS Queue Operations */
thread_queue_t* rtos_initQueue(UINT ui_max_num_threads); //Initialize an RTOSQueue
BOOLE rtos_enQueue(thread_queue_t* ps_thread_queue, thread_context_struct_t* ps_thread_data); //Enqueue a thread
thread_context_struct_t rtos_deQueue(thread_queue_t* ps_thread_queue); //Dequeue a thread

/* RTOS Queue Getters */
UINT rtos_getQueueSize(thread_queue_t* ps_thread_queue_);       //Returns the size of the queue
thread_node_t* rtos_peekQueue(thread_queue_t* ps_thread_queue); //Returns a pointer to the front of the queue
BOOLE rtos_isQueueEmpty(thread_queue_t* ps_thread_queue);       //Returns whether the queue is empty
BOOLE rtos_isQueueFull(thread_queue_t* ps_thread_queue);        //Returns whether the queue is full

#ifdef __cplusplus
}
#endif

#endif // __MY_FILE__
