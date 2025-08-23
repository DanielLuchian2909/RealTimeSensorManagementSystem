/**
 ********************************************************************************
 * @file itc_queue.c
 * @author Daniel Luchian
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <stdlib.h>
#include "itc_queue.h"

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
typedef struct thread_queue_arg_t
{
    msg_recv_queue_t* queue_;
    UCHAR num_operations_;
    UCHAR* num_successes_;
    char value_;
} thread_queue_arg_t;

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

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
CHAR
rtos_msgRecvInit( // Function to init a message receiving queue
    msg_recv_queue_t* msg_queue) // Pointer to message receiving queue to init
{
	msg_queue->head_ = NULL; // Dequeues at head
	msg_queue->tail_ = NULL; // Enqueues at tail

    if (!rtos_mutexInit(&msg_queue->queue_mutex_))
    {
    	return 0;
    }
    return -1;
}

//----------------------------------------------------------------------
CHAR // 0 if successful, non-zero otherwise
rtos_msgRecvEnqueue(     // Function to enqueue messages on a receiving queue
    msg_recv_queue_t* msg_queue,  // Pointer to message receiving queue to enqueue
    msg_container_t* msg) // Pointer to the message to enqueue
{
    // Return -1 if not given valid parameters
    if ((msg_queue == NULL) || (msg == NULL))
    {
        return -1;
    }

    msg->next_free_ = NULL;

    /* Enter Critical Section */
    rtos_mutexLock(&msg_queue->queue_mutex_);

    // Case where queue is empty
    if (msg_queue->tail_ == NULL)
    {
    	msg_queue->tail_ = msg;
    	msg_queue->head_ = msg;
    }

    // Remaining Case
    else
    {
    	msg_queue->tail_->next_free_ = msg;
    	msg_queue->tail_ = msg;
    }

    /* Exit Critical Section */
    rtos_mutexUnlock(&msg_queue->queue_mutex_);

    return 0;
}

//----------------------------------------------------------------------
msg_container_t* // Pointer
rtos_msgRecvDequeue(
    msg_recv_queue_t* msg_queue) // Pointer to message receiving queue to dequeue
{
    // Return -1 if not given a valid queue
    if (msg_queue == NULL)
    {
        return NULL;
    }

    /* Enter Critical Section */
    rtos_mutexLock(&msg_queue->queue_mutex_);

    // Case where queue is empty
    if (msg_queue->head_ == NULL)
    {
        /* Exit Critical Section */
        rtos_mutexUnlock(&msg_queue->queue_mutex_);
        return NULL;
    }

    // Dequeue from the head
    msg_container_t* msg_cont = msg_queue->head_;
    msg_queue->head_ = msg_cont->next_free_;
    msg_cont->next_free_ = NULL;

    // Handle queue becoming empty
    if (msg_queue->head_ == NULL)
    {
    	msg_queue->tail_ = NULL;
    }

    /* Exit Critical Section */
    rtos_mutexUnlock(&msg_queue->queue_mutex_);

    return msg_cont;
}
