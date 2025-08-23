/**
 ********************************************************************************
 * @file itc_queue.h
 * @author Daniel Luchian
 * @brief Header for inter-process communication interface
 ********************************************************************************
 */

#ifndef __ITC_QUEUE_H__
#define __ITC_QUEUE_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "itc_types.h"
#include "lk_mutex.h"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/
/* Message Container Structure */
typedef struct msg_container_t
{
    msg_t msg_;                         // Message
    struct msg_container_t* next_free_; // Pointer to the next free message (for O(1) access)
} msg_container_t;

/* Message Queue Structure */
typedef struct msg_recv_queue_t
{
    msg_container_t* head_; // Pointer to head of queue (dequeue)
    msg_container_t* tail_; // Pointer to tail of queue (enqueue)
    mutex_t queue_mutex_;   // Queue mutex
} msg_recv_queue_t;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
CHAR rtos_msgRecvInit(msg_recv_queue_t* msg_queue);
CHAR rtos_msgRecvEnqueue(msg_recv_queue_t* msg_queue, msg_container_t* msg);
msg_container_t* rtos_msgRecvDequeue(msg_recv_queue_t* msg_queue);

#ifdef __cplusplus
}
#endif

#endif // __ITC_QUEUE_H__
