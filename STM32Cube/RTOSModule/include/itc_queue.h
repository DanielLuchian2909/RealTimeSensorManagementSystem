/**
 ********************************************************************************
 * @file
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __itc_queue__
#define __itc_queue__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "itc_types.h"
#include "mutex.h"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/
/* Message Container Structure */
typedef struct msg_container_t
{
    msg_t msg_;                     // Message
    struct msg_container_t* next_free_; // Ptr to the next free message (for O(1) access)
} msg_container_t;

/* Message Queue Structure */
typedef struct msg_recv_queue_t
{
    msg_container_t* head_;
    msg_container_t* tail_;
    mutex_t queue_mutex_;
} msg_recv_queue_t;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
int msg_recv_init(msg_recv_queue_t* q);
int msg_recv_enqueue(msg_recv_queue_t* q, msg_container_t* msg);
msg_container_t* msg_receiver_dequeue(msg_recv_queue_t* q);

#ifdef __cplusplus
}
#endif

#endif // __itc_queue___
