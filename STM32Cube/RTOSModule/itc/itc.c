/**
 ********************************************************************************
 * @file itc.c
 * @author Daniel Luchian
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "lk_itc.h"
#include "itc_queue.h"
#include "utilities.h"
#include <string.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define MAX_NUM_MESSAGES     100
#define MAX_THREAD_ID        255
#define NUM_RECV_QUEUES      (MAX_THREAD_ID + 1) // Includes spot for thread id 0

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
/* Structure Encapsulating all Messaging Data Structures*/
typedef struct msg_system_t
{
    msg_container_t pool_[MAX_NUM_MESSAGES];      // Memory pool for all messages
    msg_container_t* next_free_;                  // Pointer to first free message
    mutex_t pool_mutex_;                          // Mutex to protect the message pool
    msg_recv_queue_t receivers_[NUM_RECV_QUEUES]; // Array of receiving queues
} msg_system_t;

/************************************
 * STATIC VARIABLES
 ************************************/
static msg_system_t msg_system;

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static CHAR initMsgSystem(void);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static CHAR // 0 if successful, non-zero otherwise
initMsgSystem(void) // Function to initialize the message library data structures
{
    // Initialize all messages in the free list
    for (size_t i = 0; i < MAX_NUM_MESSAGES - 1; i++)
    {
        msg_system.pool_[i].next_free_ = &msg_system.pool_[i + 1];
    }
    msg_system.pool_[MAX_NUM_MESSAGES - 1].next_free_ = NULL;
    msg_system.next_free_ = &msg_system.pool_[0];

    // Initialize message pool mutex
    if (lk_mutexInit(&msg_system.pool_mutex_) != 0)
    {
        return -1;
    }

    // Initialize the receiving queue for every thread
    for (size_t i = 0; i < NUM_RECV_QUEUES; i++)
    {
        if (rtos_msgRecvInit(&msg_system.receivers_[i]) != 0)
        {
            return -1;
        }
    }

    return 0;
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
CHAR
rtos_itcInit()
{
	return initMsgSystem();
}

//-----------------------------------------------------------------------
msg_t*    // Pointer to returned message structure
lk_newMsg() // Function that returns a message structure if available, NULL otherwise
{
    /* Enter Critical Section */
    lk_mutexLock(&msg_system.pool_mutex_);

    // Check if free messages are available
    if (msg_system.next_free_ == NULL)
    {
        lk_mutexUnlock(&msg_system.pool_mutex_);
        return NULL;
    }

    // Use the next available free message, and update the free message list
    msg_container_t* temp_msg_container = msg_system.next_free_;
    msg_system.next_free_ = temp_msg_container->next_free_;
    temp_msg_container->next_free_ = NULL;

    /* Exit Critical Section */
    lk_mutexUnlock(&msg_system.pool_mutex_);

    return &(temp_msg_container->msg_);
}

//-----------------------------------------------------------------------
void
lk_deleteMsg(     // Function that returns (deletes) given msg structure
    msg_t* msg) // Pointer to msg structure to delete
{
    // Nothing to delete if no msg was given
    VOID_NULL_PTR_CHECK(msg);

    /* Enter Critical Section */
    lk_mutexLock(&msg_system.pool_mutex_);

    // Wipe msg
    memset(msg, 0, sizeof(msg_t));

    // Put msg in free list
    msg_container_t* container_to_del = (msg_container_t*)( (char*)msg - offsetof(msg_container_t, msg_) );
    container_to_del->next_free_ = msg_system.next_free_;
    msg_system.next_free_ = container_to_del;

    /* Exit Critical Section */
    lk_mutexUnlock(&msg_system.pool_mutex_);
}

//-----------------------------------------------------------------------
CHAR   // 0 if successfully sent, non-zero otherwise
lk_sendMsg( // Function that sends a msg to another thread
    UCHAR dest_id, // Destination id of thread to deliver msg to
    msg_t* msg)         // Ptr to msg to deliver
{
    // Check for msg
    SIGNED_NULL_PTR_CHECK(msg);

    // Send the msg
    msg_container_t* container_to_send = (msg_container_t*)((CHAR*)msg - offsetof(msg_container_t, msg_));
    return rtos_msgRecvEnqueue(&msg_system.receivers_[dest_id], container_to_send);
}

//-----------------------------------------------------------------------
CHAR   // 0 if successfully received, non-zero otherwise
lk_recvMsg( // Function that received any pending incoming msgs
    UCHAR recv_id, // Thread ID of receiver
    msg_t* msg)      // Ptr to buffer for received msg
{
    SIGNED_NULL_PTR_CHECK(msg);

    // Receive the msg
    msg_container_t* container_to_recv = rtos_msgRecvDequeue(&msg_system.receivers_[recv_id]);

    SIGNED_NULL_PTR_CHECK(container_to_recv);

    memcpy(msg, &container_to_recv->msg_, sizeof(msg_t));
    lk_deleteMsg(&container_to_recv->msg_);

    return 0;
}

