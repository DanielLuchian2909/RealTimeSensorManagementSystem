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

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define MAX_NUM_MESSAGES     1000
#define MAX_THREAD_ID        255
#define NUM_RECV_QUEUES      (MAX_THREAD_ID + 1) // Includes spot for thread id 0

#define SUCCESSFULLY_INIT    0x0U
#define NOT_INIT             0x1U
#define FAILED_INIT          0x2U

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
/* Structure Encapsulating all Messaging Data Structures*/
typedef struct msg_system_t
{
    msg_container_t pool_[MAX_NUM_MESSAGES];      // Memory pool for all messages
    msg_container_t* next_free_;                  // Ptr to first free message
    mutex_t pool_mutex_;                          // Mutex to protect the message pool
    msg_recv_queue_t receivers_[NUM_RECV_QUEUES]; // Array of receiving queues
} msg_system_t;

/************************************
 * STATIC VARIABLES
 ************************************/
static msg_system_t msg_system;
static UCHAR msg_system_status = NOT_INIT;

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static void initMsgSystem(void);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void // 0 if successful, non-zero otherwise
initMsgSystem(void) // Function to initialize the message library data structures
{
    msg_system_status = SUCCESSFULLY_INIT;

    // Initialize all messages in the free list
    for (size_t i = 0; i < MAX_NUM_MESSAGES - 1; i++)
    {
        msg_system.pool_[i].next_free_ = &msg_system.pool_[i + 1];
    }
    msg_system.pool_[MAX_NUM_MESSAGES - 1].next_free_ = NULL;
    msg_system.next_free_ = &msg_system.pool_[0];

    // Initialize message pool mutex
    if (rtos_mutexInit(&msg_system.pool_mutex_) != 0)
    {
        msg_system_status = FAILED_INIT;
    }

    // Initialize the receiving queue for every thread
    for (size_t i = 0; i < NUM_RECV_QUEUES; i++)
    {
        if (rtos_msgRecvInit(&msg_system.receivers_[i]) != 0)
        {
            msg_system_status = FAILED_INIT;
        }
    }
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
msg_t*    // Pointer to returned message structure
lk_newMsg() // Function that returns a message structure if available, NULL otherwise
{
    // Check system is running
    if (msg_system_status != SUCCESSFULLY_INIT)
    {
        return NULL;
    }

    /* Enter Critical Section */
    rtos_mutexLock(&msg_system.pool_mutex_);

    // Check if free messages are available
    if (msg_system.next_free_ == NULL)
    {
        rtos_mutexUnlock(&msg_system.pool_mutex_);
        return NULL;
    }

    // Use the next available free message, and update the free message list
    msg_container_t* temp_msg_container = msg_system.next_free_;
    msg_system.next_free_ = temp_msg_container->next_free_;
    temp_msg_container->next_free_ = NULL;

    /* Exit Critical Section */
    rtos_mutexUnlock(&msg_system.pool_mutex_);

    return &(temp_msg_container->msg_);
}

//-----------------------------------------------------------------------
void
lk_deletemsg(     // Function that returns (deletes) given msg structure
    msg_t* msg) // Pointer to msg structure to delete
{
    // Check system is running
    if (msg_system_status != SUCCESSFULLY_INIT)
    {
        return;
    }

    // Nothing to delete if no msg was given
    if (msg == NULL)
    {
        return;
    }

    /* Enter Critical Section */
    rtos_mutexLock(&msg_system.pool_mutex_);

    // Wipe msg
    memset(msg, 0, sizeof(msg_t));

    // Put msg in free list
    msg_container_t* container_to_del = (msg_container_t*)( (char*)msg - offsetof(msg_container_t, msg_) );
    container_to_del->next_free_ = msg_system.next_free_;
    msg_system.next_free_ = container_to_del;

    /* Exit Critical Section */
   rtos_mutexUnlock(&msg_system.pool_mutex_);
}

//-----------------------------------------------------------------------
INT   // 0 if successfully sent, non-zero otherwise
lk_sendMsg( // Function that sends a msg to another thread
    UCHAR dest_id, // Destination id of thread to deliver msg to
    msg_t* msg)         // Ptr to msg to deliver
{
    // Check system is running
    if (msg_system_status != SUCCESSFULLY_INIT)
    {
        return msg_system_status;
    }

    // Check for msg
    if (msg == NULL)
    {
        return -1;
    }

    // Send the msg
    msg_container_t* container_to_send = (msg_container_t*)((CHAR*)msg - offsetof(msg_container_t, msg_));
    return rtos_msgRecvEnqueue(&msg_system.receivers_[dest_id], container_to_send);
}

//-----------------------------------------------------------------------
INT   // 0 if successfully received, non-zero otherwise
lk_recvMsg( // Function that received any pending incoming msgs
    UCHAR recv_id, // Thread ID of receiver
    msg_t* msg)      // Ptr to buffer for received msg
{
    // Check system is running
    if (msg_system_status != SUCCESSFULLY_INIT)
    {
        return msg_system_status;
    }

    // Check if valid msg given
    if (msg == NULL)
    {
        return -1;
    }

    // Receive the msg
    msg_container_t* container_to_recv = rtos_msgRecvDequeue(&msg_system.receivers_[recv_id]);
    if (container_to_recv == NULL)
    {
        return -1;
    }
    memcpy(msg, &container_to_recv->msg_, sizeof(msg_t));
    rtos_deleteMsg(&container_to_recv->msg_);

    return 0;
}

