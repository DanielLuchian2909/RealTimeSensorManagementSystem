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
#include "itc.h"
#include "itc_queue.h"

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define MAX_NUM_MESSAGES         1000
#define MAX_THREAD_ID        UINT8_MAX
#define NUM_RECEIVER_QUEUES  (MAX_THREAD_ID + 1) // Includes spots for thread id 0

#define SUCCESSFULLY_INIT    0x0U
#define NOT_INIT             0x1U
#define FAILED_INIT          0x2U

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
/* Struct Encapsulating all Messaging Data Structures*/
typedef struct msg_system_t
{
    msg_container_t pool_[MAX_NUM_MESSAGES]; // Memory pool for all messages
    msg_container_t* next_free_;         // Ptr to first free message
    mutex_t pool_mutex_;             // Mutex to protect the message pool
    msg_recv_queue_t receivers_[NUM_RECEIVER_QUEUES]; // Array of receiving queues
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
static void init_message_system(void);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void // 0 if successful, non-zero otherwise
init_message_system(void) // Function to init the message library data structures
{
    message_system_status = SUCCESSFULLY_INIT;

    // Init all messages in the free list
    for (size_t i = 0; i < MAX_NUM_MESSAGES - 1; i++)
    {
        message_system.pool_[i].next_free_ = &message_system.pool_[i + 1];
    }
    message_system.pool_[MAX_NUM_MESSAGES - 1].next_free_ = NULL;
    message_system.next_free_ = &message_system.pool_[0];

    // Init message pool mutex
    if (pthread_mutex_init(&message_system.pool_mutex_, NULL) != 0)
    {
        message_system_status = FAILED_INIT;
    }

    // Init the receiving queue for every thread
    for (size_t i = 0; i < NUM_RECEIVER_QUEUES; i++)
    {
        if (message_receiver_init(&message_system.receivers_[i]) != 0)
        {
            message_system_status = FAILED_INIT;
        }
    }
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
message_t*    // Ptr to returned message struct
new_message() // Function that returns a message struct if available, NULL otherwise
{
    // If first time running the function, init the message system
    pthread_once(&message_system.init_once_, init_message_system);

    // Check system is running
    if (message_system_status != SUCCESSFULLY_INIT)
    {
        return NULL;
    }

    /* Enter Crit Section */
    pthread_mutex_lock(&message_system.pool_mutex_);

    // Check if free messages are available
    if (message_system.next_free_ == NULL)
    {
        pthread_mutex_unlock(&message_system.pool_mutex_);
        return NULL;
    }

    // Use the next available free message, and update the free message list
    message_container_t* temp_message_container = message_system.next_free_;
    message_system.next_free_ = temp_message_container->next_free_;
    temp_message_container->next_free_ = NULL;

    /* Exit Crit Section */
    pthread_mutex_unlock(&message_system.pool_mutex_);

    return &(temp_message_container->message_);
}

//-----------------------------------------------------------------------
void
delete_message(     // Function that returns (deletes) given message struct
    message_t* msg) // Ptr to message struct to delete
{
    // If first time running the function, init the message system
    pthread_once(&message_system.init_once_, init_message_system);

    // Check system is running
    if (message_system_status != SUCCESSFULLY_INIT)
    {
        return;
    }

    // Nothing to delete if no message was given
    if (msg == NULL)
    {
        return;
    }

    /* Enter Crit Section */
    pthread_mutex_lock(&message_system.pool_mutex_);

    // Wipe message
    memset(msg, 0, sizeof(message_t));

    // Put message in free list
    message_container_t* container_to_del = (message_container_t*)( (char*)msg - offsetof(message_container_t, message_) );
    container_to_del->next_free_ = message_system.next_free_;
    message_system.next_free_ = container_to_del;

    /* Exit Crit Section */
   pthread_mutex_unlock(&message_system.pool_mutex_);
}

//-----------------------------------------------------------------------
int   // 0 if successfully sent, non-zero otherwise
send( // Function that sends a message to another thread
    uint8_t destination_id, // Destination id of thread to deliver message to
    message_t* msg)         // Ptr to message to deliver
{
    // If first time running the function, init the message system
    pthread_once(&message_system.init_once_, init_message_system);

    // Check system is running
    if (message_system_status != SUCCESSFULLY_INIT)
    {
        return message_system_status;
    }

    // Check for msg
    if (msg == NULL)
    {
        return -1;
    }

    // Send the message
    message_container_t* container_to_send = (message_container_t*)((char*)msg - offsetof(message_container_t, message_));
    return msg_recv_enqueue(&message_system.receivers_[destination_id], container_to_send);
}

//-----------------------------------------------------------------------
int   // 0 if successfully received, non-zero otherwise
recv( // Function that received any pending incoming messages
    UCHAR receiver_id, // Thread ID of receiver
    msg_t* msg)      // Ptr to buffer for received message
{
    // Check system is running
    if (message_system_status != SUCCESSFULLY_INIT)
    {
        return message_system_status;
    }

    // Check if valid message given
    if (msg == NULL)
    {
        return -1;
    }

    // Receive the message
    message_container_t* container_to_recv = msg_receiver_dequeue(&message_system.receivers_[receiver_id]);
    if (container_to_recv == NULL)
    {
        return -1;
    }
    memcpy(msg, &container_to_recv->message_, sizeof(message_t));
    delete_msg(&container_to_recv->message_);

    return 0;
}

