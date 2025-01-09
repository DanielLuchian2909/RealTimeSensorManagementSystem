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
#include "baseTypes.h" //Basic type definitions and utilities

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

//Thread Node Structure
typedef struct ThreadNode
{
	ThreadContextStruct* psThreadData; //Pointer to thread-specific data
	struct ThreadNode* psNext; //Pointer to the next node in the queue
} ThreadNode;

//RTOS Queue Structure
typedef struct RTOSQueue
{
	ThreadNode* psFront; //Pointer to the front of the queue
	ThreadNode* psRear; //Poiner to the rear of the queue
	UINT uiNumThreads; //Current number of threads in the queue
	UINT uiMaxNumThreads; //Maxium number of threads the queue can hold
} RTOSQueue;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
/* RTOS Queue Operations */
RTOSQueue* rtos_InitQueue(UINT uiMaxNumThreads_); //Initialize an RTOSQueue
BOOLE rtos_EnQueue(RTOSQueue* psRTOSQueue_, ThreadContextStruct* psThreadData_); //Enqueue a thread
ThreadContextStruct rtos_DeQueue(RTOSQueue* psRTOSQueue_); //Dequeue a thread

/* RTOS Queue Getters */
UINT rtos_GetQueueSize(RTOSQueue* psRTOSQueue_); //Returns the size of the queue
ThreadNode* rtos_PeekQueue(RTOSQueue* psRTOSQueue_); //Returns a pointer to the front of the queue
BOOLE rtos_IsQueueEmpty(RTOSQueue* psRTOSQueue_); //Returns whether the queue is empty
BOOLE rtos_IsQueueFull(RTOSQueue* psRTOSQueue_); //Returns whether the queue is full

#ifdef __cplusplus
}
#endif

#endif // __MY_FILE__
