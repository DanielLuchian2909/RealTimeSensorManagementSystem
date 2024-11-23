/**
 ********************************************************************************
 * @file queue.h
 * @date
 * @brief
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
#include "BaseTypes.h"
#include "Kernel.h"
#include <stddef.h>

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

//Thread Node Structure
typedef struct
{
	ThreadContextStruct* psThreadData;
	struct ThreadNode* psNext;
}
ThreadNode;

//OSQueue Structure
typedef struct
{
	ThreadNode* psFront;
	ThreadNode* psRear;
	UINT uiNumThreads;
	UINT uiMaxNumThreads;
}
OSQueue;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
//OSQueue Functions
OSQueue* os_InitOSQueue(UINT uiMaxNumThreads_);

//OSQueue Operations
BOOLE os_EnOSQueue(OSQueue* psOSQueue_, ThreadContextStruct* psThreadData_);
ThreadContextStruct os_DeOSQueue(OSQueue* psOSQueue_);

//OSQueue Getters
UINT os_GetOSQueueSize(OSQueue* psOSQueue_);
ThreadNode* os_PeekOSQueue(OSQueue* psOSQueue_);
BOOLE os_IsOSQueueEmpty(OSQueue* psOSQueue_);
BOOLE os_IsOSQueueFull(OSQueue* psOSQueue_);

#ifdef __cplusplus
}
#endif

#endif // __MY_FILE__
