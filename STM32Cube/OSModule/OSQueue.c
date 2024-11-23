/**
 ********************************************************************************
 * @file queue.c
 * @author
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "OSQueue.h"
#include <stdlib.h>
#include <stddef.h>

/************************************
 * EXTERN VARIABLES AND FUNCTIONS
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
OSQueue*
os_InitOSQueue( //Initializes an empty OSQueue
		UINT uiMaxNumThreads_) //A unsigned integer for the max number of threads
{
    //Dynamically allocate memory for the OSQueue
    OSQueue* psOSQueue = (OSQueue*)malloc(sizeof(OSQueue));

    if (psOSQueue == NULL)
    	return NULL;

    psOSQueue->psFront = NULL;
	psOSQueue->psRear = NULL;
	psOSQueue->uiNumThreads = 0;
	psOSQueue->uiMaxNumThreads = uiMaxNumThreads_;

	return psOSQueue;
}

//-----------------------------------------------------------------------
BOOLE //Return true if the node is successfully added
os_EnOSQueue( //Function to add a node to the OSQueue
		OSQueue* psOSQueue_, //Pointer to the OSQueue
		ThreadContextStruct* psThreadData_) //Value of the new thread node
{
	//Check if another thread is allowed to be enqueued
	if (os_IsOSQueueFull(psOSQueue_))
		return FALSE;

	//Allocate a new node
	ThreadNode* psNewThreadNode = (ThreadNode*)malloc(sizeof(ThreadNode));

	//Check if the node was allocated properly
	if (!psNewThreadNode)
		return FALSE;

	psNewThreadNode->psThreadData = psThreadData_;

	//If the queue is empty set the front to the new pointer, otherwise set the rear
	if (os_IsOSQueueEmpty(psOSQueue_))
		psOSQueue_->psFront = psNewThreadNode;
	else
		psOSQueue_->psRear->psNext = psNewThreadNode;

	//Set the rear to the new pointer, and make it point to the front, and update number of threads in the queue
	psOSQueue_->psRear = psNewThreadNode;
	psOSQueue_->psRear->psNext = psOSQueue_->psFront;
	psOSQueue_->uiNumThreads++;

	return TRUE;
}

//-----------------------------------------------------------------------
ThreadContextStruct
os_DeOSQueue(
		OSQueue* psOSQueue_)
{
	//Init dequeued val with dummy results
	ThreadContextStruct sDequeuedValue;
	sDequeuedValue.pfnMyThreadFunction = NULL;
	sDequeuedValue.puiMyThreadStackPointer = NULL;

	//Check if the queue is empty
	if (os_IsOSQueueEmpty(psOSQueue_))
		return sDequeuedValue;

	//If there is one node in the queue remove it, and update the queue information,
	//If there are multple nodes, same process but with the last node
	if (psOSQueue_->psFront == psOSQueue_->psRear)
	{
		//Get then free? the dequeued value
		sDequeuedValue = *(psOSQueue_->psFront->psThreadData);
		free(psOSQueue_->psFront);

		//Update the queue information
		psOSQueue_->psFront = NULL;
		psOSQueue_->psFront = NULL;
	}
	else
	{
		//Create a temp node and set it to the front of the queue
		ThreadNode* psTemp = psOSQueue_->psFront;

		//Get the dequeued value
		sDequeuedValue = *(psTemp->psThreadData);

		//Update the queue
		psOSQueue_->psFront = psOSQueue_->psFront->psNext;
		psOSQueue_->psRear->psNext = psOSQueue_->psFront;

		//Free the temp
		free(psTemp);
	}

	//Update size of queue
	psOSQueue_->uiNumThreads--;

	return sDequeuedValue;
}

//-----------------------------------------------------------------------
UINT
os_GetOSQueueSize(
		OSQueue* psOSQueue_)
{
	return (psOSQueue_->uiNumThreads);
}

//-----------------------------------------------------------------------
ThreadNode*
os_PeekOSQueue(
		OSQueue* psOSQueue_)
{
	return (psOSQueue_->psFront);
}

//-----------------------------------------------------------------------
BOOLE
os_IsOSQueueEmpty(
		OSQueue* psOSQueue_)
{
	return (psOSQueue_->psFront == NULL);
}

//-----------------------------------------------------------------------
BOOLE
os_IsOSQueueFull(
		OSQueue* psOSQueue_)
{
	return (psOSQueue_->uiNumThreads == psOSQueue_->uiMaxNumThreads);
}



