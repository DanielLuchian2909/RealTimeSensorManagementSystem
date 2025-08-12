/**
 ********************************************************************************
 * @file queue.c
 * @author Daniel Luchian
 * @brief Implementation of the RTOS queue for managing thread contexts
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <stdlib.h>
#include "queue.h"

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
RTOSQueue* //A pointer to an RTOSQueue
rtos_initQueue( //Initializes an empty RTOSQueue
		UINT uiMaxNumThreads_) //An unsigned integer for the max number of threads
{
    //Dynamically allocate memory for the RTOSQueue
    RTOSQueue* psRTOSQueue = (RTOSQueue*)malloc(sizeof(RTOSQueue));
    if (psRTOSQueue == NULL)
    	return NULL; //Memory allocation failed

    //Initialize queue members
    psRTOSQueue->psFront = NULL;
	psRTOSQueue->psRear = NULL;
	psRTOSQueue->uiNumThreads = 0;
	psRTOSQueue->uiMaxNumThreads = uiMaxNumThreads_;

	return psRTOSQueue;
}

//-----------------------------------------------------------------------
BOOLE //Return true if the node is successfully added
rtos_enQueue( //Function to add a node to the RTOSQueue
		RTOSQueue* psRTOSQueue_, //Pointer to the RTOSQueue
		ThreadContextStruct* psThreadData_) //Value of the new thread data
{
	//Check if another thread is allowed to be enqueued
	if (rtos_isQueueFull(psRTOSQueue_))
		return FALSE; //Queue is full

	//Allocate a new node
	ThreadNode* psNewThreadNode = (ThreadNode*)malloc(sizeof(ThreadNode));
	if (psNewThreadNode == NULL)
		return FALSE; //Memory allocation failed

	//Initialize a new thread node
	psNewThreadNode->psThreadData = psThreadData_;
	psNewThreadNode->psNext = NULL;

	//If the queue is empty set the front to the new pointer, otherwise set the rear
	if (rtos_isQueueEmpty(psRTOSQueue_))
		psRTOSQueue_->psFront = psNewThreadNode;
	else
		psRTOSQueue_->psRear->psNext = psNewThreadNode;

	//Set the rear to the new pointer, and make it point to the front, and update number of threads in the queue
	psRTOSQueue_->psRear = psNewThreadNode;
	psRTOSQueue_->psRear->psNext = psRTOSQueue_->psFront;
	psRTOSQueue_->uiNumThreads++;

	return TRUE;
}

//-----------------------------------------------------------------------
ThreadContextStruct //Returns the dequeued node
rtos_deQueue( //A function that dequeues a nodes from the queue
		RTOSQueue* psRTOSQueue_) //A pointer an RTOSQueue
{
	//Init dequeued val with dummy results
	ThreadContextStruct sDequeuedValue;
	sDequeuedValue.pfnMyThreadFunction = NULL;
	sDequeuedValue.puiMyThreadStackPointer = NULL;

	//Check if the queue is empty
	if (rtos_isQueueEmpty(psRTOSQueue_))
		return sDequeuedValue;

	//Handle single-node and multi-node cases
	if (psRTOSQueue_->psFront == psRTOSQueue_->psRear)
	{
		//Get then free dequeued value
		sDequeuedValue = *(psRTOSQueue_->psFront->psThreadData);

		//Free dynamically allocated psThreadData if necessary
		if (psRTOSQueue_->psFront != NULL)
		 	free(psRTOSQueue_->psFront->psThreadData);

		free(psRTOSQueue_->psFront);

		//Update the queue information
		psRTOSQueue_->psFront = NULL;
		psRTOSQueue_->psRear = NULL;
	}
	else
	{
		//Create a temp node and set it to the front of the queue
		ThreadNode* psTemp = psRTOSQueue_->psFront;

		//Get the dequeued value
		sDequeuedValue = *(psTemp->psThreadData);

		//Free dynamically allocated pSThreadData if necessary
		if (psRTOSQueue_->psFront != NULL)
			free(psRTOSQueue_->psFront->psThreadData);

		//Update the queue
		psRTOSQueue_->psFront = psRTOSQueue_->psFront->psNext;
		psRTOSQueue_->psRear->psNext = psRTOSQueue_->psFront;

		//Free the temp
		free(psTemp);
	}

	//Update size of queue
	psRTOSQueue_->uiNumThreads--;

	return sDequeuedValue;
}

//-----------------------------------------------------------------------
UINT //The number of threads in the RTOS Queue
rtos_getQueueSize( //A function that returns the size of the RTOSQueue
		RTOSQueue* psRTOSQueue_) //A pointer to an RTOSQueue
{
	return (psRTOSQueue_->uiNumThreads);
}

//-----------------------------------------------------------------------
ThreadNode* //A pointer to a ThreadNode
rtos_peekQueue( //A function that returns the first value of an RTOSQueue
		RTOSQueue* psRTOSQueue_) //A pointer to an RTOSQueue
{
	return (psRTOSQueue_->psFront);
}

//-----------------------------------------------------------------------
BOOLE //True if the queue is empty, otherwise false
rtos_isQueueEmpty( //A function that returns whether the RTOSQueue is empty
		RTOSQueue* psRTOSQueue_) //A pointer to an RTOSQueue
{
	return (psRTOSQueue_->psFront == NULL);
}

//-----------------------------------------------------------------------
BOOLE //True if the queue is full, otherwise false
rtos_isQueueFull( //A function that returns whether the queue is full
		RTOSQueue* psRTOSQueue_) //A pointer to an RTOSQueue
{
	return (psRTOSQueue_->uiNumThreads == psRTOSQueue_->uiMaxNumThreads);
}



