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
#include "thread_queue.h"

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
thread_queue_t* //A pointer to an thread_queue_t
rtos_initQueue( //Initializes an empty thread_queue_t
		UINT max_num_threads) //An unsigned integer for the max number of threads
{
    //Dynamically allocate memory for the thread_queue_t
    thread_queue_t* thread_queue = (thread_queue_t*)malloc(sizeof(thread_queue_t));
    if (thread_queue == NULL)
    	return NULL; //Memory allocation failed

    //Initialize queue members
    thread_queue->front_ = NULL;
	thread_queue->rear_ = NULL;
	thread_queue->num_threads_ = 0;
	thread_queue->max_num_threads_ = max_num_threads;

	return thread_queue;
}

//-----------------------------------------------------------------------
BOOLE //Return true if the node is successfully added
rtos_enQueue( //Function to add a node to the thread_queue_t
		thread_queue_t* thread_queue, //Pointer to the thread_queue_t
		tcb_t* thread_data) //Value of the new thread data
{
	//Check if another thread is allowed to be enqueued
	if (rtos_isQueueFull(thread_queue))
	{
		return FALSE; //Queue is full
	}

	//Allocate a new node
	thread_node_t* new_thread_node = (thread_node_t*)malloc(sizeof(thread_node_t));
	if (new_thread_node == NULL)
	{
		return FALSE; //Memory allocation failed
	}

	//Initialize a new thread node
	new_thread_node->thread_data_ = thread_data;
	new_thread_node->next_ = NULL;

	//If the queue is empty set the front to the new pointer, otherwise set the rear
	if (rtos_isQueueEmpty(thread_queue))
	{
		thread_queue->front_ = new_thread_node;
	}
	else
	{
		thread_queue->rear_->next_ = new_thread_node;
	}

	//Set the rear to the new pointer, and make it point to the front, and update number of threads in the queue
	thread_queue->rear_ = new_thread_node;
	thread_queue->rear_->next_ = thread_queue->front_;
	thread_queue->num_threads_++;

	return TRUE;
}

//-----------------------------------------------------------------------
tcb_t //Returns the dequeued node
rtos_deQueue( //A function that dequeues a nodes from the queue
		thread_queue_t* thread_queue) //A pointer an thread_queue_t
{
	//Init dequeued val with dummy results
	tcb_t dequeued_val;
	dequeued_val.thread_fn_ = NULL;
	dequeued_val.thread_stack_ptr_ = NULL;

	//Check if the queue is empty
	if (rtos_isQueueEmpty(thread_queue))
		return dequeued_val;

	//Handle single-node and multi-node cases
	if (thread_queue->front_ == thread_queue->rear_)
	{
		//Get then free dequeued value
		dequeued_val = *(thread_queue->front_->thread_data_);

		//Free dynamically allocated ThreadData if necessary
		if (thread_queue->front_ != NULL)
		 	free(thread_queue->front_->thread_data_);

		free(thread_queue->front_);

		//Update the queue information
		thread_queue->front_ = NULL;
		thread_queue->rear_ = NULL;
	}
	else
	{
		//Create a temp node and set it to the front of the queue
		thread_node_t* temp = thread_queue->front_;

		//Get the dequeued value
		dequeued_val = *(temp->thread_data_);

		//Free dynamically allocated ThreadData if necessary
		if (thread_queue->front_ != NULL)
			free(thread_queue->front_->thread_data_);

		//Update the queue
		thread_queue->front_ = thread_queue->front_->next_;
		thread_queue->rear_->next_ = thread_queue->front_;

		//Free the temp
		free(temp);
	}

	//Update size of queue
	thread_queue->num_threads_--;

	return dequeued_val;
}

//-----------------------------------------------------------------------
UINT //The number of threads in the RTOS Queue
rtos_getQueueSize( //A function that returns the size of the thread_queue_t
		thread_queue_t* thread_queue) //A pointer to an thread_queue_t
{
	return (thread_queue->num_threads_);
}

//-----------------------------------------------------------------------
thread_node_t* //A pointer to a thread_node_t
rtos_peekQueue( //A function that returns the first value of an thread_queue_t
		thread_queue_t* thread_queue) //A pointer to an thread_queue_t
{
	return (thread_queue->front_);
}

//-----------------------------------------------------------------------
BOOLE //True if the queue is empty, otherwise false
rtos_isQueueEmpty( //A function that returns whether the thread_queue_t is empty
		thread_queue_t* thread_queue) //A pointer to an thread_queue_t
{
	return (thread_queue->front_ == NULL);
}

//-----------------------------------------------------------------------
BOOLE //True if the queue is full, otherwise false
rtos_isQueueFull( //A function that returns whether the queue is full
		thread_queue_t* thread_queue) //A pointer to an thread_queue_t
{
	return (thread_queue->num_threads_ == thread_queue->max_num_threads_);
}



