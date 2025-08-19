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
thread_queue_t* //A pointer to an thread_queue_t
rtos_initQueue( //Initializes an empty thread_queue_t
		UINT ui_max_num_threads) //An unsigned integer for the max number of threads
{
    //Dynamically allocate memory for the thread_queue_t
    thread_queue_t* ps_thread_queue = (thread_queue_t*)malloc(sizeof(thread_queue_t));
    if (ps_thread_queue == NULL)
    	return NULL; //Memory allocation failed

    //Initialize queue members
    ps_thread_queue->ps_front_ = NULL;
	ps_thread_queue->ps_rear_ = NULL;
	ps_thread_queue->ui_num_threads_ = 0;
	ps_thread_queue->ui_max_num_threads_ = ui_max_num_threads;

	return ps_thread_queue;
}

//-----------------------------------------------------------------------
BOOLE //Return true if the node is successfully added
rtos_enQueue( //Function to add a node to the thread_queue_t
		thread_queue_t* ps_thread_queue, //Pointer to the thread_queue_t
		thread_context_struct_t* ps_thread_data) //Value of the new thread data
{
	//Check if another thread is allowed to be enqueued
	if (rtos_isQueueFull(ps_thread_queue))
	{
		return FALSE; //Queue is full
	}

	//Allocate a new node
	thread_node_t* ps_new_thread_node = (thread_node_t*)malloc(sizeof(thread_node_t));
	if (ps_new_thread_node == NULL)
	{
		return FALSE; //Memory allocation failed
	}

	//Initialize a new thread node
	ps_new_thread_node->ps_thread_data_ = ps_thread_data;
	ps_new_thread_node->ps_next_ = NULL;

	//If the queue is empty set the front to the new pointer, otherwise set the rear
	if (rtos_isQueueEmpty(ps_thread_queue))
	{
		ps_thread_queue->ps_front_ = ps_new_thread_node;
	}
	else
	{
		ps_thread_queue->ps_rear_->ps_next_ = ps_new_thread_node;
	}

	//Set the rear to the new pointer, and make it point to the front, and update number of threads in the queue
	ps_thread_queue->ps_rear_ = ps_new_thread_node;
	ps_thread_queue->ps_rear_->ps_next_ = ps_thread_queue->ps_front_;
	ps_thread_queue->ui_num_threads_++;

	return TRUE;
}

//-----------------------------------------------------------------------
thread_context_struct_t //Returns the dequeued node
rtos_deQueue( //A function that dequeues a nodes from the queue
		thread_queue_t* psthread_queue_t_) //A pointer an thread_queue_t
{
	//Init dequeued val with dummy results
	thread_context_struct_t s_dequeued_val;
	s_dequeued_val.pfn_thread_fn_ = NULL;
	s_dequeued_val.pui_thread_stack_ptr_ = NULL;

	//Check if the queue is empty
	if (rtos_isQueueEmpty(psthread_queue_t_))
		return s_dequeued_val;

	//Handle single-node and multi-node cases
	if (psthread_queue_t_->ps_front_ == psthread_queue_t_->ps_rear_)
	{
		//Get then free dequeued value
		s_dequeued_val = *(psthread_queue_t_->ps_front_->ps_thread_data_);

		//Free dynamically allocated psThreadData if necessary
		if (psthread_queue_t_->ps_front_ != NULL)
		 	free(psthread_queue_t_->ps_front_->ps_thread_data_);

		free(psthread_queue_t_->ps_front_);

		//Update the queue information
		psthread_queue_t_->ps_front_ = NULL;
		psthread_queue_t_->ps_rear_ = NULL;
	}
	else
	{
		//Create a temp node and set it to the front of the queue
		thread_node_t* ps_temp = psthread_queue_t_->ps_front_;

		//Get the dequeued value
		s_dequeued_val = *(ps_temp->ps_thread_data_);

		//Free dynamically allocated pSThreadData if necessary
		if (psthread_queue_t_->ps_front_ != NULL)
			free(psthread_queue_t_->ps_front_->ps_thread_data_);

		//Update the queue
		psthread_queue_t_->ps_front_ = psthread_queue_t_->ps_front_->ps_next_;
		psthread_queue_t_->ps_rear_->ps_next_ = psthread_queue_t_->ps_front_;

		//Free the temp
		free(ps_temp);
	}

	//Update size of queue
	psthread_queue_t_->ui_num_threads_--;

	return s_dequeued_val;
}

//-----------------------------------------------------------------------
UINT //The number of threads in the RTOS Queue
rtos_getQueueSize( //A function that returns the size of the thread_queue_t
		thread_queue_t* psthread_queue_t_) //A pointer to an thread_queue_t
{
	return (psthread_queue_t_->ui_num_threads_);
}

//-----------------------------------------------------------------------
thread_node_t* //A pointer to a thread_node_t
rtos_peekQueue( //A function that returns the first value of an thread_queue_t
		thread_queue_t* psthread_queue_t_) //A pointer to an thread_queue_t
{
	return (psthread_queue_t_->ps_front_);
}

//-----------------------------------------------------------------------
BOOLE //True if the queue is empty, otherwise false
rtos_isQueueEmpty( //A function that returns whether the thread_queue_t is empty
		thread_queue_t* psthread_queue_t_) //A pointer to an thread_queue_t
{
	return (psthread_queue_t_->ps_front_ == NULL);
}

//-----------------------------------------------------------------------
BOOLE //True if the queue is full, otherwise false
rtos_isQueueFull( //A function that returns whether the queue is full
		thread_queue_t* psthread_queue_t_) //A pointer to an thread_queue_t
{
	return (psthread_queue_t_->ui_num_threads_ == psthread_queue_t_->ui_max_num_threads_);
}



