/**
 ********************************************************************************
 * @file mutex.c
 * @author Daniel Luchian
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "lk_mutex.h"
#include "lk_sync.h"
#include "scheduler.h"
#include "utilities.h"
#include "lk_kernel.h"

/************************************
 * EXTERN VARIABLES
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
 * STATIC FUNCTION PROTOTYPES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
CHAR            // 0 on success, -1 otherwise
lk_mutexInit(   // Initialization function for a mutex
		mutex_t* mutex) // Pointer to mutex to initialize
{
	SIGNED_NULL_PTR_CHECK(mutex);

	lk_enterTaskCrit();
	mutex->lock_ = UNLOCKED;
	mutex->owner_ = NULL;
	mutex->init_ = TRUE;
	lk_exitTaskCrit();

	return 0;
}

//-----------------------------------------------------------------------
CHAR          // 0 on success, -1 otherwise
lk_mutexLock( // Function that yields thread until mutex is locked
		mutex_t* mutex) // Pointer to mutex
{
	SIGNED_NULL_PTR_CHECK(mutex);

	lk_enterTaskCrit();

	// 1) If mutex not initialize when first lock called, initialize it
	if (!mutex->init_)
	{
		lk_mutexInit(mutex);
	}

	// 2) Error if owner tries to re-lock it (not recursive mutex), avoiding deadlock
	if ( (mutex->lock_ == LOCKED) &&
		 (mutex->owner_ == g_thread_manager.current_thread_) )
	{
		lk_exitTaskCrit();
		return -1;
	}

	// 3) Block for mutex
	while (mutex->lock_ == LOCKED)
	{
		lk_exitTaskCrit();
		lk_threadYield();
		lk_enterTaskCrit();
	}

	// Acquire
	mutex->owner_ = g_thread_manager.current_thread_;
	mutex->lock_ = LOCKED;

	lk_exitTaskCrit();

	return 0;
}

//-----------------------------------------------------------------------
CHAR			  // 0 on success, -1 otherwise
lk_mutexTryLock(  // Function that tries to lock the mutex once
		mutex_t* mutex) // Pointer to mutex
{
	SIGNED_NULL_PTR_CHECK(mutex);

	lk_enterTaskCrit();

	// 1) If mutex not initialize when first try lock called, initialize it
	if (!mutex->init_)
	{
		lk_mutexInit(mutex);
	}

	// 2) Lock it if its unlocked, otherwise error
	if ( mutex->lock_ == UNLOCKED )
	{
		mutex->owner_ = g_thread_manager.current_thread_;
		mutex->lock_ = LOCKED;
		lk_exitTaskCrit();
		return 0;
	}

	lk_exitTaskCrit();
	return -1;
}

//-----------------------------------------------------------------------
CHAR			 // 0 on success, -1 otherwise
lk_mutexUnlock(  // Function to unlock a mutex
		mutex_t* mutex) // Pointer to mutex
{
	SIGNED_NULL_PTR_CHECK(mutex);

	lk_enterTaskCrit();

	// 1) Init mutex if not init
	if (!mutex->init_)
	{
		lk_exitTaskCrit();
		return -1;
	}

	// 2) Unlock mutex if possible
	if ( (mutex->owner_ == g_thread_manager.current_thread_) &&
		 (mutex->lock_ == LOCKED) )
	{
		mutex->owner_ = NULL;
		mutex->lock_ = UNLOCKED;
		lk_exitTaskCrit();
		return 0;
	}

	lk_exitTaskCrit();

	return -1;
}

//-----------------------------------------------------------------------
void
lk_mutexDestroy( // Function to destroy mutex, de-initiliazes it, struct remains usable
		mutex_t* mutex) // Pointer to mutex
{
	VOID_NULL_PTR_CHECK(mutex);

	lk_enterTaskCrit();
	mutex->lock_ = UNLOCKED;
	mutex->owner_ = NULL;
	mutex->init_ = FALSE;
	lk_exitTaskCrit();
}



