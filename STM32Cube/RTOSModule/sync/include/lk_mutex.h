/**
 ********************************************************************************
 * @file mutex.h
 * @author Daniel Luchian
 * @brief Mutex header file that defines an API for a mutex on this RTOS.
 * NOTE 1: This mutex is not ISR safe. The ISR can be blocked with an indefinite
 * unlock or ownership can be lost if the ISR unlocks it.
 * NOTE 2: This mutex is not DMA safe. Data inside crit sections created by this
 * mutex are not safe from DMA controllers accessing that data. Do not use this
 * mutex to protect data from DMA access.
 ********************************************************************************
 */

#ifndef __MUTEX_H__
#define __MUTEX_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "base_types.h"

/************************************
 * MACROS AND DEFINES
 ************************************/
//#define

/************************************
 * TYPEDEFS
 ************************************/
/* Mutex lock status enumerator */
typedef enum
{
	UNLOCKED = 0,
	LOCKED,
} mutex_state_e;

/* Mutex Structure */
typedef struct mutex_t
{
	volatile mutex_state_e lock_; // Core lock

} mutex_t;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
CHAR lk_mutexInit(mutex_t* mutex);
CHAR lk_mutexLock(mutex_t* mutex);
CHAR lk_mutexTryLock(mutex_t* mutex);
CHAR lk_mutexUnlock(mutex_t* mutex);
void lk_mutexDestroy(mutex_t* mutex);

#ifdef __cplusplus
}
#endif

#endif // __MUTEX_H__
