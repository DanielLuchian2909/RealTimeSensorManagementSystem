/**
 ********************************************************************************
 * @file mutex.h
 * @author Daniel Luchian
 * @brief
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

/************************************
 * TYPEDEFS
 ************************************/
typedef INT mutex_t; /* Pending full implemenation once critical section for the ARM Cortex M-4 are implemented */

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
INT rtos_mutexInit(mutex_t* mutex);
INT rtos_mutexLock(mutex_t* mutex);
INT rtos_mutexUnlock(mutex_t* mutex);
INT rtos_mutexTryLock(mutex_t* mutex);
void rtos_mutexDestroy(mutex_t* mutex);

#ifdef __cplusplus
}
#endif

#endif // __MUTEX_H__
