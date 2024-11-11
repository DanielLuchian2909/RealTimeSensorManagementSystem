/**
 ********************************************************************************
 * @file kernel.h
 * @date
 * @brief
 ********************************************************************************
 */

#ifndef __KERNEL_H__
#define __KERNEL_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "baseTypes.h"

/************************************
 * MACROS AND DEFINES
 ************************************/

//Std. Stack Sizes
#define STACK_SIZE 0x4000
#define THREAD_STACK_SIZE 0x400

//SVC Numbers
#define RUN_FIRST_THREAD 0x00

/************************************
 * TYPEDEFS
 ************************************/

// Thread context struct
typedef struct k_thread
{
	UINT* puiMyThreadStackPointer; //stack pointer for the thread
	void (*pfnMyThreadFunction)(void*); //thread function pointer
} ThreadContextStruct;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void os_KernelInit(void); //A function that initializes all kernel related functions/data
void os_KernelStart(void); //A function that starts the OS
BOOLE os_CreateThread(void (*pfnThreadFunction)(void*)); //A function that creates a thread


#ifdef __cplusplus
}
#endif

#endif // __MY_FILE__
