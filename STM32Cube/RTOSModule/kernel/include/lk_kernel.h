/**
 ********************************************************************************
 * @file kernel.h
 * @author Daniel Luchian
 * @brief Kernel header file for RTOS functionalities and configuration
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
#include "base_types.h" //Basic type definitions and utilities

/************************************
 * MACROS AND DEFINES
 ************************************/
/* Standard stack sizes */
#define STACK_SIZE 0x4000 //Default stack size for the system stack

/* SVC Numbers */
#define RUN_FIRST_THREAD 0x00
#define RTOS_YIELD 0x01

/* PendSVC priority */

#define SVC_PRIO     1
#define PENDSV_PRIO  2
#define SYSTICK_PRIO 3

/* Kernel Status Flags */
#define KERNEL_INITIALIZED		0x0001
#define KERNEL_NOT_INITIALIZED  0x0002
#define KERNEL_STARTED			0x0004
#define KERNEL_RUNNING			0x0008
#define KERNEL_SCHEDULING		0x0010

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
// Kernel Control Functions
BOOLE lk_kernelInit(void); //Initializes all kernel related functions/data
void lk_kernelStart(void); //A function that starts the RTOS

// Thread creation and manipulation functions
BOOLE lk_createThread(void (*thread_fn)(void*), void* parameters); //Creates a thread
BOOLE lk_createThreadWithDeadline(void (*thread_fn)(void*), void* parameters, UINT deadline); //Creates a thread
void lk_threadYield(void); //Yield function for the RTOS
void lk_delay(UINT delay_ms);

#ifdef __cplusplus
}
#endif

#endif
