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
#define THREAD_STACK_SIZE 0x400 //Stack size for individual threads

/* SVC Numbers */
#define RUN_FIRST_THREAD 0x00
#define RTOS_YIELD 0x01

/* PendSVC priority */
#define SHPR2 *(uint32_t*)0xE000ED1C //Setting SVC priority, bits 31-24
#define SHPR3 *(uint32_t*)0xE000ED20 //PendSV is bits 23-16
#define _ICSR *(uint32_t*)0xE000ED04 //Trigger PendSV

/* Kernel Status Flags */
#define KERNEL_INITIALIZED		0x0001
#define KERNEL_NOT_INITIALIZED  0x0002
#define KERNEL_STARTED			0x0003

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
BOOLE rtos_kernelInit(void); //Initializes all kernel related functions/data
void rtos_kernelStart(void); //A function that starts the RTOS
BOOLE rtos_createThread(void (*pfnThreadFunction_)(void*), void* pvParameters_); //Creates a thread
BOOLE rtos_createThreadWithDeadline(void (*pfnThreadFunction_)(void*), void* pvParameters_, UINT uiDeadline_); //Creates a thread

#ifdef __cplusplus
}
#endif

#endif
