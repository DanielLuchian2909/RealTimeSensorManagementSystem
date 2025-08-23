/**
 ********************************************************************************
 * @file arm_m4.h
 * @author Daniel Luchian
 * @brief
 ********************************************************************************
 */

#ifndef __RTOS_ARM_M4_H__
#define __RTOS_ARM_M4_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "stm32f4xx.h"

/************************************
 * MACROS AND DEFINES
 ************************************/
#define SHPR2 *(uint32_t*)0xE000ED1C //Setting SVC priority, bits 31-24
#define SHPR3 *(uint32_t*)0xE000ED20 //PendSV is bits 23-16
#define _ICSR *(uint32_t*)0xE000ED04 //Trigger PendSV

#define ALL_INTERRUPT_PRIO    0U    // Highest possible interrupt priority level (lowest value)
#define PENDSV_INTERRUPT_PRIO 0xFFU // Lowest possible interrupt priority level (highest value)
#define rtos_disableTaskInterrupts()  raiseBASEPRI()
#define rtos_enableInterrupts()       setBASEPRI()

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/
//-----------------------------------------------------------------------
__STATIC_FORCEINLINE void // Static inline function for speed and to give a copy to each translation since it short and quick
raiseBASEPRI() /* Raise the minimum interrupt priority register (lower priority register value means higher priority) to only interrupts
				* of priority below and the same as the level of context switching (PendSV). Basically means any interrupt
				* more important than PendSV can fire, any interrupt as important or less important that PendSV can't fire,
				* which stops context switches so makes the section thread safe.
				*/
{
	__set_BASEPRI(PENDSV_INTERRUPT_PRIO);
	__ISB; // Flushes processor pipeline so change in interrupt priority becomes immediately visible
	__DSB; // Make sure all instructions before this one complete before moving on
}

//-----------------------------------------------------------------------
__STATIC_FORCEINLINE void // Static inline function for speed and to give a copy to each translation since it short and quick
setBASEPRI()		      // Returns interrupt priority level lowest meaning all interrupts are allowed
{
	__set_BASEPRI(ALL_INTERRUPT_PRIO);
	/* Note for Self: Why not have ISB and DSB protection here? Because it doesn't really matter if this
	 * gets delayed by other instructions pipelined, you can enable interrupts now or later were still
	 * in the critical section, the big this is when entering the critical section that needs to happen right away.
	 */
}

/*********************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

#ifdef __cplusplus
}
#endif

#endif // __RTOS_ARM_M4_H__
