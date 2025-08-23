/**
 ********************************************************************************
 * @file scheduler.h
 * @author Daniel Luchian
 * @brief Header file for the scheduler
 ********************************************************************************
 */

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

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

/************************************
 * TYPEDEFS
 ************************************/
#define ROUND_ROBIN_TIMEOUT_MS 1000; //The default value for a threads timeslice in ms

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

#ifdef __cplusplus
}
#endif

#endif
