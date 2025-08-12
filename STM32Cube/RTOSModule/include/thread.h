/**
 ********************************************************************************
 * @file thread.h
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __THREAD__
#define __THREAD__

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

/************************************
 * Thread Context Struct (if used)
 ************************************/
typedef struct ThreadContextStruct
{
    UINT* puiMyThreadStackPointer;  // Stack pointer for the thread
    void (*pfnMyThreadFunction)(void*); // Thread function pointer
    UINT uiMyThreadRuntimeMs; // Runtime in milliseconds
    UINT uiMyThreadTimesliceMs; // Time slice for the thread in milliseconds
} ThreadContextStruct;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/


#ifdef __cplusplus
}
#endif

#endif // __THREAD___
