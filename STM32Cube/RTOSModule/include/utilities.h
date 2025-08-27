/**
 ********************************************************************************
 * @file utilities.h
 * @author Daniel Luchian
 * @brief
 ********************************************************************************
 */

#ifndef __UTILITIES_H__
#define __UTILITIES_H__

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
#define SIGNED_NULL_PTR_CHECK(ptr) do { if (ptr == NULL) { return -1; } } while (0)
#define VOID_NULL_PTR_CHECK(ptr) do { if (ptr == NULL) { return; } } while (0)

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

#endif // __UTILITIES_H__
