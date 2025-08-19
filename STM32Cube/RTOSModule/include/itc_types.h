/**
 ********************************************************************************
 * @file
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __ITC_TYPES_H__
#define __ITC_TYPES_H__

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
#define MSG_DATA_LEN_BYTES 255

/************************************
 * TYPEDEFS
 ************************************/
/* Message Struct */
typedef struct msg_t
{
    UCHAR len_;
    UCHAR data_[MSG_DATA_LEN_BYTES];
} msg_t;


#ifdef __cplusplus
}
#endif

#endif // __ITC_TYPES_H__
