/**
 ********************************************************************************
 * @file
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __itc_types__
#define __itc_types__

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

#endif // __itc_types__
