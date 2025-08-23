/**
 ********************************************************************************
 * @file
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __ITC_H__
#define __ITC_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "base_types.h"
#include "itc_types.h"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
msg_t* lk_newMsg();
void lk_deleteMsg(msg_t* msg);
INT lk_sendMsg(UCHAR dest_id, msg_t* msg);
INT lk_recvMsg(UCHAR recv_id, msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif // __ITC_H__
