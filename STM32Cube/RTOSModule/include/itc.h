/**
 ********************************************************************************
 * @file
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __itc__
#define __itc__

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
msg_t* new_msg();
void delete_msg(msg_t* msg);
int send(uint8_t dest_id, msg_t* msg);
int recv(uint8_t recv_id, msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif // __itc___
