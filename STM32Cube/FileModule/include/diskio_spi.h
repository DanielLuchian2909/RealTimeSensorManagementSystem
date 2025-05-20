/**
 ********************************************************************************
 * @file
 * @author
 * @brief
 ********************************************************************************
 */

#ifndef __DISKIO_SPI__
#define __DISKIO_SPI__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "ff_gen_drv.h"
#include "diskio.h"

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
DSTATUS sdcard_UserInitialize (BYTE pdrv);
DSTATUS sdcard_UserStatus (BYTE pdrv);
DRESULT sdcard_UserRead (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT sdcard_UserWrite (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT sdcard_UserIoctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

#ifdef __cplusplus
}
#endif

#endif // __MY_FILE__
