/*-------------------------------------------*/
/* Integer type definitions for FatFs module */
/*-------------------------------------------*/

#ifndef _FF_INTEGER
#define _FF_INTEGER

#ifdef _WIN32	/* FatFs development platform */

#include <windows.h>
#include <tchar.h>
typedef unsigned __int64 QWORD;


#else			/* Embedded platform */

#include "baseTypes.h"

/* These types MUST be 16-bit or 32-bit */
//typedef int				INT;
//typedef unsigned int	UINT;

/* This type MUST be 8-bit */
typedef UCHAR			BYTE;

/* These types MUST be 16-bit */
//typedef short			SHORT;
typedef USHORT	WORD;
typedef USHORT	WCHAR;

/* These types MUST be 32-bit */
typedef INT			FFLONG;
typedef UINT		DWORD;

/* This type MUST be 64-bit (Remove this for ANSI C (C89) compatibility) */
typedef ULONG 		QWORD;

#endif

#endif
