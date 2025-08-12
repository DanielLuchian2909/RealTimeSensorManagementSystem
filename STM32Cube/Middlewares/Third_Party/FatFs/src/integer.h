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
//typedef int	 		INT;  /* Already defined in base_types.h */
//typedef unsigned int	UINT; /* Already defined in base_types.h */

/* This type MUST be 8-bit */
typedef UCHAR			BYTE;

/* These types MUST be 16-bit */
//typedef short			SHORT; /* Already defined in base_types.h */
typedef USHORT			WORD;
typedef USHORT			WCHAR;

/* These types MUST be 32-bit */
typedef INT  			FFLONG;
typedef UINT			DWORD;

/* This type MUST be 64-bit (Remove this for ANSI C (C89) compatibility) */
typedef ULONG		QWORD;

#endif

#endif

typedef uint8_t  UCHAR;    //Unsigned 8-bit
typedef int8_t   CHAR;     //Signed 8-bit
typedef uint16_t USHORT;   //Unsigned 16-bit
typedef int16_t  SHORT;    //Signed 16-bit
typedef uint32_t UINT;     //Unsigned 32-bit
typedef int32_t  INT;      //Signed 32-bit
typedef uint64_t ULONG;    //Unsigned 64-bit
typedef int64_t  LONG;     //Signed 64-bit
typedef uint8_t  BOOLE;    //Boolean type
typedef float    FLOAT;    //32-bit floating point
typedef double   DOUBLE;   //64-bit floating point
