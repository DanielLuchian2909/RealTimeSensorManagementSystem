/**
 ********************************************************************************
 * @file BaseTypes.h
 * @date
 * @brief Common file for types for cross-processor/platform compatibility
 ********************************************************************************
 */

#ifndef __BASETYPES_H__
#define __BASETYPES_H__

#define TRUE 1
#define FALSE 0

typedef float FLOAT;
typedef double DOUBLE;
typedef unsigned short USHORT;
typedef short SHORT;
typedef unsigned char UCHAR;
typedef char CHAR;
typedef unsigned short BOOLE;
typedef unsigned int UINT;
typedef int INT;

//Thread context struct
typedef struct
{
	UINT* puiMyThreadStackPointer; //Stack pointer for the thread
	void (*pfnMyThreadFunction)(void*); //Thread function pointer
} ThreadContextStruct;

#endif /* __BASETYPES_H__ */
