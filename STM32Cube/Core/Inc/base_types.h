/**
 ********************************************************************************
 * @file base_types.h
 * @date
 * @brief Common file for types for cross-processor/platform compatibility
 ********************************************************************************
 */

#ifndef __BASETYPES_H__
#define __BASETYPES_H__

//If C++, include the C++-specific header
#ifdef __cplusplus
    #include <cstdint>
// If C, include the C header for fixed-width types
#else
    #include <stdint.h>
#endif

/************************************
 * Type Definitions (use standard types)
 ************************************/

// Use standard types from C/C++ headers
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

#define TRUE  ((BOOLE)1)  //TRUE as unsigned char 1
#define FALSE ((BOOLE)0)  //FALSE as unsigned char 0

#endif /* __BASE_TYPES_H__ */
