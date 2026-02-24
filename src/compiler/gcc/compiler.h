/**
  * @file      compiler.h
  *
  * @brief     Related to compiler macro and types
  *
   * @par
  * Copyright (C), 2025, Korban Pavel.
  * All Rights Reserved.
  *
  */

/** \addtogroup module
*  @{
*/

#ifndef COMPILER_H
#define COMPILER_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * INCLUDES
 ******************************************************************************/

#include "stdbool.h"


/******************************************************************************
 * DEFINES
 ******************************************************************************/

#define __IO                  volatile

#define NULL                  ((void *)0)

#define BOOL_BIT_QTY          (8U)

#define S8_BIT_QTY            (8U)
#define U8_BIT_QTY            (8U)

#define S16_BIT_QTY           (16U)
#define U16_BIT_QTY           (16U)

#define S32_BIT_QTY           (32U)
#define U32_BIT_QTY           (32U)

#define S64_BIT_QTY           (64U)
#define U64_BIT_QTY           (64U)

#define F32_BIT_QTY           (32U)
#define F64_BIT_QTY           (64U)



/******************************************************************************
 * PUBLIC TYPES
 ******************************************************************************/

typedef bool                 BOOL;

typedef signed char          S8;
typedef unsigned char        U8;

typedef signed short         S16;
typedef unsigned short       U16;

typedef signed long          S32;
typedef unsigned long        U32;

typedef signed long long     S64;
typedef unsigned long long   U64;

typedef float                F32;
typedef double               F64;



/******************************************************************************
 * INLINE FUNCTIONS
 ******************************************************************************/

// None.




/******************************************************************************
 * END OF HEADER'S CODE
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // COMPILER_H

/** @}*/
