/**
  *****************************************************************************
  * FILENAME   : timer.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.10.03 10:29:46 Tuesday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.10.03                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _TIMER_H_2023_10_03_10_29_46_526_
#define _TIMER_H_2023_10_03_10_29_46_526_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"



/******************************** Functions **********************************/
int32_t lptmr0_init(void (*cb)(void));
int32_t flextime_mc1_init(int32_t (*isr)(void));



#ifdef __cplusplus
}
#endif
#endif

