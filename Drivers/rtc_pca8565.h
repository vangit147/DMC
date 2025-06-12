/**
  *****************************************************************************
  * FILENAME   : rtc_pca8565.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.10.03 17:05:19 Tuesday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.10.03                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _RTC_PCA8565_H_2023_10_03_17_5_19_828_
#define _RTC_PCA8565_H_2023_10_03_17_5_19_828_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"



/******************************** Functions **********************************/
int32_t pca8565_init(void);
void PCA8565_on_timer_event(void);
int32_t PCA8565_get_data(uint8_t* data, uint32_t len);
int32_t PCA8565_set_data(uint32_t year, uint32_t mon, uint32_t day,
                         uint32_t hour, uint32_t min, uint32_t sec);



#ifdef __cplusplus
}
#endif
#endif

