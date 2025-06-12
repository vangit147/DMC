/**
  *****************************************************************************
  * FILENAME   : common.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2024.
  * CREATEDDATE: 2024.11.09 11:24:30 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * NickYang                  2024.11.09                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _COMMON_H_2024_11_09_11_24_30_540_
#define _COMMON_H_2024_11_09_11_24_30_540_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/



/******************************** Functions **********************************/
float naive_std_variance(float* sample, int32_t num);
float naive_variance(float* sample, int32_t num);
float InvSqrt(float x);
float filter_ie_by_variance(float variace, float ie, float* out_ie);
void* fast_memcpy(void*, void *, uint32_t);



#ifdef __cplusplus
}
#endif
#endif

