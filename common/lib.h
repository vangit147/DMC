#ifndef __COMMON_LIB_H__
#define __COMMON_LIB_H__

#include "stdint.h"
#include <math.h>
#include "string.h"
#include "main.h"

uint16_t Lib_ModbusCrc(uint8_t *data, uint16_t len);
void* fast_memcpy(void*, void *, uint32_t);
void swap(void *p, uint32_t len);
float naive_variance(float* sample, int32_t num);
float InvSqrt(float x);
float naive_std_variance(float* sample, int32_t num);
void cpy_swap(void *destin, void *source, uint8_t n);
void cpy_swap_inc_destion(uint8_t **destin, void *source, uint8_t n);
void cpy_swap_inc_source(void *destin, uint8_t **source, uint8_t n);
void get_max_min_acc_float(float *max, float *min, float *acc, float source);
void get_max_min_acc_int16(int16_t *max, int16_t *min, int16_t *acc, int16_t source);
#endif


