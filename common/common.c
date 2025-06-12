/**
  *****************************************************************************
  * FILENAME   : common.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2024.
  * CREATEDDATE: 2024.11.09 11:24:09 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * NickYang                  2024.11.09                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include <stdint.h>
#include <math.h>
#include "string.h"
#include "fast_math_functions.h"

#include "quick_sort.h"
#include "common.h"
/******************************** Functions **********************************/

/**
  *******************************************************************************
  * @Description:根据输入的样本数组计算标准差
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : Gordon
  * @CreatedDate: 2024.11.28 20:32:35 Wednesday
  *******************************************************************************
  */
float naive_std_variance(float* sample, int32_t num)
{
		//calculate the mean 
		float sum = 0;
    float square_root = 0;
    float mean;
		
	  if(num<=1) return 0;
    for(int i = 0; i < num; i++)
    {
        mean = sample[i];
        sum += mean;
    }
		mean = sum / num ;
		
		sum = 0.0;
    for (int i = 0; i < num; i++) {
				sum += (sample[i] - mean) * (sample[i] - mean);
    }
		
		float variance = sum / num;  // 总体方差
		arm_sqrt_f32(variance,&square_root);
		return square_root;
}



/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.10.30 22:38:35 Wednesday
  *******************************************************************************
  */
float naive_variance(float* sample, int32_t num)
{
    float sum = 0;
    float square = 0;
    float variance;
    float v;

    for(int i = 0; i < num; i++)
    {
        v = *sample++;
        sum += v;
        square += v * v;
    }

    v = sum / num;
    variance = square / num - v * v;
    return variance;
}
/**
  *******************************************************************************
  * @Description:Newton's method
  * @Parameters :
  * @RetValue   : 1/Sqrt(x)
  * @Note       : 牛顿迭代算法sqrt函数

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.26 23:54:35 Monday
  *******************************************************************************
  */
float InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters : out_ie--方差最小的ie
  * @RetValue   : 加权ie
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.10.31 23:12:49 Thursday
  *******************************************************************************
  */
float filter_ie_by_variance(float variace, float ie, float* out_ie)
{
#define FILTER_SIZE     20
    static float    ie_history[FILTER_SIZE];
    static float    standard_deviation[FILTER_SIZE];
    static float    sorted_sd[FILTER_SIZE];
    static float    weight_sd[FILTER_SIZE];

    uint8_t flag[FILTER_SIZE];
    float   weight_sum;
    float   ie_sum;


    fast_memcpy(ie_history, ie_history + 1, sizeof(ie_history) - sizeof(ie_history[0]));
    fast_memcpy(standard_deviation, standard_deviation + 1, sizeof(standard_deviation) - sizeof(standard_deviation[0]));

    //计算标准差
    ie_history[FILTER_SIZE - 1] = ie;
 //   standard_deviation[FILTER_SIZE - 1] = 1 / InvSqrt(variace);
	standard_deviation[FILTER_SIZE - 1] = variace;
//    if(standard_deviation[FILTER_SIZE - 1] < 0.1f)
//        standard_deviation[FILTER_SIZE - 1] = 0.1f;

    //寻找方差最小的ie
    if(out_ie)
    {
        float min_sd = standard_deviation[0];
        int   min_index = 0;
        for(int i = 1; i < FILTER_SIZE; i++)
        {
            if(min_sd > standard_deviation[i])
            {
                min_sd = standard_deviation[i];
                min_index = i;
            }
        }
        *out_ie = ie_history[min_index];
    }

    //计算权重
    memset(flag, 0x0, sizeof(flag));
    weight_sum = 0;
    for(int i = 0; i < FILTER_SIZE; i++)
        weight_sum += standard_deviation[i];
    fast_memcpy(sorted_sd, standard_deviation, sizeof(standard_deviation));
    quicksort_f32(sorted_sd, 0, FILTER_SIZE - 1);
    for(int i = 0; i < FILTER_SIZE; i++)
    {
        for(int j = 0; j < FILTER_SIZE; j++)
        {
            if(sorted_sd[i] == standard_deviation[j] && flag[j] == 0)
            {
                flag[j] = 1;
                weight_sd[j] = sorted_sd[FILTER_SIZE - 1 - i] / weight_sum;
                break;
            }
        }
    }
    //计算ie
    ie_sum = 0;
    for(int i = 0; i < FILTER_SIZE; i++)
        ie_sum += ie_history[i] * weight_sd[i];
    return ie_sum;
}




