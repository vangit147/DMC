/**
  *****************************************************************************
  * FILENAME   : quick_sort.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2024.
  * CREATEDDATE: 2024.08.31 12:13:11 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * NY                        2024.08.31                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "quick_sort.h"


/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:38:54 Saturday
  *******************************************************************************
  */
static void swap_u8(uint8_t* a, uint8_t* b)
{
    uint8_t t = *a;
    *a = *b;
    *b = t;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:39:37 Saturday
  *******************************************************************************
  */
static int getpoint_u8(uint8_t x[], int low, int high)
{
    uint8_t point = x[low];//一般第一个基准点等于第一个元素
    while(low < high)   //左右不相遇代表排序还未结束
    {
        while(low < high && x[high] >= point)  //找到比基准点小的，排左边
        {
            high--;
        }
        swap_u8(&x[low], &x[high]);
        while(low < high && x[low] <= point)  //找到比基准点大的，排右边
        {
            low++;
        }
        swap_u8(&x[low], &x[high]);
    }
    return low;//左右相遇时的位置是新基准点的位置
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       : https://zhuanlan.zhihu.com/p/675294257

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:39:58 Saturday
  *******************************************************************************
  */
void quicksort_u8(uint8_t x[], int low, int high)
{
    if(low < high)
    {
        int point = getpoint_u8(x, low, high);
        quicksort_u8(x, low, point - 1); //新基准点将数组分成了左右两段
        quicksort_u8(x, point + 1, high); //左右两段再各自进行快速排序
    }
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:38:54 Saturday
  *******************************************************************************
  */
static void swap_f32(float* a, float* b)
{
    float t = *a;
    *a = *b;
    *b = t;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:39:37 Saturday
  *******************************************************************************
  */
static int getpoint_f32(float x[], int low, int high)
{
    float point = x[low];//一般第一个基准点等于第一个元素
    while(low < high)   //左右不相遇代表排序还未结束
    {
        while(low < high && x[high] >= point)  //找到比基准点小的，排左边
        {
            high--;
        }
        swap_f32(&x[low], &x[high]);
        while(low < high && x[low] <= point)  //找到比基准点大的，排右边
        {
            low++;
        }
        swap_f32(&x[low], &x[high]);
    }
    return low;//左右相遇时的位置是新基准点的位置
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       : https://zhuanlan.zhihu.com/p/675294257

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:39:58 Saturday
  *******************************************************************************
  */
void quicksort_f32(float x[], int low, int high)
{
    if(low < high)
    {
        int point = getpoint_f32(x, low, high);
        quicksort_f32(x, low, point - 1); //新基准点将数组分成了左右两段
        quicksort_f32(x, point + 1, high); //左右两段再各自进行快速排序
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:38:54 Saturday
  *******************************************************************************
  */
static void swap_int32(int32_t* a, int32_t* b)
{
    int32_t t = *a;
    *a = *b;
    *b = t;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:39:37 Saturday
  *******************************************************************************
  */
static int getpoint_int32(int32_t x[], int low, int high)
{
    int32_t point = x[low];//一般第一个基准点等于第一个元素
    while(low < high)   //左右不相遇代表排序还未结束
    {
        while(low < high && x[high] >= point)  //找到比基准点小的，排左边
        {
            high--;
        }
        swap_int32(&x[low], &x[high]);
        while(low < high && x[low] <= point)  //找到比基准点大的，排右边
        {
            low++;
        }
        swap_int32(&x[low], &x[high]);
    }
    return low;//左右相遇时的位置是新基准点的位置
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       : https://zhuanlan.zhihu.com/p/675294257

  * @CreatedBy  : NY
  * @CreatedDate: 2024.06.01 22:39:58 Saturday
  *******************************************************************************
  */
void quicksort_int32(int32_t x[], int low, int high)
{
    if(low < high)
    {
        int point = getpoint_int32(x, low, high);
        quicksort_int32(x, low, point - 1); //新基准点将数组分成了左右两段
        quicksort_int32(x, point + 1, high); //左右两段再各自进行快速排序
    }
}




