/**
  *****************************************************************************
  * FILENAME   : S32K144_it.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.09 15:30:18 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.09                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _S32K144_IT_H_2023_09_09_15_30_18_477_
#define _S32K144_IT_H_2023_09_09_15_30_18_477_
#ifdef __cplusplus
extern "C" {
#endif
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"


/******************************** Functions **********************************/
uint32_t get_irq_vect_addr(void);
int32_t register_irq_handler(uint32_t irq_no, void(*irq_handler)(void));
int32_t unregister_irq_handler(uint32_t irq_no);



#ifdef __cplusplus
}
#endif
#endif

