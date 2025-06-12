/**
  *****************************************************************************
  * FILENAME   : init_calls.h
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.03.31 21:54:24 Friday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHF               2023.03.31                    CREATE
  *****************************************************************************
  */
/******************************************************************************/
#ifndef _INIT_CALLS_H_2023_03_31_21_54_24_793_
#define _INIT_CALLS_H_2023_03_31_21_54_24_793_
/************* Included files, Macros, Various and Declarations ***************/
#include <stdint.h>

typedef struct INIT_FUNC_T
{
    int (*fn)(void);
    const char* fn_name;
} INIT_FUNC;

typedef struct INIT_TASK_T
{
    void (*task_fn)(void *);
    const char* const   task_name;
    unsigned int        stack_depth;
    void*               param;
    unsigned int        task_prio;
    unsigned int        *handler;
} INIT_TASK;

#define SECTION_FN(level)  __attribute__((used, __section__(".init_fn_"level)))
#define INIT_START(func) static const INIT_FUNC init_fn_start SECTION_FN(".")   = {func, #func}
#define INIT_CALL_0(func) static const INIT_FUNC init_fn_##func SECTION_FN("0") = {func, #func}
#define INIT_CALL_1(func) static const INIT_FUNC init_fn_##func SECTION_FN("1") = {func, #func}
#define INIT_CALL_2(func) static const INIT_FUNC init_fn_##func SECTION_FN("2") = {func, #func}
#define INIT_CALL_3(func) static const INIT_FUNC init_fn_##func SECTION_FN("3") = {func, #func}
#define INIT_CALL_4(func) static const INIT_FUNC init_fn_##func SECTION_FN("4") = {func, #func}
#define INIT_CALL_5(func) static const INIT_FUNC init_fn_##func SECTION_FN("5") = {func, #func}
#define INIT_CALL_6(func) static const INIT_FUNC init_fn_##func SECTION_FN("6") = {func, #func}
#define INIT_CALL_7(func) static const INIT_FUNC init_fn_##func SECTION_FN("7") = {func, #func}
#define INIT_CALL_8(func) static const INIT_FUNC init_fn_##func SECTION_FN("8") = {func, #func}
#define INIT_CALL_9(func) static const INIT_FUNC init_fn_##func SECTION_FN("9") = {func, #func}
#define INIT_END(func) static const INIT_FUNC init_fn_end SECTION_FN("?")       = {func, #func}


#define SECTION_TASK(level)  __attribute__((used, __section__(".task.allcpu."level)))
#define INIT_TASK_START(func, task_name, stack_depth, param, task_prio, handler) \
    static const INIT_TASK init_task_start SECTION_TASK("0") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define START_TASK(func, task_name, stack_depth, param, task_prio, handler) \
    static const INIT_TASK init_task_##func SECTION_TASK("1") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define INIT_TASK_END(func, task_name, stack_depth, param, task_prio, handler) \
    static const INIT_TASK init_task_end SECTION_TASK("2") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}


#define PRIVATE_TASK_START(func, task_name, stack_depth, param, task_prio, handler, _private_) \
    static const INIT_TASK private_task_start##_private_ SECTION_TASK(".0"#_private_) = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define PRIVITE_TASK(func, task_name, stack_depth, param, task_prio, handler, _private_) \
    static const INIT_TASK private_task_##func SECTION_TASK(".1"#_private_) = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define PRIVATE_TASK_END(func, task_name, stack_depth, param, task_prio, handler, _private_) \
    static const INIT_TASK private_task_end##_private_ SECTION_TASK(".2"#_private_) = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}

#define SECTION_TASK_CPU0(level)  __attribute__((used, __section__(".task.cpu0."level)))
#define INIT_TASK_START_CPU0(func, task_name, stack_depth, param, task_prio, handler) \
         static const INIT_TASK init_task_start_cpu0 SECTION_TASK_CPU0("0") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define START_TASK_CPU0(func, task_name, stack_depth, param, task_prio, handler) \
         static const INIT_TASK init_task_##func SECTION_TASK_CPU0("1") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define INIT_TASK_END_CPU0(func, task_name, stack_depth, param, task_prio, handler) \
         static const INIT_TASK init_task_end_cpu0 SECTION_TASK_CPU0("2") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}

#define SECTION_TASK_CPU1(level)  __attribute__((used, __section__(".task.cpu1."level)))
#define INIT_TASK_START_CPU1(func, task_name, stack_depth, param, task_prio, handler) \
         static const INIT_TASK init_task_start_cpu1 SECTION_TASK_CPU1("0") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define START_TASK_CPU1(func, task_name, stack_depth, param, task_prio, handler) \
         static const INIT_TASK init_task_##func SECTION_TASK_CPU1("1") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}
#define INIT_TASK_END_CPU1(func, task_name, stack_depth, param, task_prio, handler) \
         static const INIT_TASK init_task_end_cpu1 SECTION_TASK_CPU1("2") = {func, task_name, stack_depth, param, task_prio, (unsigned int*)(handler)}


#define PRIVITE_TASK_IMPLEMENT(_private_)   \
     PRIVATE_TASK_START((void(*)(void*))0, "", 0, 0, 0, 0, _private_);  \
     PRIVATE_TASK_END((void(*)(void*))0, "", 0, 0, 0, 0, _private_);

void start_init_task(const char* task_name, void (*fn)(void*), uint32_t priority, uint32_t stack_size);
int start_task(const INIT_TASK *start, const INIT_TASK *end);
#define START_PRIVATE_TASK(_private_) \
{\
    start_task(&private_task_start##_private_ + 1, &private_task_end##_private_);\
}

/******************************** Functions **********************************/



#endif

