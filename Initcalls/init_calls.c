/**
  *****************************************************************************
  * FILENAME   : init_calls.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.03.31 21:53:04 Friday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHF               2023.03.31                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include <init_calls.h>
#include <stdio.h>
#include <freertos.h>
#include <task.h>
#include "main.h"


INIT_START((int(*)(void))0);
INIT_END((int(*)(void))0);
INIT_TASK_START((void(*)(void*))0, "", 0, 0, 0, 0);
INIT_TASK_END((void(*)(void*))0, "", 0, 0, 0, 0);
INIT_TASK_START_CPU0((void(*)(void*))0, "", 0, 0, 0, 0);
INIT_TASK_END_CPU0((void(*)(void*))0, "", 0, 0, 0, 0);
INIT_TASK_START_CPU1((void(*)(void*))0, "", 0, 0, 0, 0);
INIT_TASK_END_CPU1((void(*)(void*))0, "", 0, 0, 0, 0);
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.04.19 22:13:05 Wednesday
  *******************************************************************************
  */
static int do_start_task(uint32_t cpuid, const INIT_TASK *begin, const INIT_TASK *end)
{
    BaseType_t      ret;
//#ifdef MULTI_CPU
//    const char* const cpu_str[] = {"[CPU0] ", "[CPU1] "};
//#else
//    const char* const cpu_str[] = {""};
//#endif
    for(; begin < end; begin++)
    {
        //SEGGER_RTT_printf(0, "%sCreating %s ...\r\n", cpu_str[cpuid], begin->task_name);
        /* printf("%sCreating %s ...\r\n", cpu_str[cpuid], begin->task_name); */
        ret = xTaskCreate(begin->task_fn, begin->task_name, begin->stack_depth, begin->param,
                          begin->task_prio, (TaskHandle_t*)(begin->handler));
//        if(ret != pdPASS)
//        {
//            //SEGGER_RTT_printf(0, "Creating %s task failed! Cause code: %ld\r\n", begin->task_name, ret);
//            /* printf("Creating %s task failed! Cause code: %ld\r\n", begin->task_name, ret); */
//        }
    }
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.03.31 21:55:37 Friday
  *******************************************************************************
  */
static int booting_task()
{
#ifdef MULTI_CPU
    do_start_task(get_cpu_id(), &init_task_start + 1, &init_task_end);
    if(get_cpu_id() == CPU_ID_MAIN)
        do_start_task(CPU_ID_MAIN, &init_task_start_cpu0 + 1, &init_task_end_cpu0);
    else
        do_start_task(CPU_ID_SLAVE, &init_task_start_cpu1 + 1, &init_task_end_cpu1);
#else
    do_start_task(0, &init_task_start + 1, &init_task_end);
#endif
    return 0;
}
INIT_CALL_9(booting_task);

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.03.31 21:58:35 Friday
  *******************************************************************************
  */
static void init_task(void* p)
{
    const INIT_FUNC *init_fn = &init_fn_start + 1;

    for(; init_fn < &init_fn_end; init_fn++)
    {
        //SEGGER_RTT_printf(0, "Starting %s ...\r\n", init_fn->fn_name);
        /* printf("Starting %s ...\r\n", init_fn->fn_name); */
        (*init_fn->fn)();
    }
    vTaskPrioritySet(xTaskGetCurrentTaskHandle(), ((uint32_t*)p)[1]);
    ((void(*)(void))(((uint32_t*)p)[0]))();
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.03.31 21:58:56 Friday
  *******************************************************************************
  */
void start_init_task(const char* task_name, void (*fn)(void*), uint32_t priority, uint32_t stack_size)
{
    uint32_t* p = (uint32_t *)pvPortMalloc(8);

    p[0] = (uint32_t)fn;
    p[1] = priority;
    xTaskCreate(init_task, task_name, stack_size, p, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
}

