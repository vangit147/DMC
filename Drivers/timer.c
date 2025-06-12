/**
  *****************************************************************************
  * FILENAME   : timer.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.10.03 10:29:13 Tuesday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.10.03                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "timer.h"

#define     CMR_VAL     (2 - 1)
static void (*LPTMR_timeout_cb)(void);
static int32_t (*flextimer_mc1_isr_cb)(void);

static ftm_state_t  flextimer_mc1_state;
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.03 13:02:44 Tuesday
  *******************************************************************************
  */
static void LPTMR_IRQHandler(void)
{
//    static uint32_t counter;
    LPTMR_Type* const base = LPTMR_DRV_GetTmrBase(INST_LPTMR0);

    base->CMR += CMR_VAL;
    base->CSR |= LPTMR_CSR_TCF_MASK;

// YangHaifeng 2023.10.03 16:12:44 Tue
//  counter++;
//  if(counter >= 100)
//  {
//      counter = 0;
//      printf("lptmr\r\n");
//  }
    if(LPTMR_timeout_cb)
        LPTMR_timeout_cb();
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.10.03 10:32:19 Tuesday
  *******************************************************************************
  */
int32_t lptmr0_init(void (*cb)(void))
{
    LPTMR_timeout_cb = cb;

    LPTMR_DRV_Init(INST_LPTMR0, &lpTmr0_config0, 0);
    LPTMR_DRV_ClearCompareFlag(INST_LPTMR0);
    LPTMR_DRV_SetCompareValueByCount(INST_LPTMR0, CMR_VAL);

    INT_SYS_InstallHandler(LPTMR0_IRQn, LPTMR_IRQHandler, 0);
    INT_SYS_SetPriority(LPTMR0_IRQn, 0);
    INT_SYS_EnableIRQ(LPTMR0_IRQn);

    LPTMR_DRV_SetCompareValueByUs(INST_LPTMR0, 1);
    LPTMR_DRV_StartCounter(INST_LPTMR0);
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.01.28 00:11:49 Sunday
  *******************************************************************************
  */
static void flextimer_mc1_IRQHandler(void)
{
    FTM_DRV_ClearStatusFlags(INST_FLEXTIMER_MC1, FTM_TIME_OVER_FLOW_FLAG);
    if(flextimer_mc1_isr_cb)
        flextimer_mc1_isr_cb();
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.01.28 00:10:00 Sunday
  *******************************************************************************
  */
int32_t flextime_mc1_init(int32_t (*isr)(void))
{
    flextimer_mc1_isr_cb = isr;


    FTM_DRV_Init(INST_FLEXTIMER_MC1, &flexTimer_mc1_InitConfig, &flextimer_mc1_state);

    INT_SYS_InstallHandler(FTM0_Ovf_Reload_IRQn, flextimer_mc1_IRQHandler, 0);
    INT_SYS_SetPriority(FTM0_Ovf_Reload_IRQn, 0);
    INT_SYS_EnableIRQ(FTM0_Ovf_Reload_IRQn);

    FTM_DRV_InitCounter(INST_FLEXTIMER_MC1, &flexTimer_mc1_TimerConfig);
    FTM_DRV_CounterStart(INST_FLEXTIMER_MC1);
    return 0;
}

