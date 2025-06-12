/**
  *****************************************************************************
  * FILENAME   : i2c.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.24 22:55:44 Sunday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.24                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "i2c.h"

static lpi2c_master_state_t   i2c_master_state;
static void (*i2c_end_transfer_cb)(void);

void LPI2C0_Master_IRQHandler(void);

/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 20:38:20 Sunday
  *******************************************************************************
  */
static void i2c_master_callback(i2c_master_event_t event, void *userData)
{
    if(i2c_end_transfer_cb)
        i2c_end_transfer_cb();
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 14:57:13 Sunday
  *******************************************************************************
  */
int32_t i2c_init(void)
{
    INT_SYS_InstallHandler(LPI2C0_Master_IRQn, LPI2C0_Master_IRQHandler, 0);
    INT_SYS_SetPriority(LPI2C0_Master_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    lpi2c0_MasterConfig0.masterCallback = i2c_master_callback;
    LPI2C_DRV_MasterInit(INST_LPI2C0, &lpi2c0_MasterConfig0, &i2c_master_state);
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.29 00:17:11 Friday
  *******************************************************************************
  */
int32_t i2c_register_end_transfer_cb(void(*cb)(void))
{
    i2c_end_transfer_cb = cb;
    return 0;
}
