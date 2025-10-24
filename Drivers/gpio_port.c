/**
  *****************************************************************************
  * FILENAME   : gpio_port.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.26 22:40:24 Tuesday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.26                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#include "gpio_port.h"


void (*porta_callback0)(uint32_t);
void (*porta_callback1)(uint32_t);

void (*portc_callback0)(uint32_t);
void (*portc_callback1)(uint32_t);

void (*porte_callback0)();
void (*porte_callback1)();

static uint32_t porte_callback0_flag;
static uint32_t porte_callback1_flag;
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.28 23:13:39 Thursday
  *******************************************************************************
  */
static void PORTA_IRQHandler(void)
{
    uint32_t int_flag = PINS_DRV_GetPortIntFlag(PORTA);
    PINS_DRV_ClearPortIntFlagCmd(PORTA);
    if(porta_callback0)
        porta_callback0(int_flag);
    if(porta_callback1)
        porta_callback1(int_flag);
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.26 22:43:45 Tuesday
  *******************************************************************************
  */
static void PORTC_IRQHandler(void)
{
    uint32_t int_flag = PINS_DRV_GetPortIntFlag(PORTC);
    PINS_DRV_ClearPortIntFlagCmd(PORTC);
    if(portc_callback0)
        portc_callback0(int_flag);
    if(portc_callback1)
        portc_callback1(int_flag);
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.20 20:54:08 Tuesday
  *******************************************************************************
  */
static void PORTE_IRQHandler(void)
{
    uint32_t int_flag = PINS_DRV_GetPortIntFlag(PORTE);
    PINS_DRV_ClearPortIntFlagCmd(PORTE);
    if(porte_callback0 && (porte_callback0_flag & int_flag))
        porte_callback0();
    if(porte_callback1 && (porte_callback1_flag & int_flag))
        porte_callback1();
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.26 22:41:29 Tuesday
  *******************************************************************************
  */
int32_t gpio_port_init(void)
{
#if 0
    port_digital_filter_config_t cfg;

    cfg.clock = PORT_DIGITAL_FILTER_LPO_CLOCK;
    cfg.width = 1;
    PINS_DRV_ConfigDigitalFilter(PORTC, &cfg);
#endif

    INT_SYS_InstallHandler(PORTC_IRQn, PORTC_IRQHandler, 0);
    INT_SYS_SetPriority(PORTC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    INT_SYS_EnableIRQ(PORTC_IRQn);

    INT_SYS_InstallHandler(PORTA_IRQn, PORTA_IRQHandler, 0);
    INT_SYS_SetPriority(PORTA_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    INT_SYS_EnableIRQ(PORTA_IRQn);

    INT_SYS_InstallHandler(PORTE_IRQn, PORTE_IRQHandler, 0);
    INT_SYS_SetPriority(PORTE_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    INT_SYS_EnableIRQ(PORTE_IRQn);
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.26 22:47:16 Tuesday
  *******************************************************************************
  */
int32_t gpio_porta_register_cb(void (*cb)(uint32_t))
{
    if(porta_callback0 == 0)
        porta_callback0 = cb;
    else if(porta_callback1 == 0)
        porta_callback1 = cb;
    else
        return -1;
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.28 23:16:26 Thursday
  *******************************************************************************
  */
int32_t gpio_portc_register_cb(void (*cb)(uint32_t))
{
    if(portc_callback0 == 0)
        portc_callback0 = cb;
    else if(portc_callback1 == 0)
        portc_callback1 = cb;
    else
        return -1;
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.02.20 20:55:36 Tuesday
  *******************************************************************************
  */
int32_t gpio_porte_register_cb(uint32_t flag, void (*cb)(void))
{
    if(porte_callback0 == 0) {
        porte_callback0 = cb;
        porte_callback0_flag = flag;
		}
    else if(porte_callback1 == 0) {
        porte_callback1 = cb;
        porte_callback1_flag = flag;
		}
    else
        return -1;
    return 0;
}

