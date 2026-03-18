/**
  *****************************************************************************
  * FILENAME   : lpuart.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.23 23:54:07 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.23                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "main.h"
#include "cpu.h"

#define TX_SENDING_BUFFER_LEN   256
#define RS485_TX_LEVEL      1
static uint8_t              lpuart0_tx_fifo[512];                     /*临时暂存区*/
static uint8_t             *lpuart0_tx_fifo_wr_pos = lpuart0_tx_fifo;   /*写指针*/
static uint32_t             lpuart0_tx_fifo_data_len;                 /*数据长度*/
static uint8_t              lpuart0_tx_sending_buffer[TX_SENDING_BUFFER_LEN];            /*正在发送的缓冲区*/
static uint32_t             lpuart0_tx_sending_data_len;              /*正在发送的数据长度*/
static uint8_t              lpuart0_rx_buffer;
static void (*lpuart0_tx_cb)(uint32_t uartHandle);
static void (*lpuart0_rx_cb)(uint32_t uartHandle, uint32_t len);

static uint8_t              lpuart2_init_flag;                          /**/
static uint8_t              lpuart2_tx_fifo[256];                     /*临时暂存区*/
static uint8_t             *lpuart2_tx_fifo_wr_pos = lpuart2_tx_fifo;   /*写指针*/
static uint32_t             lpuart2_tx_fifo_data_len;                 /*数据长度*/
static uint8_t              lpuart2_tx_sending_buffer[TX_SENDING_BUFFER_LEN];            /*正在发送的缓冲区*/
static uint32_t             lpuart2_tx_sending_data_len;              /*正在发送的数据长度*/
static uint8_t              lpuart2_rx_buffer;
static void (*lpuart2_tx_cb)(uint32_t uartHandle);
static void (*lpuart2_rx_cb)(uint32_t uartHandle, uint32_t len);

/******************************** Functions **********************************/

/**
  *******************************************************************************
  * @Description: 通用UART发送中断处理函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.08.31 22:37:49 Thursday
  *******************************************************************************
  */
static int32_t uart_general_TxCpltCallback(uint32_t uartHandle, void (*tx_cb)(uint32_t),
        uint8_t* tx_fifo, uint32_t tx_fifo_size, uint32_t* tx_fifo_data_len, uint8_t* tx_fifo_wr,
        uint8_t* tx_sending_buffer, uint32_t tx_sending_buffser_size, uint32_t* tx_sendig_data_len)
{
    uint32_t    sended_count = *tx_sendig_data_len;    //已发送的字节数
    uint8_t     *tx_write_pos, *tx_fifo_rd_pos;
    int         i;
    uint32_t    tx_remained_len;

    /*把未发送的字符移到发送缓冲区前部*/
    tx_remained_len = 0;
    *tx_sendig_data_len = tx_remained_len;   //剩余数据长度
    if(tx_remained_len)
    {
        tx_write_pos = tx_sending_buffer;
        tx_fifo_rd_pos = tx_sending_buffer + sended_count;
        for(i = 0; i < tx_remained_len; i++)
            *tx_write_pos++ = *tx_fifo_rd_pos++;
    }

    /*启动传输*/
    if(tx_remained_len || *tx_fifo_data_len)
    {
        int         i;
        uint32_t    tx_fifo_data_len_shadow;

        tx_fifo_data_len_shadow = *tx_fifo_data_len;
        /*把暂存区的数据复制到发送缓冲区 [YangHF 2023.08.05 23:29:15 Sat] */
        tx_fifo_rd_pos = tx_fifo_wr - tx_fifo_data_len_shadow;
        if(tx_fifo_rd_pos < tx_fifo)
            tx_fifo_rd_pos += tx_fifo_size;

        tx_write_pos = tx_sending_buffer + tx_remained_len;
        for(i = tx_remained_len; i < tx_sending_buffser_size && tx_fifo_data_len_shadow;
                i++, tx_fifo_data_len_shadow--)
        {
            *tx_write_pos++ = *tx_fifo_rd_pos++;
            if(tx_fifo_rd_pos >= tx_fifo + tx_fifo_size)
                tx_fifo_rd_pos = tx_fifo;
        }
        *tx_fifo_data_len = tx_fifo_data_len_shadow;
        *tx_sendig_data_len = i;
        LPUART_DRV_SendData(uartHandle, tx_sending_buffer, i);

        return 1;
    }
    /*数据传输完毕 [YangHF 2023.08.06 00:01:53 Sun] */
    else if(tx_cb)
        tx_cb(uartHandle);

    return 0;
}

/**
  *******************************************************************************
  * @Description:   通用串口发送函数
  * @Parameters :   uartHandle              ---- 串口句柄
                    data                    ---- 欲发送的数据
                    len                     ---- 数据长度
                    tx_fifo                 ---- 发送暂存缓冲区
                    tx_fifo_size            ---- 暂存缓冲区大小
                    tx_fifo_data_len        ---- 暂存缓冲区有效数据长度
                    tx_fifo_wr              ---- 缓存缓冲区写指针
                    tx_sending_buffer       ---- 发送缓冲区
                    tx_sending_buffser_size ---- 发送缓冲区大小
                    tx_sendig_data_len      ---- 发送缓冲区数据长度
                    dma                     ---- 是否使用DMA进行发送
  * @RetValue   :   写入暂存区的数据长度
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.08.31 22:59:36 Thursday
  *******************************************************************************
  */
static int32_t uart_general_send(uint32_t uartHandle, const uint8_t* data, uint32_t len,
                                 uint8_t* tx_fifo, uint32_t tx_fifo_size, uint32_t* tx_fifo_data_len, uint8_t** tx_fifo_wr,
                                 uint8_t* tx_sending_buffer, uint32_t tx_sending_buffser_size, uint32_t* tx_sendig_data_len)
{
    uint32_t    primask;
    int32_t     remained_len = len;
    uint32_t    loc_usart1_tx_buffer_data_len;
    uint8_t     *usart1_write_pos, *usart1_rd_pos;
    int         i;

    primask = __get_PRIMASK();
    __disable_irq();

    /*把数据保存在发送缓冲区 [YangHF 2023.08.05 23:23:58 Sat] */
    loc_usart1_tx_buffer_data_len = *tx_fifo_data_len;
    usart1_write_pos = *tx_fifo_wr;
    while(loc_usart1_tx_buffer_data_len < tx_fifo_size && remained_len)
    {
        *usart1_write_pos++ = *data++;
        if(usart1_write_pos >= tx_fifo + tx_fifo_size)
            usart1_write_pos = tx_fifo;
        loc_usart1_tx_buffer_data_len++;
        remained_len--;
    }
    *tx_fifo_wr = usart1_write_pos;
    *tx_fifo_data_len = loc_usart1_tx_buffer_data_len;

    /*启动传输*/
    if(*tx_sendig_data_len == 0)
    {
        /*把暂存区的数据复制到发送缓冲区 [YangHF 2023.08.05 23:29:15 Sat] */
        usart1_rd_pos = usart1_write_pos - loc_usart1_tx_buffer_data_len;
        if(usart1_rd_pos < tx_fifo)
            usart1_rd_pos += tx_fifo_size;
        usart1_write_pos = tx_sending_buffer + *tx_sendig_data_len;
        for(i = *tx_sendig_data_len; i < tx_sending_buffser_size && loc_usart1_tx_buffer_data_len;
                i++, loc_usart1_tx_buffer_data_len--)
        {
            *usart1_write_pos++ = *usart1_rd_pos++;
            if(usart1_rd_pos >= tx_fifo + tx_fifo_size)
                usart1_rd_pos = tx_fifo;
        }
        *tx_fifo_data_len = loc_usart1_tx_buffer_data_len;
        *tx_sendig_data_len = i;
        LPUART_DRV_SendData(uartHandle, tx_sending_buffer, i);
    }

    __set_PRIMASK(primask);

    return len - remained_len;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.23 23:59:13 Saturday
  *******************************************************************************
  */
static void lpuart0_tx_callback(void *driverState, uart_event_t event, void *userData)
{
    switch(event)
    {
        case UART_EVENT_TX_EMPTY:
            break;

        case UART_EVENT_END_TRANSFER:
            uart_general_TxCpltCallback(INST_LPUART0, lpuart0_tx_cb, lpuart0_tx_fifo,
                                        sizeof(lpuart0_tx_fifo), &lpuart0_tx_fifo_data_len, lpuart0_tx_fifo_wr_pos,
                                        lpuart0_tx_sending_buffer, sizeof(lpuart0_tx_sending_buffer), &lpuart0_tx_sending_data_len);
            break;

        default:
            break;
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:00:41 Sunday
  *******************************************************************************
  */
static void lpuart0_rx_callback(void *driverState, uart_event_t event, void *userData)
{
    lpuart_state_t * lpuartState = (lpuart_state_t*)driverState;

    switch(event)
    {
        case UART_EVENT_RX_FULL:
            if(lpuart0_rx_cb)
                lpuart0_rx_cb(INST_LPUART0, lpuart0_rx_buffer);
            lpuartState->rxSize = 1;
            lpuartState->rxBuff = &lpuart0_rx_buffer;
            break;

        case UART_EVENT_END_TRANSFER:
        case UART_EVENT_ERROR:
            LPUART_DRV_ReceiveData(INST_LPUART0, &lpuart0_rx_buffer, 1);
            break;

        default:
            break;
    }
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 24:01:19 Sunday
  *******************************************************************************
  */
static void lpuart2_tx_callback(void *driverState, uart_event_t event, void *userData)
{
    int32_t ret;
    switch(event)
    {
        case UART_EVENT_TX_EMPTY:
            break;

        case UART_EVENT_END_TRANSFER:
            ret = uart_general_TxCpltCallback(INST_LPUART2, lpuart2_tx_cb, lpuart2_tx_fifo, sizeof(lpuart2_tx_fifo),
                                              &lpuart2_tx_fifo_data_len, lpuart2_tx_fifo_wr_pos,
                                              lpuart2_tx_sending_buffer, sizeof(lpuart2_tx_sending_buffer), &lpuart2_tx_sending_data_len);
            if(ret == 0)
            {
//                PINS_DRV_WritePin(PTD, 5, !RS485_TX_LEVEL);
            }
            break;

        default:
            break;
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:02:07 Sunday
  *******************************************************************************
  */
static void lpuart2_rx_callback(void *driverState, uart_event_t event, void *userData)
{
    lpuart_state_t * lpuartState = (lpuart_state_t*)driverState;

    switch(event)
    {
        case UART_EVENT_RX_FULL:
            if(lpuart2_rx_cb)
                lpuart2_rx_cb(INST_LPUART2, lpuart2_rx_buffer);
            lpuartState->rxSize = 1;
            lpuartState->rxBuff = &lpuart2_rx_buffer;
            break;

        case UART_EVENT_END_TRANSFER:
            break;

        case UART_EVENT_ERROR:
            LPUART_DRV_ReceiveData(INST_LPUART2, &lpuart2_rx_buffer, 1);
            break;

        default:
            break;
    }
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.23 23:57:00 Saturday
  *******************************************************************************
  */
int32_t LPUART0_init(void (*tx_cb)(uint32_t), void (*rx_cb)(uint32_t, uint32_t))
{
    lpuart0_tx_cb = tx_cb;
    lpuart0_rx_cb = rx_cb;
    LPUART_DRV_Init(INST_LPUART0, &lpuart0_State, &lpuart0_InitConfig0);
    LPUART_DRV_InstallTxCallback(INST_LPUART0, lpuart0_tx_callback, 0);
    LPUART_DRV_InstallRxCallback(INST_LPUART0, lpuart0_rx_callback, 0);
    LPUART_DRV_ReceiveData(INST_LPUART0, &lpuart0_rx_buffer, 1);

    if(lpuart0_InitConfig0.transferType == LPUART_USING_DMA)
    {
        INT_SYS_InstallHandler((IRQn_Type)lpuart0_InitConfig0.rxDMAChannel, get_DMA_IRQHandler(lpuart0_InitConfig0.rxDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpuart0_InitConfig0.rxDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

        INT_SYS_InstallHandler((IRQn_Type)lpuart0_InitConfig0.txDMAChannel, get_DMA_IRQHandler(lpuart0_InitConfig0.txDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpuart0_InitConfig0.txDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    }
    INT_SYS_SetPriority(LPUART0_RxTx_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:59:40 Sunday
  *******************************************************************************
  */
int32_t LPUART2_init(void (*tx_cb)(uint32_t), void (*rx_cb)(uint32_t, uint32_t))
{
    lpuart2_tx_cb = tx_cb;
    lpuart2_rx_cb = rx_cb;
    LPUART_DRV_Init(INST_LPUART2, &lpuart2_State, &lpuart2_InitConfig0);
    LPUART_DRV_InstallTxCallback(INST_LPUART2, lpuart2_tx_callback, 0);
    LPUART_DRV_InstallRxCallback(INST_LPUART2, lpuart2_rx_callback, 0);
    LPUART_DRV_ReceiveData(INST_LPUART2, &lpuart2_rx_buffer, 1);
    if(lpuart2_InitConfig0.transferType == LPUART_USING_DMA)
    {
        INT_SYS_InstallHandler((IRQn_Type)lpuart2_InitConfig0.rxDMAChannel, get_DMA_IRQHandler(lpuart2_InitConfig0.rxDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpuart2_InitConfig0.rxDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

        INT_SYS_InstallHandler((IRQn_Type)lpuart2_InitConfig0.txDMAChannel, get_DMA_IRQHandler(lpuart2_InitConfig0.txDMAChannel), 0);
        INT_SYS_SetPriority((IRQn_Type)lpuart2_InitConfig0.txDMAChannel, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    }
    INT_SYS_SetPriority(LPUART2_RxTx_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

    lpuart2_init_flag = 1;
    return 0;
}
/**
  *******************************************************************************
  * @Description: 关闭LPUART2
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : NickYang
  * @CreatedDate: 2024.03.09 13:28:33 Saturday
  *******************************************************************************
  */
int32_t LPUART2_deinit(void)
{
    lpuart2_init_flag = 0;
    LPUART_DRV_Deinit(INST_LPUART2);
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 00:00:31 Sunday
  *******************************************************************************
  */
int32_t LPUART0_send(uint8_t* data, uint32_t len)
{
    return  uart_general_send(INST_LPUART0, data, len,
                              lpuart0_tx_fifo, sizeof(lpuart0_tx_fifo), &lpuart0_tx_fifo_data_len,
                              &lpuart0_tx_fifo_wr_pos, lpuart0_tx_sending_buffer,
                              sizeof(lpuart0_tx_sending_buffer), &lpuart0_tx_sending_data_len);
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.24 23:09:29 Sunday
  *******************************************************************************
  */
int32_t LPUART2_send(uint8_t* data, uint32_t len)
{
    if(lpuart2_init_flag == 0)
        return -1;

//    PINS_DRV_WritePin(PTD, 5, RS485_TX_LEVEL);
    return  uart_general_send(INST_LPUART2, data, len,
                              lpuart2_tx_fifo, sizeof(lpuart2_tx_fifo), &lpuart2_tx_fifo_data_len,
                              &lpuart2_tx_fifo_wr_pos, lpuart2_tx_sending_buffer,
                              sizeof(lpuart2_tx_sending_buffer), &lpuart2_tx_sending_data_len);
}

