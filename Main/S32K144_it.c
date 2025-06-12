/**
  *****************************************************************************
  * FILENAME   : S32K144_it.c
  * COPYRIGHT  : XXXX(ShangHai) Co.,Ltd2023.
  * CREATEDDATE: 2023.09.09 15:04:35 Saturday
  * DESCRIPTION:
  *
  *
  * EDIT HISTORY:
  *   NAME                        DATE                      CONTENT
  * YangHaifeng               2023.09.09                    CREATE
  *****************************************************************************
  */
/************* Included files, Macros, Various and Declarations ***************/
#include "S32K144_it.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
extern uint32_t __initial_sp;

static void default_irq_handler(void);


static void (*irq_vector[])(void) __attribute__((used, __section__("IRQ_VECT"))) =
{
    (void(*)(void))&__initial_sp,// Top of Stack
    (void(*)(void))0,            // Reset Handler
    NMI_Handler,                 // NMI Handler
    HardFault_Handler,           // Hard Fault Handler
    MemManage_Handler,           // MPU Fault Handler
    BusFault_Handler,            // Bus Fault Handler
    UsageFault_Handler,          // Usage Fault Handler
    (void(*)(void))0,            // Reserved
    (void(*)(void))0,            // Reserved
    (void(*)(void))0,            // Reserved
    (void(*)(void))0,            // Reserved
    SVC_Handler,                 // SVCall Handler
    DebugMon_Handler,            // Debug Monitor Handler
    (void(*)(void))0,            // Reserved
    PendSV_Handler,              // PendSV Handler
    SysTick_Handler,             // SysTick Handler
    default_irq_handler,         // DMA channel 0 transfer complete
    default_irq_handler,         // DMA channel 1 transfer complete
    default_irq_handler,         // DMA channel 2 transfer complete
    default_irq_handler,         // DMA channel 3 transfer complete
    default_irq_handler,         // DMA channel 4 transfer complete
    default_irq_handler,         // DMA channel 5 transfer complete
    default_irq_handler,         // DMA channel 6 transfer complete
    default_irq_handler,         // DMA channel 7 transfer complete
    default_irq_handler,         // DMA channel 8 transfer complete
    default_irq_handler,         // DMA channel 9 transfer complete
    default_irq_handler,         // DMA channel 10 transfer complete
    default_irq_handler,         // DMA channel 11 transfer complete
    default_irq_handler,         // DMA channel 12 transfer complete
    default_irq_handler,         // DMA channel 13 transfer complete
    default_irq_handler,         // DMA channel 14 transfer complete
    default_irq_handler,         // DMA channel 15 transfer complete
    default_irq_handler,         // DMA error interrupt channels 0-15
    default_irq_handler,         // FPU sources
    default_irq_handler,         // FTFC Command complete
    default_irq_handler,         // FTFC Read collision
    default_irq_handler,         // PMC Low voltage detect interrupt
    default_irq_handler,         // FTFC Double bit fault detect
    default_irq_handler,         // Single interrupt vector for WDOG and EWM
    default_irq_handler,         // RCM Asynchronous Interrupt
    default_irq_handler,         // LPI2C0 Master Interrupt
    default_irq_handler,         // LPI2C0 Slave Interrupt
    default_irq_handler,         // LPSPI0 Interrupt
    default_irq_handler,         // LPSPI1 Interrupt
    default_irq_handler,         // LPSPI2 Interrupt
    default_irq_handler,         // Reserved Interrupt 45
    default_irq_handler,         // Reserved Interrupt 46
    default_irq_handler,         // LPUART0 Transmit / Receive Interrupt
    default_irq_handler,         // Reserved Interrupt 48
    default_irq_handler,         // LPUART1 Transmit / Receive  Interrupt
    default_irq_handler,         // Reserved Interrupt 50
    default_irq_handler,         // LPUART2 Transmit / Receive  Interrupt
    default_irq_handler,         // Reserved Interrupt 52
    default_irq_handler,         // Reserved Interrupt 53
    default_irq_handler,         // Reserved Interrupt 54
    default_irq_handler,         // ADC0 interrupt request.
    default_irq_handler,         // ADC1 interrupt request.
    default_irq_handler,         // CMP0 interrupt request
    default_irq_handler,         // Reserved Interrupt 58
    default_irq_handler,         // Reserved Interrupt 59
    default_irq_handler,         // ERM single bit error correction
    default_irq_handler,         // ERM double bit error non-correctable
    default_irq_handler,         // RTC alarm interrupt
    default_irq_handler,         // RTC seconds interrupt
    default_irq_handler,         // LPIT0 channel 0 overflow interrupt
    default_irq_handler,         // LPIT0 channel 1 overflow interrupt
    default_irq_handler,         // LPIT0 channel 2 overflow interrupt
    default_irq_handler,         // LPIT0 channel 3 overflow interrupt
    default_irq_handler,         // PDB0 interrupt
    default_irq_handler,         // Reserved Interrupt 69
    default_irq_handler,         // Reserved Interrupt 70
    default_irq_handler,         // Reserved Interrupt 71
    default_irq_handler,         // Reserved Interrupt 72
    default_irq_handler,         // SCG bus interrupt request
    default_irq_handler,         // LPTIMER interrupt request
    default_irq_handler,         // Port A pin detect interrupt
    default_irq_handler,         // Port B pin detect interrupt
    default_irq_handler,         // Port C pin detect interrupt
    default_irq_handler,         // Port D pin detect interrupt
    default_irq_handler,         // Port E pin detect interrupt
    default_irq_handler,         // Software interrupt
    default_irq_handler,         // Reserved Interrupt 81
    default_irq_handler,         // Reserved Interrupt 82
    default_irq_handler,         // Reserved Interrupt 83
    default_irq_handler,         // PDB1 interrupt
    default_irq_handler,         // FlexIO Interrupt
    default_irq_handler,         // Reserved Interrupt 86
    default_irq_handler,         // Reserved Interrupt 87
    default_irq_handler,         // Reserved Interrupt 88
    default_irq_handler,         // Reserved Interrupt 89
    default_irq_handler,         // Reserved Interrupt 90
    default_irq_handler,         // Reserved Interrupt 91
    default_irq_handler,         // Reserved Interrupt 92
    default_irq_handler,         // Reserved Interrupt 93
    default_irq_handler,         // CAN0 OR'ed [Bus Off OR Transmit Warning OR Receive Warning]
    default_irq_handler,         // CAN0 Interrupt indicating that errors were detected on the CAN bus
    default_irq_handler,         // CAN0 Interrupt asserted when Pretended Networking operation is enabled, and a valid message matches the selected filter criteria during Low Power mode
    default_irq_handler,         // CAN0 OR'ed Message buffer (0-15)
    default_irq_handler,         // CAN0 OR'ed Message buffer (16-31)
    default_irq_handler,         // Reserved Interrupt 99
    default_irq_handler,         // Reserved Interrupt 100
    default_irq_handler,         // CAN1 OR'ed [Bus Off OR Transmit Warning OR Receive Warning]
    default_irq_handler,         // CAN1 Interrupt indicating that errors were detected on the CAN bus
    default_irq_handler,         // Reserved Interrupt 103
    default_irq_handler,         // CAN1 OR'ed Interrupt for Message buffer (0-15)
    default_irq_handler,         // Reserved Interrupt 105
    default_irq_handler,         // Reserved Interrupt 106
    default_irq_handler,         // Reserved Interrupt 107
    default_irq_handler,         // CAN2 OR'ed [Bus Off OR Transmit Warning OR Receive Warning]
    default_irq_handler,         // CAN2 Interrupt indicating that errors were detected on the CAN bus
    default_irq_handler,         // Reserved Interrupt 110
    default_irq_handler,         // CAN2 OR'ed Message buffer (0-15)
    default_irq_handler,         // Reserved Interrupt 112
    default_irq_handler,         // Reserved Interrupt 113
    default_irq_handler,         // Reserved Interrupt 114
    default_irq_handler,         // FTM0 Channel 0 and 1 interrupt
    default_irq_handler,         // FTM0 Channel 2 and 3 interrupt
    default_irq_handler,         // FTM0 Channel 4 and 5 interrupt
    default_irq_handler,         // FTM0 Channel 6 and 7 interrupt
    default_irq_handler,         // FTM0 Fault interrupt
    default_irq_handler,         // FTM0 Counter overflow and Reload interrupt
    default_irq_handler,         // FTM1 Channel 0 and 1 interrupt
    default_irq_handler,         // FTM1 Channel 2 and 3 interrupt
    default_irq_handler,         // FTM1 Channel 4 and 5 interrupt
    default_irq_handler,         // FTM1 Channel 6 and 7 interrupt
    default_irq_handler,         // FTM1 Fault interrupt
    default_irq_handler,         // FTM1 Counter overflow and Reload interrupt
    default_irq_handler,         // FTM2 Channel 0 and 1 interrupt
    default_irq_handler,         // FTM2 Channel 2 and 3 interrupt
    default_irq_handler,         // FTM2 Channel 4 and 5 interrupt
    default_irq_handler,         // FTM2 Channel 6 and 7 interrupt
    default_irq_handler,         // FTM2 Fault interrupt
    default_irq_handler,         // FTM2 Counter overflow and Reload interrupt
    default_irq_handler,         // FTM3 Channel 0 and 1 interrupt
    default_irq_handler,         // FTM3 Channel 2 and 3 interrupt
    default_irq_handler,         // FTM3 Channel 4 and 5 interrupt
    default_irq_handler,         // FTM3 Channel 6 and 7 interrupt
    default_irq_handler,         // FTM3 Fault interrupt
    default_irq_handler,         // FTM3 Counter overflow and Reload interrupt
    default_irq_handler,         // 139
    default_irq_handler,         // 140
    default_irq_handler,         // 141
    default_irq_handler,         // 142
    default_irq_handler,         // 143
    default_irq_handler,         // 144
    default_irq_handler,         // 145
    default_irq_handler,         // 146
    default_irq_handler,         // 147
    default_irq_handler,         // 148
    default_irq_handler,         // 149
    default_irq_handler,         // 150
    default_irq_handler,         // 151
    default_irq_handler,         // 152
    default_irq_handler,         // 153
    default_irq_handler,         // 154
    default_irq_handler,         // 155
    default_irq_handler,         // 156
    default_irq_handler,         // 157
    default_irq_handler,         // 158
    default_irq_handler,         // 159
    default_irq_handler,         // 160
    default_irq_handler,         // 161
    default_irq_handler,         // 162
    default_irq_handler,         // 163
    default_irq_handler,         // 164
    default_irq_handler,         // 165
    default_irq_handler,         // 166
    default_irq_handler,         // 167
    default_irq_handler,         // 168
    default_irq_handler,         // 169
    default_irq_handler,         // 170
    default_irq_handler,         // 171
    default_irq_handler,         // 172
    default_irq_handler,         // 173
    default_irq_handler,         // 174
    default_irq_handler,         // 175
    default_irq_handler,         // 176
    default_irq_handler,         // 177
    default_irq_handler,         // 178
    default_irq_handler,         // 179
    default_irq_handler,         // 180
    default_irq_handler,         // 181
    default_irq_handler,         // 182
    default_irq_handler,         // 183
    default_irq_handler,         // 184
    default_irq_handler,         // 185
    default_irq_handler,         // 186
    default_irq_handler,         // 187
    default_irq_handler,         // 188
    default_irq_handler,         // 189
    default_irq_handler,         // 190
    default_irq_handler,         // 191
    default_irq_handler,         // 192
    default_irq_handler,         // 193
    default_irq_handler,         // 194
    default_irq_handler,         // 195
    default_irq_handler,         // 196
    default_irq_handler,         // 197
    default_irq_handler,         // 198
    default_irq_handler,         // 199
    default_irq_handler,         // 200
    default_irq_handler,         // 201
    default_irq_handler,         // 202
    default_irq_handler,         // 203
    default_irq_handler,         // 204
    default_irq_handler,         // 205
    default_irq_handler,         // 206
    default_irq_handler,         // 207
    default_irq_handler,         // 208
    default_irq_handler,         // 209
    default_irq_handler,         // 210
    default_irq_handler,         // 211
    default_irq_handler,         // 212
    default_irq_handler,         // 213
    default_irq_handler,         // 214
    default_irq_handler,         // 215
    default_irq_handler,         // 216
    default_irq_handler,         // 217
    default_irq_handler,         // 218
    default_irq_handler,         // 219
    default_irq_handler,         // 220
    default_irq_handler,         // 221
    default_irq_handler,         // 222
    default_irq_handler,         // 223
    default_irq_handler,         // 224
    default_irq_handler,         // 225
    default_irq_handler,         // 226
    default_irq_handler,         // 227
    default_irq_handler,         // 228
    default_irq_handler,         // 229
    default_irq_handler,         // 230
    default_irq_handler,         // 231
    default_irq_handler,         // 232
    default_irq_handler,         // 233
    default_irq_handler,         // 234
    default_irq_handler,         // 235
    default_irq_handler,         // 236
    default_irq_handler,         // 237
    default_irq_handler,         // 238
    default_irq_handler,         // 239
    default_irq_handler,         // 240
    default_irq_handler,         // 241
    default_irq_handler,         // 242
    default_irq_handler,         // 243
    default_irq_handler,         // 244
    default_irq_handler,         // 245
    default_irq_handler,         // 246
    default_irq_handler,         // 247
    default_irq_handler,         // 248
    default_irq_handler,         // 249
    default_irq_handler,         // 250
    default_irq_handler,         // 251
    default_irq_handler,         // 252
    default_irq_handler,         // 253
    default_irq_handler,         // 254
    (void(*)(void))0xFFFFFFFF      // Reserved for user TRIM value
};
/******************************** Functions **********************************/
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.18 23:21:35 Monday
  *******************************************************************************
  */
uint32_t get_irq_vect_addr(void)
{
    return (uint32_t)irq_vector;
}
/**
  *******************************************************************************
  * @Description: 中断注册函数
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.08.30 21:45:44 Wed
  *******************************************************************************
  */
int32_t register_irq_handler(uint32_t irq_no, void(*irq_handler)(void))
{
    irq_no += 16;

    if(irq_no > sizeof(irq_vector) / sizeof(irq_vector[0]))
        return -1;
    if(irq_vector[irq_no] != default_irq_handler)
        return -2;

    irq_vector[irq_no] = irq_handler;
    return 0;
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHF
  * @CreatedDate: 2023.08.30 21:45:57 Wed
  *******************************************************************************
  */
int32_t unregister_irq_handler(uint32_t irq_no)
{
    irq_no += 16;
    irq_vector[irq_no] = default_irq_handler;
    return 0;
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHf
  * @CreatedDate: 2023.09.06 19:45:59 Wednesday
  *******************************************************************************
  */
void NMI_Handler(void)
{
    while(1)
    {
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHf
  * @CreatedDate: 2023.09.06 19:46:23 Wednesday
  *******************************************************************************
  */
void HardFault_Handler(void)
{
    while(1)
    {
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHf
  * @CreatedDate: 2023.09.06 19:46:52 Wednesday
  *******************************************************************************
  */
void MemManage_Handler(void)
{
    while(1)
    {
    }
}
/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHf
  * @CreatedDate: 2023.09.06 19:47:10 Wednesday
  *******************************************************************************
  */
void BusFault_Handler(void)
{
    while(1)
    {
    }
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHf
  * @CreatedDate: 2023.09.06 19:47:30 Wednesday
  *******************************************************************************
  */
void UsageFault_Handler(void)
{
    while(1)
    {
    }
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHf
  * @CreatedDate: 2023.09.06 19:47:49 Wednesday
  *******************************************************************************
  */
void DebugMon_Handler(void)
{
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.09 15:27:50 Saturday
  *******************************************************************************
  */
static void default_irq_handler(void)
{
    while(1);
}

/**
  *******************************************************************************
  * @Description:
  * @Parameters :
  * @RetValue   :
  * @Note       :

  * @CreatedBy  : YangHaifeng
  * @CreatedDate: 2023.09.18 22:16:55 Monday
  *******************************************************************************
  */
void SysTick_Handler(void)
{
    xPortSysTickHandler();
}

