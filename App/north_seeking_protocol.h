/**
 *******************************************************************************
 * @file    north_seeking_protocol.h
 * @brief   MG-01 寻北仪协议：测斜模组通过 LPUART2 与寻北模组通信
 *
 * 接收（寻北模组 → 测斜模组）：96 字节数据帧
 * 发送（测斜模组 → 寻北模组）：32 字节装订指令（NSL= 触发）
 *******************************************************************************
 */

#ifndef _NORTH_SEEKING_PROTOCOL_H_
#define _NORTH_SEEKING_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---------- 发送：测斜模组 -> 寻北模组 装订指令帧 ---------- */
/* 长度 40 字节，字节 [2] 起参与累加，最后 1 字节为累加和低 8 位 */
#define NS_FRAME_CMD_LEN         40

/* 装订帧内偏移 */
#define NS_CMD_OFFSET_LON         3   /* 经度 4 字节小端 int32，单位 0.0000001 度 */
#define NS_CMD_OFFSET_LAT         7   /* 纬度 4 字节小端 int32 */

/* ---------- 接收：寻北模组 -> 测斜模组 数据帧（96 字节） ---------- */
#define NS_FRAME_DATA_HEADER_0   0x55
#define NS_FRAME_DATA_HEADER_1   0xAA
#define NS_FRAME_DATA_LEN_FIELD  0x5C /* 字节 [2] 固定，表示数据段长度 92 字节 */
#define NS_FRAME_DATA_TOTAL      96

/* 数据帧内偏移 */
#define NS_DATA_OFFSET_SYS_STATUS     7
#define NS_DATA_OFFSET_OUT_CH         8
#define NS_DATA_OFFSET_OVER_RANGE    9
#define NS_DATA_OFFSET_RESP_STATUS   10
#define NS_DATA_OFFSET_NS_TOTAL_T    11  /* 寻北总时间 uint16 */
#define NS_DATA_OFFSET_NS_REMAIN_T   13  /* 剩余寻北时间 uint16 */
#define NS_DATA_OFFSET_GYRO_X       15  /* 角速率 X int32，当量 0.0000005 °/s */
#define NS_DATA_OFFSET_GYRO_Y       19
#define NS_DATA_OFFSET_GYRO_Z       23
#define NS_DATA_OFFSET_ACC_X        27  /* 加速度 int32，当量 0.000001 m/s² */
#define NS_DATA_OFFSET_ACC_Y        31
#define NS_DATA_OFFSET_ACC_Z        35
#define NS_DATA_OFFSET_LON          39  /* 经度 int32，当量 0.0000001 */
#define NS_DATA_OFFSET_LAT          43
#define NS_DATA_OFFSET_ALT          47  /* 高度 uint16，当量 0.5 m */
#define NS_DATA_OFFSET_NORTH_SPD    49  /* 北速 uint16，当量 0.04 m/s */
#define NS_DATA_OFFSET_SKY_SPD      51
#define NS_DATA_OFFSET_EAST_SPD     53
#define NS_DATA_OFFSET_GRAV_TF      55  /* 重力工具面角 uint16，当量 0.006 ° */
#define NS_DATA_OFFSET_INC_AZIMUTH  57  /* 井斜方位角 uint16，当量 0.006 ° */
#define NS_DATA_OFFSET_INC_ANGLE    59  /* 井斜角 uint16，当量 0.006 ° */
#define NS_DATA_OFFSET_GYRO_TEMP    61
#define NS_DATA_OFFSET_ACC_TEMP     62
#define NS_DATA_OFFSET_GYRO_TF      89  /* 陀螺工具面角 uint16，当量 0.006 ° */
#define NS_DATA_OFFSET_CHECKSUM     95  /* 校验和：字节 2~94 累加和低 8 位 */

/* 当量（与上位机一致） */
#define NS_SCALE_ANGULAR_RATE   0.0000005
#define NS_SCALE_ACCEL          0.000001
#define NS_SCALE_LONLAT         0.0000001
#define NS_SCALE_ANGLE          0.006
#define NS_SCALE_ALTITUDE       0.5     /* 高度当量 m */
#define NS_SCALE_SPEED          0.04    /* 速度当量 m/s */

/**
 * @brief 96 字节数据帧解析后的结构体（供 north_seeking_parse_rx_frame 解析与打印）
 */
typedef struct {
    uint8_t  sys_status;    /* 系统工作状态：
                             * 0x10 自检中，0x12 自检好，0x1F 自检错误，
                             * 0x33 粗对准，0x35 精对准，0x41 导航中，0x43 零速修正 */
    uint8_t  out_ch;        /* 输出通道：
                             * 0 = 晃动寻北通道，
                             * 1 = 纯静态寻北通道 */
    uint8_t  over_range;    /* 超量程状态位（产品进入对准后，一个导航周期 5ms 内）：
                             * bit0 X 陀螺超量程置 1，bit1 Y 陀螺超量程置 1，
                             * bit2 Z 陀螺超量程置 1，bit3 X 加表超量程置 1，
                             * bit4 Y 加表超量程置 1，bit5 Z 加表超量程置 1；
                             * 重新对准后上述状态清 0 */
    uint8_t  resp_status;   /* 响应命令状态：
                             * 0x10 = 响应正确指令成功（连续发送 5 包后软重置），
                             * 0x1F = 指令错误（连续发送 5 包提醒检查指令/输出配置，保持原状态） */
    uint16_t ns_total_t;    /* 寻北总时间，单位：秒 */
    uint16_t ns_remain_t;   /* 剩余寻北时间，单位：秒 */
    float    gx_dps;      /* 角速率 °/s */
    float    gy_dps;
    float    gz_dps;
    float    ax_ms2;      /* 加速度 m/s² */
    float    ay_ms2;
    float    az_ms2;
    double   lon_deg;      /* 经度，单位：度（= 原始值 * 0.0000001） */
    double   lat_deg;      /* 纬度，单位：度（= 原始值 * 0.0000001） */
    float    alt;           /* 高度 m（= 原始值 * 0.5，与上位机一致） */
    float    north_spd;     /* 北速 m/s（= 原始值 * 0.04） */
    float    sky_spd;       /* 天速 m/s */
    float    east_spd;      /* 东速 m/s */
    float    grav_tf;     /* 重力工具面角 ° */
    float    inc_azimuth; /* 井斜方位角 ° */
    float    inc_angle;   /* 井斜角 ° */
    uint8_t  gyro_temp;     /* 陀螺温度（原始温度字节） */
    uint8_t  acc_temp;      /* 加速度计温度（原始温度字节） */
    float    gyro_tf;     /* 陀螺工具面角 ° */
} north_seeking_parsed_data_t;

/* ---------- 测斜模组与寻北模组之间通信（统一在本模块处理） ---------- */
/**
 * @brief 测斜模组接收处理：接收 96 字节数据帧，校验通过后原样存储
 * @param buf 接收缓冲区（会被 memmove 更新）
 * @param len 缓冲区数据长度（入参/出参）
 */
void north_seeking_handle_rx(uint8_t *buf, uint32_t *len);

/**
 * @brief 发送装订指令给寻北模组（经 LPUART2_send_internal）
 * @param lon_deg 经度（度）
 * @param lat_deg 纬度（度）
 * @note  由 uart_service 收到 NSL 命令时调用
 */
void north_seeking_send_compass_command(double lon_deg, double lat_deg);

/**
 * @brief 获取已存储的 96 字节帧（原样，供 NS=? 发送给上位机）
 * @param out 输出缓冲区，至少 96 字节；若尚未收到则填 0
 * @return 1 表示有有效数据，0 表示尚未收到
 */
int north_seeking_get_rx_frame(uint8_t *out);

/**
 * @brief 解析已存储的 96 字节帧到模块内全局 s_north_seeking_parsed_data 并打印到 LPUART0
 * @return 0 成功，-1 失败或尚未收到有效帧
 */
int north_seeking_parse_rx_frame(void);


#ifdef __cplusplus
}
#endif

#endif /* _NORTH_SEEKING_PROTOCOL_H_ */
