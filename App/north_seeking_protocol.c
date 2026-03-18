/**
 *******************************************************************************
 * @file    north_seeking_protocol.c
 * @brief   MG-01 寻北仪协议：测斜模组通过 LPUART2 与寻北模组通信
 *
 * 接收（寻北模组 → 测斜模组）：
 *   - north_seeking_handle_rx：接收 96 字节数据帧，校验通过后原样存储
 *   - north_seeking_get_rx_frame：供 NS=? 原样发送给上位机
 *   - north_seeking_parse_rx_frame：解析并打印到 LPUART0（用到时调用）
 *
 * 发送（测斜模组 → 寻北模组）：
 *   - north_seeking_send_compass_command：经 LPUART2_send_internal 发 32 字节装订指令
 *   - 触发：uart_service 收到 NSL=经度,纬度 时调用
 *
 * 硬件接线（全双工 RS-422）：
 *   - 寻北仪接口：T-、T+、R-、R+
 *   - 测斜模组 M1 接口：A、B、Z、Y
 *   - 连接关系：A 接 T+，B 接 T-，Z 接 R-，Y 接 R+
 *******************************************************************************
 */

#include "main.h"
#include "north_seeking_protocol.h"
#include "lpuart.h"
#include <stdio.h>

/* 来自寻北仪的 96 字节数据：收到后原样存储，NS=? 时原样发送；用到时再解析 */
static uint8_t s_north_seeking_rx_frame[NS_FRAME_DATA_TOTAL];
static uint8_t s_north_seeking_rx_frame_valid;  /* 1=有有效帧，0=无 */

/* 解析后的寻北数据：每次收到有效 96 字节帧并解析成功后更新，调试时可查看 */
static north_seeking_parsed_data_t s_north_seeking_parsed_data;

/* 装订指令模板（经度/纬度/校验和由运行时填充）
 * 帧格式（字节偏移）：
 *   0   : 0x55
 *   1   : 0xAA
 *   2   : 0x24
 *   3-6 : 经度 int32 LE，单位 0.0000001 度（运行时填充）
 *   7-10: 纬度 int32 LE，单位 0.0000001 度（运行时填充）
 *   11-14: 高度等，示例：0x00 0x00 0x5C 0x42
 *   15-18: 备份
 *   19-20: 数据有效模式 0x05 0x00
 *   21-24: 备份
 *   25   : 输出波特率配置
 *   26   : 输出周期
 *   27-32: 安装误差角、备份
 *   33-34: 备份
 *   35-36: 用户命令 0x11 0x00
 *   37-38: 对准时间 0x8C 0x0A
 *   39   : 校验和，占位，运行时计算（字节 2~38 累加和低 8 位）
 */
static const uint8_t s_compass_cmd_template[NS_FRAME_CMD_LEN] = {
    0x55, 0xAA, 0x24,
    0x00, 0x00, 0x00, 0x00,       /* 经度，占位，运行时填充 */
    0x00, 0x00, 0x00, 0x00,       /* 纬度，占位，运行时填充 */
    0x00, 0x00, 0x5C, 0x42,       /* 高度等，示例值 */
    0x00, 0x00, 0x00, 0x00,       /* 备份1 */
    0x05, 0x00,                   /* 数据有效模式 */
    0x00, 0x00, 0x00, 0x00,       /* 备份2/3 */
    0x00,                         /* 输出波特率配置 */
    0x00,                         /* 输出周期 */
    0x00, 0x00,                   /* 滚动安装误差角 */
    0x00, 0x00,                   /* 航向安装误差角 */
    0x00, 0x00,                   /* 俯仰安装误差角 */
    0x00, 0x00,                   /* 备份4 */
    0x11, 0x00,                   /* 用户命令 */
    0x8C, 0x0A,                   /* 对准时间 */
    0x00                          /* 校验和，占位，运行时计算 */
};

/* ---------- 功能实现函数 ---------- */
/* 小端读取 */
static int32_t get_i32_le(const uint8_t *buf)
{
    return (int32_t)((uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                     ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
}
static uint16_t get_u16_le(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

static void put_u32_le(uint8_t *buf, uint32_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
    buf[2] = (uint8_t)((v >> 16) & 0xFF);
    buf[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_i32_le(uint8_t *buf, int32_t v)
{
    put_u32_le(buf, (uint32_t)v);
}

/* 装订指令校验和：字节 2 ~(NS_FRAME_CMD_LEN-2) 累加和低 8 位（与上位机一致） */
static uint8_t ns_cmd_calc_checksum(const uint8_t *frame)
{
    uint32_t sum = 0;
    int i;

    for (i = 2; i <= NS_FRAME_CMD_LEN - 2; i++)
        sum += frame[i];

    return (uint8_t)(sum & 0xFF);
}

/* 计算数据帧校验和：字节 2~94 累加和低 8 位 */
static uint8_t calc_checksum(const uint8_t *frame)
{
    uint32_t sum = 0;
    for (int i = 2; i <= 94; i++)
        sum += frame[i];
    return (uint8_t)(sum & 0xFF);
}

/**
 * @brief 解析 s_north_seeking_rx_frame 到 s_north_seeking_parsed_data
 * @return 0 成功，-1 失败
 */
static int parse_north_seeking_data_frame(void)
{
    const uint8_t *buf = s_north_seeking_rx_frame;

    if (buf[0] != NS_FRAME_DATA_HEADER_0 || buf[1] != NS_FRAME_DATA_HEADER_1 ||
        buf[2] != NS_FRAME_DATA_LEN_FIELD)
        return -1;
    if (buf[NS_DATA_OFFSET_CHECKSUM] != calc_checksum(buf))
        return -1;

    s_north_seeking_parsed_data.sys_status  = buf[NS_DATA_OFFSET_SYS_STATUS];
    s_north_seeking_parsed_data.out_ch     = buf[NS_DATA_OFFSET_OUT_CH];
    s_north_seeking_parsed_data.over_range = buf[NS_DATA_OFFSET_OVER_RANGE];
    s_north_seeking_parsed_data.resp_status = buf[NS_DATA_OFFSET_RESP_STATUS];
    s_north_seeking_parsed_data.ns_total_t = get_u16_le(buf + NS_DATA_OFFSET_NS_TOTAL_T);
    s_north_seeking_parsed_data.ns_remain_t = get_u16_le(buf + NS_DATA_OFFSET_NS_REMAIN_T);
    s_north_seeking_parsed_data.gx_dps = (float)(get_i32_le(buf + NS_DATA_OFFSET_GYRO_X) * (double)NS_SCALE_ANGULAR_RATE);
    s_north_seeking_parsed_data.gy_dps = (float)(get_i32_le(buf + NS_DATA_OFFSET_GYRO_Y) * (double)NS_SCALE_ANGULAR_RATE);
    s_north_seeking_parsed_data.gz_dps = (float)(get_i32_le(buf + NS_DATA_OFFSET_GYRO_Z) * (double)NS_SCALE_ANGULAR_RATE);
    s_north_seeking_parsed_data.ax_ms2 = (float)(get_i32_le(buf + NS_DATA_OFFSET_ACC_X) * (double)NS_SCALE_ACCEL);
    s_north_seeking_parsed_data.ay_ms2 = (float)(get_i32_le(buf + NS_DATA_OFFSET_ACC_Y) * (double)NS_SCALE_ACCEL);
    s_north_seeking_parsed_data.az_ms2 = (float)(get_i32_le(buf + NS_DATA_OFFSET_ACC_Z) * (double)NS_SCALE_ACCEL);
    s_north_seeking_parsed_data.lon_deg = (double)get_i32_le(buf + NS_DATA_OFFSET_LON) * NS_SCALE_LONLAT;
    s_north_seeking_parsed_data.lat_deg = (double)get_i32_le(buf + NS_DATA_OFFSET_LAT) * NS_SCALE_LONLAT;
    s_north_seeking_parsed_data.alt = (float)(get_u16_le(buf + NS_DATA_OFFSET_ALT) * NS_SCALE_ALTITUDE);
    s_north_seeking_parsed_data.north_spd = (float)(get_u16_le(buf + NS_DATA_OFFSET_NORTH_SPD) * NS_SCALE_SPEED);
    s_north_seeking_parsed_data.sky_spd = (float)(get_u16_le(buf + NS_DATA_OFFSET_SKY_SPD) * NS_SCALE_SPEED);
    s_north_seeking_parsed_data.east_spd = (float)(get_u16_le(buf + NS_DATA_OFFSET_EAST_SPD) * NS_SCALE_SPEED);
    s_north_seeking_parsed_data.grav_tf = (float)(get_u16_le(buf + NS_DATA_OFFSET_GRAV_TF) * NS_SCALE_ANGLE);
    s_north_seeking_parsed_data.inc_azimuth = (float)(get_u16_le(buf + NS_DATA_OFFSET_INC_AZIMUTH) * NS_SCALE_ANGLE);
    s_north_seeking_parsed_data.inc_angle = (float)(get_u16_le(buf + NS_DATA_OFFSET_INC_ANGLE) * NS_SCALE_ANGLE);
    s_north_seeking_parsed_data.gyro_temp = buf[NS_DATA_OFFSET_GYRO_TEMP];
    s_north_seeking_parsed_data.acc_temp = buf[NS_DATA_OFFSET_ACC_TEMP];
    s_north_seeking_parsed_data.gyro_tf = (float)(get_u16_le(buf + NS_DATA_OFFSET_GYRO_TF) * NS_SCALE_ANGLE);
    return 0;
}

/* ---------- 任务函数（对外接口） ---------- */
/**
 * @brief 测斜模组接收处理：处理从寻北模组收到的 96 字节数据帧（仿 handle_uart_msg 参数风格）
 * @param buf 接收缓冲区（会被 memmove 更新）
 * @param len 缓冲区数据长度（入参/出参）
 * @note  查找 55 AA 帧头，凑满 96 字节且校验通过则原样存储并移除；未消费数据保留
 */
void north_seeking_handle_rx(uint8_t *buf, uint32_t *len)
{
    if (!buf || !len || *len < 2)
        return;
    while (*len >= NS_FRAME_DATA_TOTAL)
    {
        if (buf[0] != NS_FRAME_DATA_HEADER_0 || buf[1] != NS_FRAME_DATA_HEADER_1)
        {
            int i;
            for (i = 1; i <= (int)*len - 2; i++)
            {
                if (buf[i] == NS_FRAME_DATA_HEADER_0 && buf[i + 1] == NS_FRAME_DATA_HEADER_1)
                    break;
            }
            if (i > (int)*len - 2)
                break;
            memmove(buf, buf + i, *len - i);
            *len -= i;
            continue;
        }
        if (buf[2] != NS_FRAME_DATA_LEN_FIELD)  /* 字节 [2] 固定 0x5C(92) */
        {
            memmove(buf, buf + 3, *len - 3);
            *len -= 3;
            continue;
        }
        uint8_t ck = calc_checksum(buf);
        if (buf[NS_DATA_OFFSET_CHECKSUM] != ck)
        {
            memmove(buf, buf + 3, *len - 3);
            *len -= 3;
            continue;
        }
        /* 校验通过，原样存储 96 字节，供 NS=? 发送；用到时再解析 */
        memcpy(s_north_seeking_rx_frame, buf, NS_FRAME_DATA_TOTAL);
        s_north_seeking_rx_frame_valid = 1;

        /* 立即解析并写入 s_north_seeking_parsed_data 供调试查看，同时通过 LPUART0 打印一次 */
        (void)north_seeking_parse_rx_frame();

        memmove(buf, buf + NS_FRAME_DATA_TOTAL, *len - NS_FRAME_DATA_TOTAL);
        *len -= NS_FRAME_DATA_TOTAL;
    }
}

/**
 * @brief 发送装订指令给寻北模组（经 LPUART2_send_internal，测斜↔寻北全双工）
 * @param lon_deg 经度（度）
 * @param lat_deg 纬度（度）
 * @note  由 uart_service 收到 NSL 命令时调用
 */
void north_seeking_send_compass_command(double lon_deg, double lat_deg)
{
    uint8_t frame[NS_FRAME_CMD_LEN];
    memcpy(frame, s_compass_cmd_template, NS_FRAME_CMD_LEN);
    put_i32_le(frame + NS_CMD_OFFSET_LON, (int32_t)(lon_deg / NS_SCALE_LONLAT));
    put_i32_le(frame + NS_CMD_OFFSET_LAT, (int32_t)(lat_deg / NS_SCALE_LONLAT));
    frame[NS_FRAME_CMD_LEN - 1] = ns_cmd_calc_checksum(frame);
    LPUART2_send(frame, NS_FRAME_CMD_LEN);
}

/**
 * @brief 获取收到的寻北 96 字节帧（原样，供 NS=? 发送给上位机）
 * @param out 输出缓冲区，至少 96 字节；若尚未收到则填 0
 * @return 1 表示有有效数据，0 表示尚未收到
 */
int north_seeking_get_rx_frame(uint8_t *out)
{
    if (!out)
        return 0;
    if (s_north_seeking_rx_frame_valid)
    {
        memcpy(out, s_north_seeking_rx_frame, NS_FRAME_DATA_TOTAL);
        return 1;
    }
    memset(out, 0, NS_FRAME_DATA_TOTAL);
    return 0;
}

/**
 * @brief 解析已存储的 96 字节帧为结构体并打印到 LPUART0（用到时再调用）
 */
int north_seeking_parse_rx_frame(void)
{
    if (!s_north_seeking_rx_frame_valid)
        return -1;
    if (parse_north_seeking_data_frame() != 0)
        return -1;
    return 0;
}
