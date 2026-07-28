/**
 * @file    test_emcy_callback.c
 * @brief   演示通过原始 CAN 回调捕获 EMCY 紧急事件帧
 *
 * 电机发生故障时会主动发送 EMCY 帧 (COB-ID = 0x080 + node_id)。
 * 本示例通过注册 CAN 接收回调来捕获并解析 EMCY 帧。
 *
 * EMCY 帧数据格式 (8 字节):
 *   [0-1] 错误码 (CiA 402), [2] 错误寄存器, [3-7] 制造商数据
 *
 * 用法: ./example_emcy_callback [can_if] [node_id]
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;

static void signal_handler(int sig) { (void)sig; g_quit = true; }

/*====================================================================
 * CAN 接收回调: 过滤并解析 EMCY 帧
 *====================================================================*/
static void on_can_recv(uint32_t can_id, const uint8_t *data,
                         uint8_t len, void *user_data)
{
    (void)user_data;

    /* EMCY 帧 COB-ID = 0x080 + node_id, 数据长度 8 字节 */
    if ((can_id & 0xF80) != 0x080 || len < 2)
        return;  /* 不是 EMCY 帧 */

    uint8_t node_id = (uint8_t)(can_id & 0x7F);
    uint16_t err_code = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    uint8_t  err_reg  = data[2];

    const char *desc;
    switch (err_code) {
    case 0x0000: desc = "No error";           break;
    case 0x1000: desc = "Generic error";      break;
    case 0x2310: desc = "Overcurrent";        break;
    case 0x2320: desc = "Overvoltage";        break;
    case 0x2330: desc = "Overtemperature";    break;
    case 0x6300: desc = "Device profile";     break;
    case 0xFF00: desc = "Monitoring error";   break;
    default:     desc = "Unknown";            break;
    }

    printf("\n[EMCY] node=%u  code=0x%04X (%s)  reg=0x%02X",
           node_id, err_code, desc, err_reg);

    if (len > 3) {
        printf("  mfg=");
        for (uint8_t i = 3; i < len; i++)
            printf("%02X ", data[i]);
    }
    printf("\n");
}

/*====================================================================
 * 主函数
 *====================================================================*/
int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* --- 打开电机 (内部创建 CAN 上下文) --- */
    MiraMotor *motor = miraculous_motor_open(ifname, 0, (uint8_t)node_id);
    if (!motor) {
        fprintf(stderr, "Failed to open motor on %s node %d\n",
                ifname, node_id);
        return -1;
    }
    printf("Motor opened: %s node %d\n", ifname, node_id);

    /* --- 获取 CAN 上下文, 注册接收回调 --- */
    MiraCanCtx *can = miraculous_motor_get_can_ctx(motor);
    if (!can) {
        fprintf(stderr, "Failed to get CAN context\n");
        miraculous_motor_close(motor);
        return -1;
    }

    miraculous_can_set_recv_callback(can, on_can_recv, NULL);
    printf("EMCY monitor started (listening on COB-ID 0x080+%d)...\n", node_id);
    printf("Press Ctrl+C to exit.\n");

    /* --- 等待 EMCY 事件 (由 recv 线程处理) --- */
    while (!g_quit) {
        usleep(100000);
    }

    miraculous_motor_close(motor);
    return 0;
}
