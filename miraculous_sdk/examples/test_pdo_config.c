/**
 * @file    test_pdo_config.c
 * @brief   PDO 映射配置示例 — 配置 RPDO + TPDO 映射并接收 TPDO 数据
 *
 * 演示:
 *   1. 配置 RxPDO1: 映射控制字(0x6040) + 目标位置(0x607A)
 *   2. 配置 TxPDO1: 映射状态字(0x6041) + 实际位置(0x6064) + 实际速度(0x606C)
 *   3. 注册 TPDO 回调接收从站主动上报的数据
 *
 * 配置流程 (每 PDO 6 步):
 *   禁用 → 设置传输类型 → 清空映射 → 设置映射条目 → 写映射数 → 启用
 *
 * 使用:
 *   ./example_pdo_config [can_if] [node_id]
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;
static void signal_handler(int sig) { (void)sig; g_quit = true; }

/* TPDO 数据接收回调 */
static void on_tpdo(uint8_t node_id, uint8_t pdo_num,
                    const uint8_t *data, uint8_t len,
                    void *user_data)
{
    (void)user_data;
    printf("[TPDO%u from node %u] len=%u  ", pdo_num, node_id, len);
    for (uint8_t i = 0; i < len; i++)
        printf("%02X ", data[i]);

    /* 按 TxPDO1 映射解析: 0x6041(2B) + 0x6064(4B) + 0x606C(4B) */
    if (pdo_num == 1 && len >= 10) {
        uint16_t sw  = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
        int32_t  pos = (int32_t)(data[2] | (data[3] << 8)
                               | (data[4] << 16) | (data[5] << 24));
        int32_t  vel = (int32_t)(data[6] | (data[7] << 8)
                               | (data[8] << 16) | (data[9] << 24));
        printf("| sw=0x%04X pos=%d vel=%d", sw, pos, vel);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* --- 打开电机 --- */
    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }
    if (miraculous_motor_bootstrap(motor, 3000) < 0) goto cleanup;

    /* ================================================================
     * 1. 配置 TxPDO1 (从站→主站): 0x6041 + 0x6064 + 0x606C
     *    COB-ID = 0x180 + node_id, 传输类型 254 (事件触发)
     * ================================================================ */
    uint32_t tpdo1_maps[] = {
        0x60410010,  /* 0x6041 状态字, sub0, 16位 */
        0x60640020,  /* 0x6064 实际位置, sub0, 32位 */
        0x606C0020,  /* 0x606C 实际速度, sub0, 32位 */
    };

    printf("--- Configure TxPDO1 (0x180+%d) ---\n", node_id);
    int ret = miraculous_motor_tpdo_config(motor, 1,
                                           0x180 + node_id,  /* COB-ID */
                                           254,              /* 传输类型: 事件触发 */
                                           0,                /* 禁止时间: 0=禁用 */
                                           0,                /* 事件定时器: 0=禁用 */
                                           3,                /* 3个映射对象 */
                                           tpdo1_maps);
    if (ret < 0) { fprintf(stderr, "TxPDO1 config failed\n"); goto cleanup; }

    /* ================================================================
     * 2. 配置 RxPDO1 (主站→从站): 0x6040 + 0x607A
     *    COB-ID = 0x200 + node_id, 传输类型 254 (事件触发)
     * ================================================================ */
    uint32_t rpdo1_maps[] = {
        0x60400010,  /* 0x6040 控制字, sub0, 16位 */
        0x607A0020,  /* 0x607A 目标位置, sub0, 32位 */
    };

    printf("--- Configure RxPDO1 (0x200+%d) ---\n", node_id);
    ret = miraculous_motor_rpdo_config(motor, 1,
                                       0x200 + node_id,  /* COB-ID */
                                       254,              /* 传输类型: 事件触发 */
                                       0,                /* 事件定时器: 0=禁用 */
                                       2,                /* 2个映射对象 */
                                       rpdo1_maps);
    if (ret < 0) { fprintf(stderr, "RxPDO1 config failed\n"); goto cleanup; }

    /* ================================================================
     * 3. 注册 TPDO 回调, 接收 TxPDO1 上报的数据
     * ================================================================ */
    miraculous_motor_set_tpdo_callback(motor, on_tpdo, NULL);

    /* ================================================================
     * 4. NMT 启动节点 (使能 PDO 通信)
     * ================================================================ */
    if (miraculous_motor_full_enable(motor) < 0) {
        fprintf(stderr, "full_enable failed\n");
        goto cleanup;
    }

    printf("\nMotor enabled. Waiting for TPDO data... (Ctrl+C to quit)\n\n");

    /* ================================================================
     * 5. 循环: 发 RPDO 控制字 → poll 收 TPDO
     * ================================================================ */
    while (!g_quit) {
        /* 发送 RxPDO1: 控制字=0x000F (使能运行) */
        uint8_t rpdo_data[8] = { 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        miraculous_motor_rpdo_send(motor, 1, rpdo_data, 8);

        usleep(50000);   /* 50ms */
        miraculous_motor_poll(motor, 0);
    }

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
