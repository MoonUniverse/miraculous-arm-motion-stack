/**
 * @file    test_csp_ex.c
 * @brief   CSP 周期同步位置模式示例 — 使用弧度单位
 *
 * 演示:
 *   1. miraculous_motor_csp_set_target_ex() — 用弧度设置目标位置
 *   2. miraculous_motor_get_position_ex()   — 读取弧度位置
 *   3. miraculous_motor_get_position()      — 读取原始脉冲
 *
 * 使用:
 *   1. 连接 CAN 总线: sudo ip link set can0 type can bitrate 1000000
 *      sudo ip link set up can0
 *   2. 运行: ./example_csp_ex
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;

static void signal_handler(int sig)
{
    (void)sig;
    g_quit = true;
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

    /* --- Bootstrap + 使能 --- */
    int ret = miraculous_motor_bootstrap(motor, 3000);
    if (ret < 0) { fprintf(stderr, "Bootstrap failed: %s\n", mrc_strerror(ret)); goto cleanup; }

    ret = miraculous_motor_full_enable(motor);
    if (ret < 0) { fprintf(stderr, "Enable failed: %s\n", mrc_strerror(ret)); goto cleanup; }

    /* --- CSP 初始化 (10ms SYNC) --- */
    ret = miraculous_motor_csp_init(motor, 10000, false);
    if (ret < 0) { fprintf(stderr, "CSP init failed: %s\n", mrc_strerror(ret)); goto cleanup; }

    printf("CSP started. Moving with radian positions...\n\n");

    /* --- 打印初始位置 --- */
    {
        float rad;
        miraculous_motor_get_position_ex(motor, &rad, POS_UNIT_RADIAN);
        printf("Initial position: %.4f rad\n\n", rad);
    }

    /* --- 运动循环: 步进 0.5 rad --- */
    float pos_rad = 0.0f;
    const float step_rad = 0.5f;

    while (!g_quit) {
        pos_rad += step_rad;

        /* 用弧度设置目标位置 */
        ret = miraculous_motor_csp_set_target_ex(motor, pos_rad, POS_UNIT_RADIAN);
        if (ret < 0) {
            fprintf(stderr, "CSP set target failed: %s\n", mrc_strerror(ret));
            break;
        }

        /* 等待 ~1s */
        for (int i = 0; i < 100 && !g_quit; i++) {
            int32_t pulses;
            float rad;

            miraculous_motor_get_position_ex(motor, &rad, POS_UNIT_RADIAN);
            miraculous_motor_get_position(motor, &pulses);

            printf("\rtarget=%.3f rad  actual=%.3f rad  pulses=%d  ",
                   pos_rad, rad, pulses);
            fflush(stdout);

            miraculous_motor_poll(motor, 10);
        }
        printf("\n");
    }

    /* --- 停机 --- */
    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
