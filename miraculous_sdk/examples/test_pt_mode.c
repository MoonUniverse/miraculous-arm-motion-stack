/**
 * @file    test_pt_mode.c
 * @brief   PT 轮廓转矩模式示例
 *
 * 演示轮廓转矩模式 (Profile Torque) 的使用。
 * 设置目标转矩和斜率，循环切换正反转转矩。
 *
 * 用法: ./example_pt_mode [can_if] [node_id]
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

    /* --- Init --- */
    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    if (miraculous_motor_bootstrap(motor, 3000) < 0) goto cleanup;
    if (miraculous_motor_full_enable(motor) < 0) goto cleanup;

    printf("PT mode — toggling torque direction every 5s...\n");

    /* --- 设置操作模式为 PT --- */
    if (miraculous_motor_set_mode(motor, CIA_MODE_PT) < 0) {
        fprintf(stderr, "Set PT mode failed\n");
        goto cleanup;
    }

    int16_t target_torque = 100;  /* 0.01A 单位，1A*/
    uint16_t slope = 3;         /* 3A/s */

    while (!g_quit) {
        int ret = miraculous_motor_pt_move(motor, target_torque, slope);
        if (ret < 0) {
            fprintf(stderr, "PT move failed: %s\n", mrc_strerror(ret));
            break;
        }

        printf("Running torque = %d (0.01A)\n", target_torque);

        /* 运行 5 秒 */
        for (int i = 0; i < 5 && !g_quit; i++) {
            sleep(1);
        }

        target_torque = -target_torque; /* 反转转矩方向 */
    }

    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
