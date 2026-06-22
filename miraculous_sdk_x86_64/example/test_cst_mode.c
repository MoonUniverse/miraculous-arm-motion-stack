/**
 * @file    test_cst_mode.c
 * @brief   CST 周期同步转矩模式示例
 *
 * CST 模式中，上位机在每个 SYNC 周期发送目标转矩。
 * 演示初始化 CST 并周期性设置转矩。
 *
 * 用法: ./example_cst_mode [can_if] [node_id]
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

    /* 设模式 → 使能 → init */
    if (miraculous_motor_set_mode(motor, CIA_MODE_CST) < 0) goto cleanup;
    if (miraculous_motor_full_enable(motor) < 0) goto cleanup;

    uint32_t sync_period_us = 4000;
    if (miraculous_motor_cst_init(motor, sync_period_us) < 0) {
        fprintf(stderr, "CST init failed\n");
        goto cleanup;
    }

    printf("CST mode — sending torque target each %u us...\n",
           sync_period_us);

    int16_t target_torque = 500;  /* 0.01A */
    int16_t step = 10;

    while (!g_quit) {
        int ret = miraculous_motor_cst_set_target(motor, target_torque);
        if (ret < 0) {
            fprintf(stderr, "CST set target failed: %s\n", mrc_strerror(ret));
            break;
        }

        target_torque += step;

        usleep(sync_period_us);
    }

    miraculous_motor_shutdown(motor);
    /* sync_stop 由 motor_close 自动处理 */

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
