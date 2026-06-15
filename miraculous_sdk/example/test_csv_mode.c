/**
 * @file    test_csv_mode.c
 * @brief   CSV 周期同步速度模式示例
 *
 * CSV 模式中，上位机在每个 SYNC 周期发送目标速度。
 * 演示初始化 CSV 并周期性设置速度。
 *
 * 用法: ./example_csv_mode [can_if] [node_id]
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

    /* --- 初始化 CSV (SYNC 周期 4ms) --- */
    uint32_t sync_period_us = 4000;
    if (miraculous_motor_csv_init(motor, sync_period_us) < 0) {
        fprintf(stderr, "CSV init failed\n");
        goto cleanup;
    }

    printf("CSV mode — sending velocity trajectory each %u us...\n",
           sync_period_us);

    int32_t target_vel = 50000;   /* RPM */
    int32_t step = 1000;          /* 每周期速度增量 */

    while (!g_quit) {
        int ret = miraculous_motor_csv_set_target(motor, target_vel);
        if (ret < 0) {
            fprintf(stderr, "CSV set target failed: %s\n", mrc_strerror(ret));
            break;
        }

        target_vel += step;

        /* 用 usleep 近似周期（生产环境用 timerfd） */
        usleep(sync_period_us);
    }

    miraculous_motor_shutdown(motor);
    /* sync_stop 由 motor_close 自动处理 */

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
