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

    /* 设模式 → 使能 → init */
    if (miraculous_motor_set_mode(motor, CIA_MODE_CSV) < 0) goto cleanup;
    if (miraculous_motor_full_enable(motor) < 0) goto cleanup;

    uint32_t sync_period_us = 1000;
    if (miraculous_motor_csv_init(motor, sync_period_us) < 0) {
        fprintf(stderr, "CSV init failed\n");
        goto cleanup;
    }

    printf("CSV mode — sending velocity trajectory each %u us...\n",
           sync_period_us);

    int32_t target_vel = 0;
    int32_t step = 1000;         /* 每周期速度增量 */
    int32_t direction = 1;       /* 1: 加速, -1: 减速 */
    int max_cycles = 1000;       /* 1s / 1ms = 1000 个周期 */

    while (!g_quit && max_cycles-- > 0) {
        int ret = miraculous_motor_csv_set_target(motor, target_vel);
        if (ret < 0) {
            fprintf(stderr, "CSV set target failed: %s\n", mrc_strerror(ret));
            break;
        }

        /* poll 处理 timerfd，发送 SYNC 帧 */
        miraculous_motor_poll(motor, 0);

        /* 等待一个 SYNC 周期 */
        usleep(sync_period_us);

        /* 读取实际速度并打印（检查返回值） */
        int32_t actual_vel = 0;
        ret = miraculous_motor_get_velocity(motor, &actual_vel);
        if (ret < 0) {
            printf("[csv] target=%5d rpm  actual= (SDO error %d)\n", target_vel, ret);
        } else {
            printf("[csv] target=%5d rpm  actual=%5d rpm\n", target_vel, actual_vel);
        }

        /* 判断是否到达限速并反转方向 */
        if (direction > 0) {
            /* 加速阶段：达到或超过 3000 rpm 则反转 */
            if (actual_vel >= 3000 || target_vel >= 3000) {
                direction = -1;
                printf("[csv] --- reached limit, reversing ---\n");
            }
        } else {
            /* 减速阶段：回到 0 rpm 左右则重新加速 */
            if (actual_vel <= 0 || target_vel <= 0) {
                direction = 1;
                printf("[csv] --- returned to zero, accelerating ---\n");
            }
        }

        target_vel += step * direction;
    }

    miraculous_motor_shutdown(motor);
    /* sync_stop 由 motor_close 自动处理 */

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
