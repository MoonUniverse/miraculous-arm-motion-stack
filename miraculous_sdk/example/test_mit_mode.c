/**
 * @file    test_mit_mode.c
 * @brief   MIT 力位混合模式示例
 *
 * MIT (阻抗控制) 模式下，电机根据刚度/阻尼参数和位置误差输出转矩:
 *   目标转矩 = KP * (目标位置 - 实际位置) - KD * 实际速度
 *
 * 本示例演示:
 *   1. 初始化 MIT 模式 (PDS 使能 + MIT 模式设置)
 *   2. 配置刚度 KP 和阻尼 KD 参数
 *   3. 设置力矩限制
 *   4. 在每个 SYNC 周期发送目标位置
 *
 * 用法: ./example_mit_mode [can_if] [node_id]
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

    if (miraculous_motor_bootstrap(motor, 3000) < 0) {
        fprintf(stderr, "Bootstrap failed\n");
        goto cleanup;
    }
    if (miraculous_motor_full_enable(motor) < 0) {
        fprintf(stderr, "Enable failed\n");
        goto cleanup;
    }

    /* --- 配置 MIT 参数 (通过 SDO 设置，需在 Init 之前完成) --- */

    /* 刚度系数 Kp [A/rad]: 位置误差 1 rad 产生的附加转矩 */
    float stiffness_kp = 50.0f;
    int ret = miraculous_motor_mit_set_stiffness(motor, stiffness_kp);
    if (ret < 0) {
        fprintf(stderr, "Set stiffness failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }
    printf("MIT stiffness Kp = %.1f A/rad\n", stiffness_kp);

    /* 阻尼系数 Kd [A*s/rad]: 速度 1 rad/s 产生的阻抗转矩 */
    float damping_kd = 5.0f;
    ret = miraculous_motor_mit_set_damping(motor, damping_kd);
    if (ret < 0) {
        fprintf(stderr, "Set damping failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }
    printf("MIT damping Kd = %.1f A*s/rad\n", damping_kd);

    /* 力矩限制: 千分之一额定转矩, 300 = 30% */
    int16_t torque_limit = 300;
    ret = miraculous_motor_mit_set_torque_limit(motor, torque_limit);
    if (ret < 0) {
        fprintf(stderr, "Set torque limit failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }
    printf("MIT torque limit = %d (0.1%% rated)\n", torque_limit);

    /* --- 初始化 MIT 模式 (SYNC 周期 4ms) --- */
    uint32_t sync_period_us = 4000;
    ret = miraculous_motor_mit_init(motor, sync_period_us);
    if (ret < 0) {
        fprintf(stderr, "MIT init failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }

    printf("MIT mode started — sending target position each %u us...\n",
           sync_period_us);
    printf("Press Ctrl+C to stop\n");

    /* --- MIT 轨迹循环 --- */
    /* 阻抗控制: 上位机发送目标位置, 固件根据 Kp/Kd 计算输出转矩 */
    int32_t start_pos;
    miraculous_motor_get_position(motor, &start_pos);
    int32_t pos  = start_pos;
    int32_t step = 100;   /* 每周期步进脉冲数 */
    int     tick = 0;

    while (!g_quit) {
        /* 发送目标位置 (固件通过 Kp/Kd 计算阻抗转矩) */
        ret = miraculous_motor_mit_set_target(motor, pos);
        if (ret < 0) {
            fprintf(stderr, "MIT set target failed: %s\n", mrc_strerror(ret));
            break;
        }

        pos += step;
        tick++;

        /* 每 100 周期打印一次当前状态 */
        if (tick % 100 == 0) {
            int32_t actual_pos;
            int16_t actual_torque;
            if (miraculous_motor_get_position(motor, &actual_pos) == 0 &&
                miraculous_motor_get_torque(motor, &actual_torque) == 0) {
                printf("[MIT] target=%d  actual=%d  torque=%d\n",
                       pos, actual_pos, actual_torque);
            }
        }

        usleep(sync_period_us);
    }

    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
