/**
 * @file    test_csp_ex.c
 * @brief   CSP 周期同步位置模式示例 — 使用弧度单位
 *
 * CSP 模式中，上位机在每个 SYNC 周期发送目标位置 + SYNC 帧。
 * 使用 miraculous_motor_csp_set_target_ex() 以弧度设置目标位置，
 * 并使用 miraculous_motor_get_position_ex() 以弧度读取实际位置。
 *
 * 用法: ./example_csp_ex [can_if] [node_id] [sync_mode]
 *   sync_mode: "timer" (默认, 10ms SYNC定时器) 或 "manual" (手动发SYNC)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

/* 到位容差 (弧度) */
#define POSITION_TOLERANCE_RAD  0.02f

/* 每组内移动次数 */
#define MOVES_PER_BATCH     10

/* 测试组数（负数表示无限循环，按 Ctrl+C 退出） */
#define BATCH_COUNT         4

/* 每步步进弧度 */
#define STEP_SIZE_RAD       0.1f

/* 每次下发后等待电机到位的时间 (us) */
#define SETTLE_TIME_US     10000 /* 10 ms */

/* 速度判零阈值 (脉冲/s) */
#define VELOCITY_ZERO_THRESHOLD  10

static volatile bool g_quit = false;

static void signal_handler(int sig)
{
    (void)sig;
    g_quit = true;
}

/* 等待电机速度降至零 */
static void wait_velocity_zero(MiraMotor *motor, int timeout_ms)
{
    int32_t vel;
    for (int i = 0; i < timeout_ms / 10; i++) {
        if (g_quit) break;
        if (miraculous_motor_get_velocity(motor, &vel) == 0) {
            if (vel < 0) vel = -vel;
            if (vel <= VELOCITY_ZERO_THRESHOLD) return;
        }
        usleep(10000);  /* 10ms 轮询一次 */
    }
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;
    bool manual_sync = (argc > 3 && strcmp(argv[3], "manual") == 0);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* --- Init: 打开 CAN 接口 --- */
    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    /* NMT Start */
    if (miraculous_motor_bootstrap(motor, 3000) < 0) goto cleanup;

    /* 设模式 → 使能 → init */
    if (miraculous_motor_set_mode(motor, CIA_MODE_CSP) < 0) goto cleanup;
    if (miraculous_motor_full_enable(motor) < 0) goto cleanup;
    uint32_t sync_period_us = SETTLE_TIME_US;
    if (miraculous_motor_csp_init(motor, sync_period_us, manual_sync) < 0) {
        fprintf(stderr, "CSP init failed\n");
        goto cleanup;
    }

    printf("CSP mode (radian) — SYNC: %s\n", manual_sync ? "MANUAL" : "TIMER 10000 us");
    printf("Settle time = %d us (%d ms)\n", SETTLE_TIME_US, SETTLE_TIME_US / 1000);
    printf("Position tolerance = ±%.3f rad\n\n", POSITION_TOLERANCE_RAD);

    /* --- 读取初始位置 --- */
    float start_rad;
    if (miraculous_motor_get_position_ex(motor, &start_rad, POS_UNIT_RADIAN) < 0) {
        fprintf(stderr, "Failed to read start position\n");
        goto cleanup;
    }
    printf("Start position: %.4f rad\n\n", start_rad);


    printf("---------------------------------------------------------------\n");
    printf(" # |  Target(rad)  |  Actual(rad)  | Delta(rad)  | Status\n");
    printf("---------------------------------------------------------------\n");

    printf("Step size = %.2f rad, %d moves per batch, %d batches\n\n",
           STEP_SIZE_RAD, MOVES_PER_BATCH, BATCH_COUNT);

    int passed = 0;
    int failed = 0;
    int move_no = 0;
    float direction = 1.0f;        /* 1.0: 正向, -1.0: 反向 */
    float pos_rad;

    for (int batch = 0; batch < BATCH_COUNT; batch++) {
        if (g_quit) break;

        printf("--- Batch %d (%s step) ---\n",
               batch + 1, (direction > 0) ? "正向" : "反向");

        /* 换向时等待速度降为零，再发下一批目标 */
        if (batch > 0) {
            wait_velocity_zero(motor, 2000);
        }

        /* 批次开始：读取当前位置作为基准 */
        usleep(200);
        miraculous_motor_get_position_ex(motor, &pos_rad, POS_UNIT_RADIAN);
        float batch_base = pos_rad;

        for (int m = 0; m < MOVES_PER_BATCH; m++) {
            if (g_quit) break;

            move_no++;
            float target_rad = batch_base + direction * STEP_SIZE_RAD * (float)(m + 1);

            /* 以弧度发送目标位置 */
            int ret = miraculous_motor_csp_set_target_ex(motor, target_rad, POS_UNIT_RADIAN);
            if (ret < 0) {
                fprintf(stderr, "CSP set target #%d failed: %s\n",
                        move_no, mrc_strerror(ret));
                failed++;
                continue;
            }

            /* 发送 SYNC 帧, 通知从站锁存目标位置 */
            miraculous_motor_sync_send(motor);

            usleep(SETTLE_TIME_US);

            /* 读取实际位置 (弧度) */
            if (miraculous_motor_get_position_ex(motor, &pos_rad, POS_UNIT_RADIAN) < 0) {
                fprintf(stderr, "Failed to read position after move #%d\n", move_no);
                failed++;
                continue;
            }

            /* 计算差值 */
            float delta = target_rad - pos_rad;

            const char *status;
            if (delta < 0) delta = -delta;
            if (delta <= POSITION_TOLERANCE_RAD) {
                status = "PASS";
                passed++;
            } else {
                status = "FAIL";
                failed++;
            }

            printf(" %3d |   %.4f      |   %.4f      |  %+.4f    | %s\n",
                   move_no, target_rad, pos_rad, target_rad - pos_rad, status);
        }

        /* 批次结束，等待 3s 后打印最终位置并与最后目标比较 */
        float last_target_rad = batch_base + direction * STEP_SIZE_RAD * (float)MOVES_PER_BATCH;
        usleep(3000000);
        float final_rad;
        if (miraculous_motor_get_position_ex(motor, &final_rad, POS_UNIT_RADIAN) == 0) {
            float delta = last_target_rad - final_rad;
            if (delta < 0) delta = -delta;
            printf("  Batch %d final: %.4f rad, target: %.4f rad, delta: %.4f\n",
                   batch + 1, final_rad, last_target_rad, delta);
        }

        /* 每组完成后方向取反 */
        direction = -direction;
        printf("\n");
    }

    printf("---------------------------------------------------------------\n\n");

    /* --- 汇总 --- */
    printf("=== Summary ===\n");
    printf("  Total moves : %d\n", passed + failed);
    printf("  Passed      : %d\n", passed);
    printf("  Failed      : %d\n", failed);
    printf("  Tolerance   : ±%.3f rad\n", POSITION_TOLERANCE_RAD);

    if (failed == 0) {
        printf("\n  All moves within tolerance.\n");
    } else {
        printf("\n  %d move(s) exceeded tolerance.\n", failed);
    }

    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return (failed > 0) ? 1 : 0;
}
