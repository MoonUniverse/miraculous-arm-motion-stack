/**
 * @file    test_csp_mode.c
 * @brief   CSP 周期同步位置模式示例 — 带位置到位校验
 *
 * CSP 模式中，上位机在每个 SYNC 周期发送目标位置 + SYNC 帧。
 * 下发后等待电机到位，读取实际位置并与目标位置比较，
 * 判断是否在 200 inc 容差范围内。
 *
 * 用法: ./example_csp_mode [can_if] [node_id] [sync_mode]
 *   sync_mode: "timer" (默认, 5ms SYNC定时器) 或 "manual" (手动发SYNC)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

/* 到位容差 (inc) */
#define POSITION_TOLERANCE  100

/* 每组内移动次数 */
#define MOVES_PER_BATCH     5

/* 测试组数（负数表示无限循环，按 Ctrl+C 退出） */
#define BATCH_COUNT         4

/* 每步的步进脉冲数 */
#define STEP_SIZE           1093

/* 每次下发后等待电机到位的时间 (us) */
#define SETTLE_TIME_US      500000 /* 5 ms (1× SYNC 周期) */

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
    bool manual_sync = (argc > 3 && strcmp(argv[3], "manual") == 0);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* --- Init: 打开 CAN 接口，状态机由 csp_init 通过 PDO 控制 --- */
    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    /* NMT Reset Node → 等待 → NMT Start，确保从机从已知状态启动 */
    {
        MiraCanCtx *can = miraculous_motor_get_can_ctx(motor);
        uint8_t reset_data[2] = {0x81, (uint8_t)node_id};
        int reset_ret = miraculous_can_send(can, 0x000, reset_data, 2);
        printf("[test] NMT Reset Node -> %s\n", (reset_ret < 0) ? "FAIL" : "OK");
        usleep(100000);  /* 等 100ms 让从机完成复位 */

        uint8_t start_data[2] = {0x01, (uint8_t)node_id};
        int nmt_ret = miraculous_can_send(can, 0x000, start_data, 2);
        printf("[test] NMT Start -> %s\n", (nmt_ret < 0) ? "FAIL" : "OK");
        usleep(50000);
    }

    /* --- 初始化 CSP --- */
    uint32_t sync_period_us = manual_sync ? 0 : 5000; /* 0=手动SYNC, 5000=定时器5ms */
    if (miraculous_motor_csp_init(motor, sync_period_us) < 0) {
        fprintf(stderr, "CSP init failed\n");
        goto cleanup;
    }

    printf("CSP mode — SYNC: %s\n", manual_sync ? "MANUAL" : "TIMER 5000 us");
    printf("Settle time = %d us (%d ms)\n", SETTLE_TIME_US, SETTLE_TIME_US / 1000);
    printf("Position tolerance = ±%d inc\n\n", POSITION_TOLERANCE);

    /* --- 读取初始位置 --- */
    int32_t start_pos;
    if (miraculous_motor_get_position(motor, &start_pos) < 0) {
        fprintf(stderr, "Failed to read start position\n");
        goto cleanup;
    }
    printf("Start position: 0x%08X (%d)\n\n", start_pos, start_pos);

    /* CSP 启动前先发一次当前位置，初始化 CspPrevTarget_inc + CspReceived */
    printf("[test] Sending initial position to seed CSP controller...\n");
    int seed_ret = miraculous_motor_csp_set_target(motor, start_pos);
    if (seed_ret < 0) {
        fprintf(stderr, "CSP seed target failed\n");
    }


    printf("---------------------------------------------------------------\n");
    printf(" # |  Target Pos   |  Actual Pos   |  Delta(inc)  | Status\n");
    printf("---------------------------------------------------------------\n");

    printf("Step size = 0x%X (%d), %d moves per batch, %d batches\n\n",
           STEP_SIZE, STEP_SIZE, MOVES_PER_BATCH, BATCH_COUNT);

    int passed = 0;
    int failed = 0;
    int move_no = 0;
    int32_t direction = 1;           /* 1: 正向, -1: 反向 */
    int32_t pos = start_pos;

    for (int batch = 0; batch < BATCH_COUNT; batch++) {
        if (g_quit) break;

        printf("--- Batch %d (%s step) ---\n",
               batch + 1, (direction > 0) ? "正向" : "反向");

        for (int m = 0; m < MOVES_PER_BATCH; m++) {
            if (g_quit) break;

            move_no++;
            int32_t prev_pos = pos;
            int32_t target   = pos + direction * STEP_SIZE;

            /* 发送目标位置（首次） */
            int ret = miraculous_motor_csp_set_target(motor, target);
            if (ret < 0) {
                fprintf(stderr, "CSP set target #%d failed: %s\n",
                        move_no, mrc_strerror(ret));
                failed++;
                continue;
            }

            
            usleep(SETTLE_TIME_US);
              

            /* 读取实际位置 */
            if (miraculous_motor_get_position(motor, &pos) < 0) {
                fprintf(stderr, "Failed to read position after move #%d\n", move_no);
                failed++;
                continue;
            }

            /* 计算差值 */
            int32_t delta = abs(pos - target);
            int32_t total_move = abs(target - prev_pos);

            const char *status;
            if (delta <= POSITION_TOLERANCE) {
                status = "PASS";
                passed++;
            } else {
                status = "FAIL";
                failed++;
            }

            printf(" %3d | 0x%08X | 0x%08X | %+6d (%d%%) | %s\n",
                   move_no,
                   target, pos,
                   pos - target,
                   (total_move > 0) ? (int)((double)delta / total_move * 100) : 0,
                   status);
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
    printf("  Tolerance   : ±%d inc\n", POSITION_TOLERANCE);

    if (failed == 0) {
        printf("\n  ✓ All moves within tolerance.\n");
        printf("  Position loop tuning is acceptable.\n");
    } else {
        printf("\n  ✗ %d move(s) exceeded tolerance.\n", failed);
        printf("  Consider reducing position loop Ki (0x2004 sub3/sub4)\n");
        printf("  or increasing Kp (0x2004 sub1/sub2).\n");
        printf("  See example_read_params for current PI values.\n");
    }

    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return (failed > 0) ? 1 : 0;
}
