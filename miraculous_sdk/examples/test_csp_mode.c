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
#define MOVES_PER_BATCH     20

/* 测试组数（负数表示无限循环，按 Ctrl+C 退出） */
#define BATCH_COUNT         2

/* 每步的步进脉冲数 */
#define STEP_SIZE           (524288*4/10/360) // 0.4 度

/* 每次下发后等待电机到位的时间 (us) */
#define SETTLE_TIME_US     15000 /* 10 ms */

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
        miraculous_motor_sync_send(motor);  /* 每次检查前发 SYNC 获取最新 TPDO */
        usleep(3000);
        if (miraculous_motor_get_velocity(motor, &vel) == 0) {
            if (vel < 0) vel = -vel;
            if (vel <= VELOCITY_ZERO_THRESHOLD) return;
        }
        usleep(7000);
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
    if (miraculous_motor_full_enable(motor) < 0) {
        uint16_t sw = 0;
        uint16_t err = 0;
        uint8_t len = 2;
        miraculous_motor_get_statusword(motor, &sw);
        fprintf(stderr, "full_enable failed, statusword=0x%04X", sw);
        if (miraculous_motor_sdo_read(motor, CIA402_OD_ERROR_CODE, 0, &err, &len) == 0)
            fprintf(stderr, " error_code=0x%04X", err);
        fprintf(stderr, "\n");
        goto cleanup;
    }

    uint32_t sync_period_us = SETTLE_TIME_US;
    if (miraculous_motor_csp_init(motor, sync_period_us, manual_sync) < 0) {
        fprintf(stderr, "CSP init failed\n");
        goto shutdown;
    }

    printf("CSP mode — SYNC: %s\n", manual_sync ? "MANUAL" : "TIMER 5000 us");
    printf("Settle time = %d us (%d ms)\n", SETTLE_TIME_US, SETTLE_TIME_US / 1000);
    printf("Position tolerance = ±%d inc\n\n", POSITION_TOLERANCE);

    /* --- 发送 SYNC 触发 TPDO, 然后读取初始位置 --- */
    miraculous_motor_sync_send(motor);
    usleep(1000);
    int32_t start_pos;
    if (miraculous_motor_get_position(motor, &start_pos) < 0) {
        /* TPDO 未到, 用 SDO 读一次 */
        uint8_t len = 4;
        if (miraculous_motor_sdo_read(motor, CIA402_OD_ACTUAL_POSITION, 0,
                                       &start_pos, &len) < 0) {
            fprintf(stderr, "Failed to read start position\n");
            goto shutdown;
        }
    }
    printf("Start position: 0x%08X (%d)\n\n", start_pos, start_pos);


    printf("---------------------------------------------------------------\n");
    printf(" # |  Target Pos   |  Actual Pos   |  Delta(inc)  | Status\n");
    printf("---------------------------------------------------------------\n");

    printf("Step size = 0x%X (%d), %d moves per batch, %d batches\n\n",
           STEP_SIZE, STEP_SIZE, MOVES_PER_BATCH, BATCH_COUNT);

    int passed = 0;
    int failed = 0;
    int move_no = 0;
    int move_display = 0;   /* 显示序号 */
    int32_t direction = 1;           /* 1: 正向, -1: 反向 */
    int32_t pos = 0;
    int32_t batch_base = 0;

    for (int batch = 0; batch < BATCH_COUNT; batch++) {
        if (g_quit) break;

        printf("--- Batch %d (%s step) ---\n",
               batch + 1, (direction > 0) ? "正向" : "反向");

        /* 换向时等待速度降为零，再发下一批目标 */
        if (batch > 0) {
            wait_velocity_zero(motor, 20000);
        }

        /* 批次开始：读取当前位置作为基准 */
        usleep(200);
        if (miraculous_motor_get_position(motor, &pos) < 0) {
            pos = batch_base;  /* TPDO 未到，用上一批的基准 */
        }
        batch_base = pos;

        for (int m = 0; m <= MOVES_PER_BATCH; m++) {
            if (g_quit) break;

            move_no++;

            /* 最后一次不发目标 */
            if (m < MOVES_PER_BATCH) {
                int32_t target = batch_base + direction * STEP_SIZE * (m + 1);

                int ret = miraculous_motor_csp_set_target(motor, target);
                if (ret < 0) {
                    fprintf(stderr, "CSP set target #%d failed: %s\n",
                            move_no, mrc_strerror(ret));
                    failed++;
                    continue;
                }
            }

            /* 发送 SYNC 帧, 通知从站锁存目标位置 */
            miraculous_motor_sync_send(motor);

            usleep(SETTLE_TIME_US);

            /* 读取实际位置 (本次 SYNC 的位置是上一目标到位后的结果) */
            if (miraculous_motor_get_position(motor, &pos) < 0) {
                fprintf(stderr, "Failed to read position after move #%d\n", move_no);
                failed++;
                continue;
            }

            /* 第一次不显示 */
            if (m == 0) {
                continue;
            }

            move_display++;
            /* 用本次位置与上一个目标比较 */
            int32_t prev_target = batch_base + direction * STEP_SIZE * m;
            int32_t delta = prev_target - pos;
            int32_t total_move = STEP_SIZE;

            const char *status;
            if (abs(delta) <= POSITION_TOLERANCE) {
                status = "PASS";
                passed++;
            } else {
                status = "FAIL";
                failed++;
            }

            printf(" %3d | 0x%08X | 0x%08X | %+6d (%d%%) | %s\n",
                   move_display, prev_target, pos, delta,
                   (total_move > 0) ? (int)((double)abs(delta) / total_move * 100) : 0,
                   status);
        }

        /* 批次结束，发 SYNC 获取最终位置并与最后目标比较 */
        int32_t last_target = batch_base + direction * STEP_SIZE * MOVES_PER_BATCH;
        sleep(3);
        miraculous_motor_sync_send(motor);
        usleep(SETTLE_TIME_US);
        int32_t final_pos;
        if (miraculous_motor_get_position(motor, &final_pos) == 0) {
            int32_t delta = last_target - final_pos;
            if (delta < 0) delta = -delta;
            const char *status = (abs(delta) <= POSITION_TOLERANCE) ? "PASS" : "FAIL";
            printf(" F%-2d | 0x%08X | 0x%08X | %+6d (%d%%) | %s\n",
                   batch + 1, last_target, final_pos, delta,
                   (int)((double)abs(delta) / STEP_SIZE * 100), status);
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

shutdown:
    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return (failed > 0) ? 1 : 0;
}
