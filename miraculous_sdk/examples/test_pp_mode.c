/**
 * @file    test_pp_mode.c
 * @brief   PP 轮廓位置模式示例
 *
 * 参照意优 SDK 的 test_csp_mode.cpp 移植
 *
 * 使用:
 *   1. 启动 vcan (测试): sudo ip link add dev vcan0 type vcan
 *      sudo ip link set up vcan0
 *   2. 或连接真实 CAN 总线: sudo ip link set can0 type can bitrate 1000000
 *      sudo ip link set up can0
 *   3. 运行: ./example_pp_mode
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

    /* --- 4. Bootstrap (NMT Reset → 等 Operational) --- */
    int ret = miraculous_motor_bootstrap(motor, 3000);
    if (ret < 0) {
        fprintf(stderr, "Bootstrap failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }

    /* --- 5. 使能电机 (Shutdown → Switch On → Enable Operation) --- */
    ret = miraculous_motor_full_enable(motor);
    if (ret < 0) {
        fprintf(stderr, "Enable failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }

    printf("Motor enabled. Starting PP moves...\n");

    /* --- 6. PP 运动循环 --- */
    int32_t pos      = 0;
    int32_t step     = 10000;    /* 脉冲步长 */
    uint32_t vel    = 50000;     /* 脉冲/s */
    uint32_t acc    = 100000;    /* 脉冲/s² */
    uint32_t dec    = 100000;

    while (!g_quit) {
        /* 绝对位置模式，每步走 step 个脉冲 */
        ret = miraculous_motor_pp_move(motor, pos, vel, acc, dec,
                                        false,   /* absolute */
                                        false);  /* 不立即更新 */
        if (ret < 0) {
            fprintf(stderr, "PP move failed: %s\n", mrc_strerror(ret));
            break;
        }

        printf("Moving to pos = %d\n", pos);

        /* 等待到达 (带超时 5s) */
        ret = miraculous_motor_pp_wait_target(motor, 5000);
        if (ret < 0) {
            fprintf(stderr, "Target wait failed: %s\n", mrc_strerror(ret));
            break;
        }

        /* 读取实际位置 */
        int32_t actual;
        miraculous_motor_get_position(motor, &actual);
        printf("  Actual pos = %d\n", actual);

        pos += step;
        sleep(1);
    }

    /* --- 7. 停机 --- */
    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
