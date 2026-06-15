/**
 * @file    test_pv_mode.c
 * @brief   PV 轮廓速度模式示例
 *
 * 参照意优 SDK 的 test_pv_mode.cpp 移植
 *
 * 用法: ./example_pv_mode [can_if] [node_id]
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

    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    if (miraculous_motor_bootstrap(motor, 3000) < 0) goto cleanup;
    if (miraculous_motor_full_enable(motor) < 0) goto cleanup;

    printf("PV mode — toggling velocity direction every 5s...\n");

    int32_t  target_vel = 3000;   /* RPM */
    uint32_t acc        = 10000;   /* rpm/s */
    uint32_t dec        = 10000;  /* rpm/s */
    CiaProfileType_t profile = CIA_PROFILE_SCURVE;

    while (!g_quit) {
        /* 换向安全策略: 先减速到 0, 再切到目标速度 */
        if (target_vel != 0) {
            miraculous_motor_pv_move(motor, 0, acc, dec, profile);
            usleep(800000); /* 等待电机减速到 0 (6000/10000=0.6s) */
        }

        int ret = miraculous_motor_pv_move(motor, target_vel, acc, dec, profile);
        if (ret < 0) {
            fprintf(stderr, "PV move failed: %s\n", mrc_strerror(ret));
            break;
        }

        printf("Running at %d RPM...\n", target_vel);

        /* 每秒读取一次实际速度 */
        for (int i = 0; i < 5 && !g_quit; i++) {
            sleep(1);
            int32_t vel;
            miraculous_motor_get_velocity(motor, &vel);
            printf("  Actual vel = %d RPM\n", vel);
        }

        target_vel = -target_vel; /* 反转方向 */
    }

    /* Ctrl+C 退出: 先减速到 0 再关机 */
    printf("Decelerating to stop...\n");
    miraculous_motor_pv_move(motor, 0, acc, dec, profile);
    sleep(1);

    miraculous_motor_shutdown(motor);
cleanup:
    miraculous_motor_close(motor);
    return 0;
}
