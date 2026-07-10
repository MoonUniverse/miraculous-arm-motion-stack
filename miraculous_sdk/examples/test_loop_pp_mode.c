/**
 * @file    test_loop_pp_mode.c
 * @brief   循环 PP 轮廓位置模式示例
 *
 * 在两个目标位置间来回运动，演示绝对位置与相对位置的用法。
 *
 * 用法: ./example_loop_pp_mode [can_if] [node_id]
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

    printf("Loop PP mode — moving between two positions...\n");

    /* --- 运动参数 --- */
    int32_t pos_a = 100000;       /* 第一个目标位置 */
    int32_t pos_b = -50000;       /* 第二个目标位置 */
    uint32_t vel = 50000;         /* 脉冲/s */
    uint32_t acc = 100000;        /* 脉冲/s² */
    uint32_t dec = 100000;

    int use_relative = 1;         /* 0=绝对位置, 1=相对位置 */

    while (!g_quit) {
        /* 目标 A */
        int ret = miraculous_motor_pp_move(motor, use_relative ? pos_a - pos_a : pos_a,
                                            vel, acc, dec,
                                            use_relative ? true : false,
                                            false);
        if (ret < 0) {
            fprintf(stderr, "PP move A failed: %s\n", mrc_strerror(ret));
            break;
        }
        printf("Moving to pos_a = %d\n", pos_a);
        if (miraculous_motor_pp_wait_target(motor, 5000) < 0) break;

        /* 读取实际位置 */
        int32_t actual;
        miraculous_motor_get_position(motor, &actual);
        printf("  Actual pos = %d\n", actual);

        /* 目标 B */
        ret = miraculous_motor_pp_move(motor, use_relative ? pos_b - pos_a : pos_b,
                                        vel, acc, dec,
                                        use_relative ? true : false,
                                        false);
        if (ret < 0) {
            fprintf(stderr, "PP move B failed: %s\n", mrc_strerror(ret));
            break;
        }
        printf("Moving to pos_b = %d\n", pos_b);
        if (miraculous_motor_pp_wait_target(motor, 5000) < 0) break;

        miraculous_motor_get_position(motor, &actual);
        printf("  Actual pos = %d\n", actual);

        /* 交换位置值，循环 */
        int32_t tmp = pos_a;
        pos_a = pos_b;
        pos_b = tmp;

        sleep(1);
    }

    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
