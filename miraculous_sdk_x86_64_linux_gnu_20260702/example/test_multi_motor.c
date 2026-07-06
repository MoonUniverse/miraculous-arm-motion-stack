/**
 * @file    test_multi_motor.c
 * @brief   多电机同步控制示例
 *
 * 两个电机同时运行 PP 模式，或通过 CSP 实现同步。
 *
 * 用法: ./example_multi_motor [can_if] [node_1] [node_2]
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
    const char *ifname  = (argc > 1) ? argv[1] : "can0";
    int node_id_1 = (argc > 2) ? atoi(argv[2]) : 1;
    int node_id_2 = (argc > 3) ? atoi(argv[3]) : 2;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* --- 分别打开两个电机 (各管理自己的 CAN 总线) --- */
    MiraMotor *motor1 = miraculous_motor_open(ifname, 0, node_id_1);
    MiraMotor *motor2 = miraculous_motor_open(ifname, 0, node_id_2);
    if (!motor1 || !motor2) {
        if (motor1) miraculous_motor_close(motor1);
        if (motor2) miraculous_motor_close(motor2);
        return -1;
    }

    /* --- Bootstrap both --- */
    printf("Bootstrapping node %d...\n", node_id_1);
    if (miraculous_motor_bootstrap(motor1, 3000) < 0) goto cleanup;
    printf("Bootstrapping node %d...\n", node_id_2);
    if (miraculous_motor_bootstrap(motor2, 3000) < 0) goto cleanup;

    /* --- Enable both --- */
    if (miraculous_motor_full_enable(motor1) < 0) goto cleanup;
    if (miraculous_motor_full_enable(motor2) < 0) goto cleanup;

    printf("Both motors enabled. Simultaneous PP moves...\n");

    /* --- Simultaneous PP moves --- */
    int32_t pos1 = 100000, pos2 = -100000;
    uint32_t vel = 50000, acc = 100000, dec = 100000;

    while (!g_quit) {
        /* 同时触发起始位置不同、方向相反的运动 */
        miraculous_motor_pp_move(motor1, pos1, vel, acc, dec,
                                  false, false);
        miraculous_motor_pp_move(motor2, pos2, vel, acc, dec,
                                  false, false);

        printf("M1 → %d, M2 → %d\n", pos1, pos2);

        /* 等待到达 */
        miraculous_motor_pp_wait_target(motor1, 5000);
        miraculous_motor_pp_wait_target(motor2, 5000);

        /* 交换方向 */
        pos1 = -pos1;
        pos2 = -pos2;
        sleep(1);
    }

    miraculous_motor_shutdown(motor1);
    miraculous_motor_shutdown(motor2);

cleanup:
    if (motor1) miraculous_motor_close(motor1);
    if (motor2) miraculous_motor_close(motor2);
    return 0;
}
