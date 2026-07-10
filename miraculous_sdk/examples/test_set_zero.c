/**
 * @file    test_set_zero.c
 * @brief   设置零点示例
 *
 * 将当前位置设为零点。如需掉电保持,
 * 请再调用 miraculous_motor_save_config()。
 *
 * 使用:
 *   1. 启动 vcan (测试): sudo ip link add dev vcan0 type vcan
 *      sudo ip link set up vcan0
 *   2. 或连接真实 CAN 总线: sudo ip link set can0 type can bitrate 1000000
 *      sudo ip link set up can0
 *   3. 运行: ./example_set_zero
 */

#include <stdio.h>
#include <stdlib.h>
#include "miraculous_sdk.h"

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;

    /* --- 打开电机 --- */
    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    /* --- Bootstrap (NMT Reset → 等 Operational) --- */
    int ret = miraculous_motor_bootstrap(motor, 3000);
    if (ret < 0) {
        fprintf(stderr, "Bootstrap failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }

    /* --- 设为零点 --- */
    ret = miraculous_motor_set_zero_position(motor);
    if (ret < 0) {
        fprintf(stderr, "Set zero position failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }

    printf("Zero position set!\n");
    printf("(如需掉电保持, 请调用 miraculous_motor_save_config())\n");

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
