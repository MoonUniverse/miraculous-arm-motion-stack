/**
 * @file    test_get_position.c
 * @brief   获取位置示例: 脉冲 / 度 / 弧度
 *
 * 使用:
 *   1. 启动 vcan (测试): sudo ip link add dev vcan0 type vcan
 *      sudo ip link set up vcan0
 *   2. 或连接真实 CAN 总线: sudo ip link set can0 type can bitrate 1000000
 *      sudo ip link set up can0
 *   3. 运行: ./example_get_position
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

    /* 编码器默认 19-bit (524288 counts/rev), 如不同请调用
     *   miraculous_motor_set_encoder_bw(motor, bw);
     * 来修改。 */

    /* --- Bootstrap (NMT Reset → 等 Operational) --- */
    int ret = miraculous_motor_bootstrap(motor, 3000);
    if (ret < 0) {
        fprintf(stderr, "Bootstrap failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }

    /* --- 读取位置 --- */
    int32_t pulses;
    float deg, rad;

    ret = miraculous_motor_get_position(motor, &pulses);
    if (ret < 0) {
        fprintf(stderr, "Get position failed: %s\n", mrc_strerror(ret));
        goto cleanup;
    }

    miraculous_motor_get_position_ex(motor, &deg, POS_UNIT_DEGREE);
    miraculous_motor_get_position_ex(motor, &rad, POS_UNIT_RADIAN);

    printf("pulses=%d  deg=%.3f  rad=%.4f\n", pulses, deg, rad);

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
