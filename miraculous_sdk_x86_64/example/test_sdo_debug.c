/**
 * @file    test_sdo_debug.c
 * @brief   SDO 调试: 只发送一次 0x6041 读, 验证重发机制
 *
 * 用法: ./example_sdo_debug [can_if] [node_id]
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "miraculous_sdk.h"

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;

    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    printf("Reading StatusWord (0x6041) from node %d...\n", node_id);

    uint16_t sw = 0;
    int ret = miraculous_motor_get_statusword(motor, &sw);
    if (ret == 0) {
        printf("StatusWord = 0x%04X\n", sw);
    } else {
        printf("StatusWord read failed: %s\n", mrc_strerror(ret));
    }

    miraculous_motor_close(motor);
    return (ret == 0) ? 0 : 1;
}
