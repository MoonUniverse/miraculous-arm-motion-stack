/**
 * @file    test_pdo_config.c
 * @brief   演示 full_enable 使能 + 读取实时数据
 *
 * 用法: ./example_pdo_config [can_if] [node_id]
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;
static void signal_handler(int sig) { (void)sig; g_quit = true; }

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    if (miraculous_motor_full_enable(motor) < 0) {
        fprintf(stderr, "full_enable failed\n");
        miraculous_motor_close(motor);
        return -1;
    }
    printf("Motor enabled via SDO.\n");

    while (!g_quit) {
        int32_t pos, vel;
        int16_t tor;
        if (miraculous_motor_get_position(motor, &pos) == 0 &&
            miraculous_motor_get_velocity(motor, &vel) == 0 &&
            miraculous_motor_get_torque(motor, &tor) == 0) {
            printf("pos=%d  vel=%d  tor=%d\n", pos, vel, tor);
        }
        miraculous_motor_poll(motor, 200);
    }

    miraculous_motor_close(motor);
    return 0;
}
