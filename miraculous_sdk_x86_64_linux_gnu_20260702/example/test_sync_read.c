/**
 * @file    test_sync_read.c
 * @brief   SYNC 触发 TPDO 读取示例 — 多节点位置 + 速度 + 转矩
 *
 * 一个 SYNC 同时触发多个电机上报 TPDO，
 * 一次 poll 即可获取所有电机的缓存数据。
 *
 * 使用:
 *   ./example_sync_read [can_if] [node_list]
 *   默认: can0, node 1
 *   示例: ./example_sync_read can1 1,2,3,4,5
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

#define MAX_NODES 10

static volatile bool g_quit = false;
static void signal_handler(int sig) { (void)sig; g_quit = true; }

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int ids[MAX_NODES];
    int count = 0;

    if (argc > 2) {
        char *token = strtok(argv[2], ",");
        while (token && count < MAX_NODES) {
            ids[count++] = atoi(token);
            token = strtok(NULL, ",");
        }
    } else {
        ids[count++] = 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);


    /* 打开并使能所有电机 */
    MiraMotor *motors[MAX_NODES] = {0};
    int opened = 0;
    for (int i = 0; i < count; i++) {
        int id = ids[i];
        motors[opened] = miraculous_motor_open(ifname, 0, id);
        if (!motors[opened]) { fprintf(stderr, "Failed to open node %d\n", id); continue; }

        if (miraculous_motor_bootstrap(motors[opened], 3000) < 0) {
            fprintf(stderr, "Bootstrap node %d failed\n", id);
            miraculous_motor_close(motors[opened]);
            continue;
        }
        if (miraculous_motor_full_enable(motors[opened]) < 0) {
            fprintf(stderr, "Enable node %d failed\n", id);
            miraculous_motor_close(motors[opened]);
            continue;
        }
        opened++;
    }

    if (opened == 0) { fprintf(stderr, "No motors found\n"); return -1; }

    printf("=== SYNC TPDO Read (%d motors) ===\n", opened);
    printf("ID | Pos(pulses) | Vel(RPM) | Torque(0.01A)\n");
    printf("---|------------|----------|--------------\n");

    while (!g_quit) {
        /* 一个 SYNC 触发所有电机上报 TPDO */
        miraculous_motor_sync_send(motors[0]);

        /* 一次 poll 收完所有 TPDO 帧 */
        miraculous_motor_poll(motors[0], 2);

        /* 从缓存读取每个电机数据 */
        for (int i = 0; i < opened; i++) {
            int32_t pos, vel;
            int16_t torque;
            if (miraculous_motor_get_position(motors[i], &pos) < 0) continue;
            miraculous_motor_get_velocity(motors[i], &vel);
            miraculous_motor_get_torque(motors[i], &torque);
            printf("%2d | %9d | %7d | %11d\n",
                   miraculous_motor_get_node_id(motors[i]), pos, vel, torque);
        }
        printf("\n");
        usleep(20000);
    }

    for (int i = 0; i < opened; i++) {
        miraculous_motor_shutdown(motors[i]);
        miraculous_motor_close(motors[i]);
    }
    return 0;
}
