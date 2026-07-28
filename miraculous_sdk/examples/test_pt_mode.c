/**
 * @file    test_pt_mode.c
 * @brief   PT 轮廓转矩模式示例
 *
 * 演示轮廓转矩模式 (Profile Torque) 的使用。
 * 设置目标转矩和斜率，循环切换正反转转矩。
 *
 * 用法: ./example_pt_mode [can_if] [node_id]
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
      /* --- 设置操作模式为 PT --- */
    if (miraculous_motor_set_mode(motor, CIA_MODE_PT) < 0) {
        fprintf(stderr, "Set PT mode failed\n");
        goto cleanup;
    }
    
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

    printf("PT mode — toggling torque direction every 5s...\n");

    int16_t torque_values[] = {100, 0, -100, 0};
    int torque_idx = 0;
    uint16_t slope = 2;         /* 2A/s */
    printf(" slope = %d A/s\n",slope);

    while (!g_quit) {
        int16_t target_torque = torque_values[torque_idx % 4];
        torque_idx++;

        int ret = miraculous_motor_pt_move(motor, target_torque, slope);
        if (ret < 0) {
            fprintf(stderr, "PT move failed: %s\n", mrc_strerror(ret));
            break;
        }

        printf("Running torque = %d (0.01A)\n", target_torque);

        /* 运行 3 秒, 每 3ms 通过 TPDO 读取实际转矩 */
        for (int i = 0; i < 600 && !g_quit; i++) {
            miraculous_motor_sync_send(motor);
            usleep(5000);
            int16_t torque;
            if (miraculous_motor_get_torque(motor, &torque) == 0)
                printf(" t=%dms  Actual torque = %d (0.01A) \r\n ",
                       (i + 1) * 5, torque);
        }
    }

shutdown:
    miraculous_motor_shutdown(motor);

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
