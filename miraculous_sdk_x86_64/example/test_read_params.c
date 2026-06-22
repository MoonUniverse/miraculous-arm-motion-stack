/**
 * @file    test_read_params.c
 * @brief   参数读写示例
 *
 * 演示读取电机状态、位置、速度、温度，以及读写 SDO。
 *
 * 用法: ./example_read_params [can_if] [node_id]
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

    if (miraculous_motor_bootstrap(motor, 3000) < 0) {
        fprintf(stderr, "Bootstrap failed — is motor powered on?\n");
        goto cleanup;
    }

    printf("=== Motor Info ===\n");

    /* --- 设备类型 0x1000 --- */
    uint32_t dev_type;
    uint8_t len = 4;
    if (miraculous_motor_sdo_read(motor, 0x1000, 0, &dev_type, &len) == 0) {
        printf("Device Type:    0x%08X\n", dev_type);
    }

    /* --- 波特率 0x2001 sub2 --- */
    uint16_t baudrate;
    if (miraculous_motor_get_baudrate(motor, &baudrate) == 0) {
        printf("Baudrate:       %u kbps\n", baudrate);
    }

    /* --- 心跳周期 0x1017 --- */
    uint16_t hb_period;
    if (miraculous_motor_get_heartbeat(motor, &hb_period) == 0) {
        printf("Heartbeat:      %u ms (0=disabled)\n", hb_period);
        if (hb_period == 0) {
            miraculous_motor_set_heartbeat(motor, 1000);
            printf("  -> Enabled: 1000 ms\n");
        }
    }

    /* --- 身份 0x1018 --- */
    uint32_t vendor_id, product_code, rev, serial;
    len = 4;
    miraculous_motor_sdo_read(motor, 0x1018, 1, &vendor_id, &len);
    len = 4;
    miraculous_motor_sdo_read(motor, 0x1018, 2, &product_code, &len);
    len = 4;
    miraculous_motor_sdo_read(motor, 0x1018, 3, &rev, &len);
    len = 4;
    miraculous_motor_sdo_read(motor, 0x1018, 4, &serial, &len);
    printf("Vendor ID:      0x%08X\n", vendor_id);
    printf("Product Code:   0x%08X\n", product_code);
    printf("Revision:       0x%08X\n", rev);
    printf("Serial:         0x%08X\n", serial);

    /* --- PDS 状态 --- */
    Cia402State_t state;
    if (miraculous_motor_get_state(motor, &state) == 0) {
        printf("PDS State:      %d\n", state);
    }

    /* --- 支持的驱动模式 --- */
    uint32_t modes;
    if (miraculous_motor_get_supported_modes(motor, &modes) == 0) {
        printf("Drive Modes:    0x%08X\n", modes);
    }

    /* --- 温度 (仅当使能后才有值) --- */
    if (miraculous_motor_full_enable(motor) == 0) {
        int16_t mt, mos;
        if (miraculous_motor_get_temperature(motor, &mt, &mos) == 0) {
            printf("Motor Temp:     %d C\n", mt);
            printf("MOS Temp:       %d C\n", mos);
        }

        /* --- 读取电流环 PI 参数 (0x2002) --- */
        int16_t  cp_kp, cp_ki;
        uint16_t cp_kp_div, cp_ki_div;
        if (miraculous_motor_get_current_pi(motor, &cp_kp, &cp_kp_div,
                                             &cp_ki, &cp_ki_div) == 0) {
            printf("Current PI Kp:  %d / 2^%u = %f\n",
                   cp_kp, cp_kp_div, (double)cp_kp / (double)(1u << cp_kp_div));
            printf("Current PI Ki:  %d / 2^%u = %f\n",
                   cp_ki, cp_ki_div, (double)cp_ki / (double)(1u << cp_ki_div));
        }

        /* --- 读取速度环 PI 参数 (0x2003) --- */
        int16_t  vp_kp, vp_ki;
        uint16_t vp_kp_div, vp_ki_div;
        if (miraculous_motor_get_velocity_pi(motor, &vp_kp, &vp_kp_div,
                                              &vp_ki, &vp_ki_div) == 0) {
            printf("Velocity PI Kp: %d / 2^%u = %f\n",
                   vp_kp, vp_kp_div, (double)vp_kp / (double)(1u << vp_kp_div));
            printf("Velocity PI Ki: %d / 2^%u = %f\n",
                   vp_ki, vp_ki_div, (double)vp_ki / (double)(1u << vp_ki_div));
        }

        /* --- 读取位置环 PI 参数 (0x2004) --- */
        int16_t  pp_kp;
        uint16_t pp_kp_div, pp_ki, pp_ki_div;
        if (miraculous_motor_get_position_pi(motor, &pp_kp, &pp_kp_div,
                                              &pp_ki, &pp_ki_div) == 0) {
            printf("Position PI Kp: %d / 2^%u = %f\n",
                   pp_kp, pp_kp_div, (double)pp_kp / (double)(1u << pp_kp_div));
            printf("Position PI Ki: %d / 2^%u = %f\n",
                   pp_ki, pp_ki_div, (double)pp_ki / (double)(1u << pp_ki_div));
        }

        /* --- 演示修改电流环 PI (设置当前值为例) --- */
        if (miraculous_motor_set_current_pi(motor, cp_kp, cp_kp_div,
                                             cp_ki, cp_ki_div) == 0) {
            printf("Current PI params re-confirmed (no change)\n");
        }

        miraculous_motor_shutdown(motor);
    }

    printf("=== End ===\n");

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
