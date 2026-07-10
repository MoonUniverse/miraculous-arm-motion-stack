/**
 * @file    motor_diag.c
 * @brief   电机诊断工具 — 读取并显示电机详细状态
 *
 * 用法: ./motor_diag [can_if] [node_id]
 */

#include <stdio.h>
#include <stdlib.h>
#include "miraculous_sdk.h"

static void print_state(Cia402State_t s)
{
    switch (s) {
    case CIA_STATE_NOT_READY_TO_SWITCH_ON: printf("Not Ready to Switch On"); break;
    case CIA_STATE_SWITCH_ON_DISABLED:     printf("Switch On Disabled");     break;
    case CIA_STATE_READY_TO_SWITCH_ON:     printf("Ready to Switch On");     break;
    case CIA_STATE_SWITCHED_ON:            printf("Switched On");            break;
    case CIA_STATE_OPERATION_ENABLED:      printf("Operation Enabled");      break;
    case CIA_STATE_QUICK_STOP_ACTIVE:      printf("Quick Stop Active");      break;
    case CIA_STATE_FAULT_REACTION_ACTIVE:  printf("Fault Reaction Active");  break;
    case CIA_STATE_FAULT:                  printf("FAULT!");                 break;
    default:                               printf("Unknown (0x%02X)", s);   break;
    }
}

static void print_mode(Cia402Mode_t m)
{
    switch (m) {
    case CIA_MODE_NONE: printf("None");        break;
    case CIA_MODE_PP:   printf("Profile Position"); break;
    case CIA_MODE_PV:   printf("Profile Velocity"); break;
    case CIA_MODE_PT:   printf("Profile Torque");   break;
    case CIA_MODE_HM:   printf("Homing");       break;
    case CIA_MODE_IP:   printf("Interpolated Position"); break;
    case CIA_MODE_CSP:  printf("CSP");          break;
    case CIA_MODE_CSV:  printf("CSV");          break;
    case CIA_MODE_CST:  printf("CST");          break;
    case CIA_MODE_MIT:  printf("MIT (Force-Pos)"); break;
    default:            printf("Unknown (0x%02X)", m); break;
    }
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;

    MiraMotor *motor = miraculous_motor_open(ifname, 0, node_id);
    if (!motor) { fprintf(stderr, "Failed to open motor\n"); return -1; }

    if (miraculous_motor_bootstrap(motor, 3000) < 0) {
        fprintf(stderr, "Motor not responding!\n");
        goto cleanup;
    }

    printf("=== Miraculous Motor Diagnostic ===\n");
    printf("Node ID:        %d\n", node_id);

    /* 状态 */
    uint16_t sw;
    Cia402State_t state;
    if (miraculous_motor_get_statusword(motor, &sw) == 0) {
        printf("Statusword:     0x%04X\n", sw);
    }
    if (miraculous_motor_get_state(motor, &state) == 0) {
        printf("PDS State:      ");
        print_state(state);
        printf("\n");
    }

    /* 模式 */
    Cia402Mode_t mode;
    if (miraculous_motor_get_mode(motor, &mode) == 0) {
        printf("Operate Mode:   ");
        print_mode(mode);
        printf("\n");
    }

    /* 错误码 */
    uint16_t err_code;
    uint8_t len = 2;
    if (miraculous_motor_sdo_read(motor, CIA402_OD_ERROR_CODE, 0,
                                   &err_code, &len) == 0) {
        printf("Error Code:     0x%04X\n", err_code);
    }

    /* 实际值 */
    int32_t pos, vel;
    int16_t torque;
    if (miraculous_motor_get_position(motor, &pos) == 0)
        printf("Actual Pos:     %d pulses\n", pos);
    if (miraculous_motor_get_velocity(motor, &vel) == 0)
        printf("Actual Vel:     %d RPM\n", vel);
    if (miraculous_motor_get_torque(motor, &torque) == 0)
        printf("Actual Torque:  %d (0.01A)\n", torque);

    /* 温度 (需要使能后才有有效值) */
    if (miraculous_motor_full_enable(motor) == 0) {
        int16_t mt, mos;
        if (miraculous_motor_get_temperature(motor, &mt, &mos) == 0) {
            printf("Motor Temp:     %d °C\n", mt);
            printf("MOS Temp:       %d °C\n", mos);
        }
        miraculous_motor_shutdown(motor);
    }

    printf("=== End ===\n");

cleanup:
    miraculous_motor_close(motor);
    return 0;
}
