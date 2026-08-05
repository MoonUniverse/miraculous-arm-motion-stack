/**
 * @file    motion_csp.c
 * @brief   周期同步位置模式 (CSP — Cyclic Synchronous Position)
 *
 * 通过 SDO 设置 CSP 模式 (0x6060)，使能由 full_enable 完成。
 * SYNC 策略通过 sync_period_us 选择: >0=定时器, =0=手动。
 *
 * PDO 映射 (EDS 出厂预配):
 *   - RPDO1 (0x200+nid): 0x6040:10 controlword (U16) + 0x6060:08 mode (I8)
 *   - RPDO2 (0x300+nid): 0x607A:20 target position (I32)
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "miraculous_internal.h"

int miraculous_motor_csp_init(MiraMotor *motor, uint32_t sync_period_us, bool manual)
{
    if (!motor)
        return MRC_ERROR_INVALID_PARAM;

    int ret;

    motor->sync_active = false;

    /* SYNC 策略 + 0x1006 超时 (仅 timer 模式) */
    if (!manual)
    {
        uint32_t period = sync_period_us;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_COMM_CYCLE_PERIOD, 0, &period, 200);
        if (ret < 0) {
            fprintf(stderr, "[csp] warning: write 0x1006 failed (continuing)\n");
        } else {
            printf("[csp] SDO write 0x1006 = %u us -> OK\n", period);
        }

        ret = miraculous_co_sync_start(motor->co, sync_period_us);
        if (ret < 0)
        {
            fprintf(stderr, "[csp] sync start failed\n");
            return ret;
        }

        motor->sync_active = true;
        printf("[csp] init: node=%d timer_sync %u us\n",
               motor->node_id, sync_period_us);
    }
    else
    {
        motor->sync_active = false;
        printf("[csp] init: node=%d manual_sync\n", motor->node_id);
    }


    return MRC_SUCCESS;
}

int miraculous_motor_csp_set_target(MiraMotor *motor, int32_t target_pos)
{
    if (!motor)
        return MRC_ERROR_INVALID_PARAM;

    int ret;

    /* 仅通过 PDO 发送 RPDO2 (位置) */
    uint32_t cob2 = CO_COB_RPDO2 + motor->node_id;
    const uint32_t raw = (uint32_t)target_pos;
    uint8_t pos_data[4];
    pos_data[0] = (uint8_t)(raw & 0xFFU);
    pos_data[1] = (uint8_t)((raw >> 8) & 0xFFU);
    pos_data[2] = (uint8_t)((raw >> 16) & 0xFFU);
    pos_data[3] = (uint8_t)((raw >> 24) & 0xFFU);
    ret = miraculous_co_pdo_send(motor->co, 2, cob2, pos_data, 4);
    if (ret < 0)
        return ret;

    return MRC_SUCCESS;
}

int miraculous_motor_csp_set_target_ex(MiraMotor *motor, float target_pos, PosUnit_t unit)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int32_t pulses;
    int ret = miraculous_position_to_counts(
        target_pos, unit, motor->encoder_bw, &pulses);
    if (ret < 0) return ret;

    return miraculous_motor_csp_set_target(motor, pulses);
}
