/**
 * @file    motion_mit.c
 * @brief   MIT 力位混合模式 (厂商特定, CIA_MODE_MIT = 0xFE)
 *
 * 目标转矩 = MIT_KP*(目标-实际) - MIT_KD*实际速度。
 * 每周期发 RPDO1 (0x6040+0x6060) + RPDO2 (0x607A) + SYNC。
 */
#include <stdio.h>
#include <string.h>
#include "miraculous_internal.h"

int miraculous_motor_mit_init(MiraMotor *motor, uint32_t sync_period_us)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    /* SYNC 策略 */
    motor->sync_active = false;
    int ret;

    if (sync_period_us > 0) {
        ret = miraculous_co_sync_start(motor->co, sync_period_us);
        if (ret < 0) {
            fprintf(stderr, "[mit] sync start failed\n");
            return ret;
        }

        /* 通知从机实际的 SYNC 周期，避免 CSP 超时检测误触发 */
        uint32_t period = sync_period_us;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_COMM_CYCLE_PERIOD, 0, &period, 200);
        printf("[mit] SDO write 0x1006 = %u us -> %s\n",
               period, (ret < 0) ? "FAIL" : "OK");
        if (ret < 0) return ret;

        motor->sync_active = true;
    }

    printf("[mit] init: node=%d cyc=%u us %s\n",
           motor->node_id, sync_period_us,
           motor->sync_active ? "(timer_sync)" : "(manual_sync)");
    return MRC_SUCCESS;
}

int miraculous_motor_mit_set_stiffness(MiraMotor *motor, float kp)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    /* 0x2005 sub1: MIT_KP (F32) — 刚度系数 [A/rad] */
    return CO_SDO_WRITE(motor->co, motor->node_id,
                        CIA402_OD_MIT_CONTROL, 0x01, &kp, 300);
}

int miraculous_motor_mit_set_damping(MiraMotor *motor, float kd)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    /* 0x2005 sub2: MIT_KD (F32) — 阻尼系数 [A*s/rad] */
    return CO_SDO_WRITE(motor->co, motor->node_id,
                        CIA402_OD_MIT_CONTROL, 0x02, &kd, 300);
}

int miraculous_motor_mit_set_torque_limit(MiraMotor *motor, int16_t limit)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    /* 0x2102: MIT Torque Limit (I16, 千分之一额定转矩) */
    return CO_SDO_WRITE(motor->co, motor->node_id,
                        CIA402_OD_MIT_TORQUE_LIMIT, 0x00, &limit, 300);
}

int miraculous_motor_mit_set_target(MiraMotor *motor, int32_t target_pos)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int ret;

    /* RPDO1 (0x200+nid): controlword 0x000F + mode 0xFE (MIT) */
    {
        uint32_t cob1 = CO_COB_RPDO1 + motor->node_id;
        uint8_t  cw_data[3];
        cw_data[0] = 0x0F;
        cw_data[1] = 0x00;
        cw_data[2] = 0xFE; /* MIT */
        ret = miraculous_co_pdo_send(motor->co, 1, cob1, cw_data, 3);
        if (ret < 0) return ret;
    }

    /* RPDO2 (0x300+nid): target position 0x607A (I32 LE) */
    {
        uint32_t cob2 = CO_COB_RPDO2 + motor->node_id;
        uint8_t  pos_data[4];
        pos_data[0] = (uint8_t)(target_pos & 0xFF);
        pos_data[1] = (uint8_t)((target_pos >> 8) & 0xFF);
        pos_data[2] = (uint8_t)((target_pos >> 16) & 0xFF);
        pos_data[3] = (uint8_t)((target_pos >> 24) & 0xFF);
        ret = miraculous_co_pdo_send(motor->co, 2, cob2, pos_data, 4);
        if (ret < 0) return ret;
    }

    if (!motor->sync_active) {
        return miraculous_co_sync_send_once(motor->co);
    }
    return MRC_SUCCESS;
}
