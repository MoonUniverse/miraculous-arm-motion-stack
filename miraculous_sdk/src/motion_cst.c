/**
 * @file    motion_cst.c
 * @brief   周期同步转矩模式 (CST — Cyclic Synchronous Torque)
 *
 * 每周期发 RPDO1 (0x6040+0x6060) + RPDO4 (0x6071) + SYNC。
 * 两种 SYNC 模式: sync_period_us > 0=定时器, =0=手动。
 */
#include <stdio.h>
#include "miraculous_internal.h"

int miraculous_motor_cst_init(MiraMotor *motor, uint32_t sync_period_us)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int ret;

    motor->sync_active = false;

    if (sync_period_us > 0) {
        ret = miraculous_co_sync_start(motor->co, sync_period_us);
        if (ret < 0) return ret;

        /* 通知从机实际的 SYNC 周期，避免 CSP 超时检测误触发 */
        uint32_t period = sync_period_us;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_COMM_CYCLE_PERIOD, 0, &period, 200);
        printf("[cst] SDO write 0x1006 = %u us -> %s\n",
               period, (ret < 0) ? "FAIL" : "OK");
        if (ret < 0) return ret;

        motor->sync_active = true;
        printf("[cst] init: node=%d timer_sync %u us\n",
               motor->node_id, sync_period_us);
    } else {
        printf("[cst] init: node=%d manual_sync\n", motor->node_id);
    }
    return MRC_SUCCESS;
}

int miraculous_motor_cst_set_target(MiraMotor *motor, int16_t target_torque)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int ret;

    /* RPDO1 (0x200+nid): controlword 0x000F + mode 0x0A (CST) */
    {
        uint32_t cob1 = CO_COB_RPDO1 + motor->node_id;
        uint8_t  cw_data[3];
        cw_data[0] = 0x0F;
        cw_data[1] = 0x00;
        cw_data[2] = 0x0A; /* CST */
        ret = miraculous_co_pdo_send(motor->co, 1, cob1, cw_data, 3);
        if (ret < 0) return ret;
    }

    /* RPDO4 (0x500+nid): target torque 0x6071 (I16 LE) */
    {
        uint32_t cob4 = CO_COB_RPDO4 + motor->node_id;
        uint8_t  data[2];
        data[0] = (uint8_t)(target_torque & 0xFF);
        data[1] = (uint8_t)((target_torque >> 8) & 0xFF);
        ret = miraculous_co_pdo_send(motor->co, 4, cob4, data, 2);
        if (ret < 0) return ret;
    }

    if (!motor->sync_active) {
        return miraculous_co_sync_send_once(motor->co);
    }
    return MRC_SUCCESS;
}
