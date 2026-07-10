/**
 * @file    motion_csv.c
 * @brief   周期同步速度模式 (CSV — Cyclic Synchronous Velocity)
 *
 * 每周期发 RPDO3 (0x60FF target velocity) + SYNC。
 * controlword + mode 在 csv_init + full_enable 中设置。
 * 两种 SYNC 模式: sync_period_us > 0=定时器, =0=手动。
 */
#include <stdio.h>
#include "miraculous_internal.h"

int miraculous_motor_csv_init(MiraMotor *motor, uint32_t sync_period_us)
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
        printf("[csv] SDO write 0x1006 = %u us -> %s\n",
               period, (ret < 0) ? "FAIL" : "OK");
        if (ret < 0) return ret;

        motor->sync_active = true;
        printf("[csv] init: node=%d timer_sync %u us\n",
               motor->node_id, sync_period_us);
    } else {
        printf("[csv] init: node=%d manual_sync\n", motor->node_id);
    }
    return MRC_SUCCESS;
}

int miraculous_motor_csv_set_target(MiraMotor *motor, int32_t target_vel)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int ret;

    /* 仅发 RPDO3 (0x400+nid): target velocity 0x60FF (I32 LE)
     * controlword + mode 已在 csv_init/init_full_enable 中设置 */
    {
        uint32_t cob3 = CO_COB_RPDO3 + motor->node_id;
        uint8_t  data[4];
        data[0] = (uint8_t)(target_vel & 0xFF);
        data[1] = (uint8_t)((target_vel >> 8) & 0xFF);
        data[2] = (uint8_t)((target_vel >> 16) & 0xFF);
        data[3] = (uint8_t)((target_vel >> 24) & 0xFF);
        ret = miraculous_co_pdo_send(motor->co, 3, cob3, data, 4);
        if (ret < 0) return ret;
    }

    if (!motor->sync_active) {
        return miraculous_co_sync_send_once(motor->co);
    }
    return MRC_SUCCESS;
}
