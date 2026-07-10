/**
 * @file    motion_pv.c
 * @brief   轮廓速度模式 (PV — Profile Velocity) 实现
 *
 * 流程:
 *   1. 按需切换到 CIA_MODE_PV
 *   2. 设置 0x6086(运动曲线类型), 0x6083/0x6084(加/减速度)
 *   3. 设置 0x60FF(目标速度)
 *
 * 注: PV 模式速度即时生效, 无需控制字 bit4 翻转。
 */
#include <stdio.h>
#include "miraculous_internal.h"

int miraculous_motor_pv_move(MiraMotor *motor,
                              int32_t target_vel,
                              uint32_t acc,
                              uint32_t dec,
                              CiaProfileType_t profile_type)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    /* Step 1: 按需切换模式 (仅在首次或模式变更时) */
    Cia402Mode_t cur_mode;
    if (miraculous_motor_get_mode(motor, &cur_mode) == 0 &&
        cur_mode != CIA_MODE_PV && cur_mode != CIA_MODE_VEL) {
        int ret = miraculous_motor_set_mode(motor, CIA_MODE_PV);
        if (ret < 0) return ret;
    }

    /* Step 2: 运动曲线类型 (0x6086) — 仅首次尝试 */
    static bool profile_set = false;
    if (!profile_set) {
        int8_t pt = (int8_t)profile_type;
        int ret = CO_SDO_WRITE(motor->co, motor->node_id,
                               CIA402_OD_MOTION_PROFILE_TYPE, 0,
                               &pt, 5);
        if (ret < 0) {
            fprintf(stderr, "[pv] motion profile type 0x6086 not supported\n");
        }
        profile_set = true;
    }

    /* Step 3: 轨迹参数 */
    int ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_PROFILE_ACCELERATION, 0, &acc, 5);
    if (ret < 0) return ret;

    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_PROFILE_DECELERATION, 0, &dec, 5);
    if (ret < 0) return ret;

    /* Step 4: 目标速度 — PV 模式即时生效 */
    return CO_SDO_WRITE(motor->co, motor->node_id,
                        CIA402_OD_TARGET_VELOCITY, 0, &target_vel, 5);
}
