/**
 * @file    motion_pt.c
 * @brief   轮廓转矩模式 (PT — Profile Torque) 实现
 *
 * 流程:
 *   1. 切换到 CIA_MODE_PT (0x6060 = 4)
 *   2. 设置 0x6087(转矩斜率) — 可选
 *   3. 设置 0x6071(目标转矩) + 触发
 */

#include <stdio.h>
#include "miraculous_internal.h"

#define PT_CTRL_SDO_TIMEOUT_MS 2000

int miraculous_motor_pt_move(MiraMotor *motor,
                              int16_t target_torque,
                              uint16_t slope)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int ret;

    /* Step 1: 转矩斜率 (0x6087) — 仅首次尝试 */
    static bool slope_unsupported = false;
    if (slope > 0 && !slope_unsupported) {
        uint32_t slope_val = slope;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_TORQUE_SLOPE, 0, &slope_val, PT_CTRL_SDO_TIMEOUT_MS);
        if (ret < 0) {
            slope_unsupported = true;
            /* 非致命, 记录一次即可 */
            fprintf(stderr, "[pt] torque slope 0x6087 not supported\n");
        }
    }

    /* Step 3: 目标转矩 0x6071 */
    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_TARGET_TORQUE, 0, &target_torque, PT_CTRL_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* 目标转矩写入后立即生效, 无需额外触发 */
    return MRC_SUCCESS;
}
