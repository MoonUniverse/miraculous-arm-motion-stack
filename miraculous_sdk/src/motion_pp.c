/**
 * @file    motion_pp.c
 * @brief   轮廓位置模式 (PP — Profile Position) 实现
 *
 * PP 模式流程:
 *   1. 切换到 CIA_MODE_PP (0x6060 = 1)
 *   2. 设置轨迹参数: 0x6081(轮廓速度), 0x6083(加速度), 0x6084(减速度)
 *   3. 设置目标位置: 0x607A
 *   4. 触发: 写 controlword bit4=1 + bit5(immediate) + bit6(relative)
 *   5. 等待 bit10 (Target Reached)
 *
 * 注意: 必须在 Operation Enabled 状态下发命令
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <time.h>
#include "miraculous_internal.h"

int miraculous_motor_pp_move(MiraMotor *motor,
                              int32_t target_pos,
                              uint32_t profile_vel,
                              uint32_t acc,
                              uint32_t dec,
                              bool relative,
                              bool immediate)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    /* Step 1: 设置模式为 PP */
    int ret = miraculous_motor_set_mode(motor, CIA_MODE_PP);
    if (ret < 0) {
        fprintf(stderr, "[pp] set mode failed\n");
        return ret;
    }

    /* Step 2: 设置轨迹参数 */
    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_PROFILE_VELOCITY, 0,
                       &profile_vel, 100);
    if (ret < 0) return ret;

    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_PROFILE_ACCELERATION, 0,
                       &acc, 100);
    if (ret < 0) return ret;

    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_PROFILE_DECELERATION, 0,
                       &dec, 100);
    if (ret < 0) return ret;

    /* Step 3: 设置目标位置 */
    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_TARGET_POSITION, 0,
                       &target_pos, 100);
    if (ret < 0) return ret;

    /* Step 4: 触发运动 — 构建控制字
     * bit4 (New Set-point): 1
     * bit5 (Change Set Immediately): immediate
     * bit6 (Absolute/Relative): relative
     * bits0-3: Enable Operation = 0x0F
     */
    uint16_t cw = CIA402_CW_ENABLE_OPERATION
                | CIA402_CW_NEW_SET_POINT_MASK;
    if (immediate) cw |= CIA402_CW_CHANGE_SET_IMM_MASK;
    if (relative)  cw |= CIA402_CW_ABS_REL_MASK;

    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_CONTROLWORD, 0,
                       &cw, 100);
    if (ret < 0) return ret;

    /* 清零 bit4 (New Set-point 是上升沿触发) */
    cw &= ~CIA402_CW_NEW_SET_POINT_MASK;
    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_CONTROLWORD, 0,
                       &cw, 100);

    return ret;
}

int miraculous_motor_pp_wait_target(MiraMotor *motor, int timeout_ms)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    if (timeout_ms <= 0) timeout_ms = 10000;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        uint16_t sw;
        int ret = CO_SDO_READ(motor->co, motor->node_id,
                              CIA402_OD_STATUSWORD, 0, &sw, 100);
        if (ret < 0) return ret;

        /* bit10 = Target Reached? */
        if (sw & (1U << CIA402_SW_TARGET_REACHED_BIT)) {
            return MRC_SUCCESS;
        }

        /* Fault 检查 */
        if (sw & (1U << CIA402_SW_FAULT_BIT)) {
            return MRC_ERROR_MOTION_FAULT;
        }

        /* 超时 */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t elapsed = (now.tv_sec - start.tv_sec) * 1000LL
                        + (now.tv_nsec - start.tv_nsec) / 1000000LL;
        if (elapsed >= timeout_ms) return MRC_ERROR_TIMEOUT;

        int remaining = (int)(timeout_ms - elapsed);
        if (remaining > 10) remaining = 10;
        miraculous_co_poll(motor->co, remaining);
    }
}
