/**
 * @file    co_nmt.c
 * @brief   NMT 网络管理协议实现
 *
 * NMT 帧格式 (CAN ID = 0x000, DLC = 2):
 *   Byte 0: Command Specifier (cs)
 *   Byte 1: Node-ID (0 = 广播)
 */

#include <stdio.h>
#include "miraculous_internal.h"

int miraculous_co_nmt_send(MiraCoMaster *co, uint8_t node_id,
                           CoNmtCommand_t cmd)
{
    if (!co) return MRC_ERROR_INVALID_PARAM;

    uint8_t data[2];
    data[0] = (uint8_t)cmd;
    data[1] = (node_id == 0) ? 0 : node_id;

    MiraCanCtx *can = miraculous_co_get_can(co);
    return miraculous_can_send(can, CO_COB_NMT, data, 2);
}
