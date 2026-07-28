/**
 * @file    co_emcy.c
 * @brief   EMCY 紧急报文处理
 *
 * EMCY 帧格式 (CAN ID = 0x080 + NodeID, DLC = 8):
 *   Bytes 0-1: Emergency Error Code (U16 LE)
 *   Byte 2:    Error Register (0x1001)
 *   Bytes 3-7: Manufacturer-specific Error Field
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "miraculous_internal.h"

typedef struct EmcyCtx_t {
    MiraEmcyCallback callback;
    void            *user_data;
} EmcyCtx_t;

EmcyCtx_t* co_emcy_create(void)
{
    return calloc(1, sizeof(EmcyCtx_t));
}

void co_emcy_destroy(EmcyCtx_t *ctx)
{
    free(ctx);
}

int miraculous_co_emcy_set_callback(MiraCoMaster *co,
                                     MiraEmcyCallback cb,
                                     void *user_data)
{
    extern EmcyCtx_t* miraculous_co_get_emcy(MiraCoMaster *co);
    EmcyCtx_t *emcy = miraculous_co_get_emcy(co);
    if (!emcy) return MRC_ERROR_NOT_INIT;

    emcy->callback  = cb;
    emcy->user_data = user_data;
    return MRC_SUCCESS;
}

void co_emcy_handle(EmcyCtx_t *ctx, uint32_t can_id,
                     const uint8_t *data, uint8_t len)
{
    if (!ctx || !data || len < 3) return;

    /* 解析 EMCY */
    uint8_t  node_id    = can_id - CO_COB_EMCY_BASE;
    uint16_t error_code = data[0] | ((uint16_t)data[1] << 8);
    uint8_t  error_reg  = data[2];
    const uint8_t *mfg  = (len > 3) ? &data[3] : NULL;
    uint8_t  mfg_len    = (len > 3) ? (len - 3) : 0;

    if (mfg_len > 5) mfg_len = 5;

    printf("[emcy] node=%d code=0x%04X reg=0x%02X\n",
           node_id, error_code, error_reg);

    if (ctx->callback) {
        ctx->callback(node_id, error_code, error_reg,
                     mfg, mfg_len, ctx->user_data);
    }
}
