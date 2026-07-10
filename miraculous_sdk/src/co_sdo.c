/**
 * @file    co_sdo.c
 * @brief   SDO (Service Data Object) 协议实现 — Expedited 读写
 *
 * SDO Expedited Upload (读从站 OD):
 *   Master → Slave: 0x600+NID | 0x40 | index_lo | index_hi | sub | 0 | 0 | 0 | 0
 *   Slave → Master: 0x580+NID | 0x4F | index_lo | index_hi | sub | (data 1-4 B)...
 *
 * SDO Expedited Download (写从站 OD):
 *   Master → Slave: 0x600+NID | 0x22/0x23 | index_lo | index_hi | sub | (data 1-4 B)...
 *   Slave → Master: 0x580+NID | 0x60 | index_lo | index_hi | sub | 0 | 0 | 0 | 0
 *
 * SDO Abort (双方都可发):
 *   Byte 0: 0x80, Byte 4-7: Abort Code (U32 LE)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include "miraculous_internal.h"

/* 默认 SDO 超时 */
#define SDO_DEFAULT_TIMEOUT_MS  10
#define SDO_RETRIES             4

/*----------------------------------------------------------------------------
 * SDO Expedited Upload (读)
 *----------------------------------------------------------------------------*/

int miraculous_co_sdo_read(MiraCoMaster *co, uint8_t node_id,
                            uint16_t index, uint8_t subindex,
                            void *data, uint8_t *data_len, int timeout_ms)
{
    if (!co || !data || !data_len || *data_len == 0)
        return MRC_ERROR_INVALID_PARAM;

    MiraCanCtx *can = miraculous_co_get_can(co);
    if (!can) return MRC_ERROR_NOT_INIT;

    if (timeout_ms <= 0) timeout_ms = SDO_DEFAULT_TIMEOUT_MS;

    /* --- 构造 SDO Upload Request --- */
    uint8_t req[8];
    memset(req, 0, 8);
    req[0] = CO_SDO_CCS_UPLOAD_INITIATE;     /* 0x40 */
    req[1] = (uint8_t)(index & 0xFF);         /* index low */
    req[2] = (uint8_t)((index >> 8) & 0xFF);  /* index high */
    req[3] = subindex;
    /* req[4-7] = 0 */

    /* 发送请求 */
    int ret = miraculous_can_send(can, CO_COB_SDO_RX + node_id, req, 8);
    if (ret < 0) return ret;

    /* 等待响应 (循环，超时则继续等，跳过不匹配的帧) */
    uint8_t resp[8];
    uint8_t resp_len;
    int retries = SDO_RETRIES;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        resp_len = 8;
        ret = miraculous_can_recv_timeout(can, CO_COB_SDO_TX + node_id,
                                          resp, &resp_len, timeout_ms);
        if (ret == MRC_ERROR_TIMEOUT) {
            continue;
        }
        if (ret < 0) return ret;

        /* 检查 Abort */
        if ((resp[0] & 0xE0) == CO_SDO_CCS_ABORT) {
            uint32_t abort_code = resp[4] | ((uint32_t)resp[5] << 8)
                                | ((uint32_t)resp[6] << 16)
                                | ((uint32_t)resp[7] << 24);
            fprintf(stderr, "[sdo] read abort: node=%d idx=0x%04X sub=%d "
                    "code=0x%08X\n", node_id, index, subindex, abort_code);
            return MRC_ERROR_CO_SDO_ABORT;
        }

        /* 检查 SCS */
        uint8_t scs = resp[0] & 0xE0;
        if (scs != CO_SDO_SCS_UPLOAD_RESPONSE) {  /* 0x40 */
            continue;
        }

        /* 检查 index/subindex 是否匹配 */
        uint16_t resp_idx = (uint16_t)resp[1] | ((uint16_t)resp[2] << 8);
        if (resp_idx != index || resp[3] != subindex) {
            continue;
        }

        /* 检查 expedited (bit 1) */
        if (!(resp[0] & 0x02)) {
            fprintf(stderr, "[sdo] segmented upload not supported\n");
            return MRC_ERROR_NOT_SUPPORTED;
        }

        /* 检查 size indicated (bit 0) */
        uint8_t size_flag = resp[0] & 0x01;

        /* 提取数据 (bytes 4-7) */
        uint8_t payload_len;
        if (size_flag) {
            uint8_t n = (resp[0] >> 2) & 0x03;
            payload_len = 4 - n;
        } else {
            payload_len = 4;
        }

        if (payload_len > *data_len) payload_len = *data_len;
        memcpy(data, &resp[4], payload_len);
        *data_len = payload_len;

        return MRC_SUCCESS;
    }

    fprintf(stderr, "[sdo] read timeout after %d retries: node=%d "
            "idx=0x%04X sub=%d\n", retries, node_id, index, subindex);
    return MRC_ERROR_TIMEOUT;
}

/*----------------------------------------------------------------------------
 * SDO Expedited Download (写)
 *----------------------------------------------------------------------------*/

int miraculous_co_sdo_write(MiraCoMaster *co, uint8_t node_id,
                             uint16_t index, uint8_t subindex,
                             const void *data, uint8_t data_len,
                             int timeout_ms)
{
    if (!co || !data || data_len == 0 || data_len > 4)
        return MRC_ERROR_INVALID_PARAM;

    MiraCanCtx *can = miraculous_co_get_can(co);
    if (!can) return MRC_ERROR_NOT_INIT;

    if (timeout_ms <= 0) timeout_ms = SDO_DEFAULT_TIMEOUT_MS;

    /* --- 构造 SDO Download Request --- */
    uint8_t req[8];
    memset(req, 0, 8);

    /* 0x22 = expedited download, size indicated */
    req[0] = CO_SDO_CCS_DOWNLOAD_INITIATE | CO_SDO_EXPEDITED
           | CO_SDO_SIZE_INDICATED;
    /* 指示未使用的字节数 (bits 3-2) */
    req[0] |= (uint8_t)((4 - data_len) << 2);

    req[1] = (uint8_t)(index & 0xFF);
    req[2] = (uint8_t)((index >> 8) & 0xFF);
    req[3] = subindex;

    /* 数据放在 bytes 4-7 (小端序) */
    memcpy(&req[4], data, data_len);

    /* 发送请求 */
    int ret = miraculous_can_send(can, CO_COB_SDO_RX + node_id, req, 8);
    if (ret < 0) return ret;

    /* 等待响应 (循环，超时则继续等) */
    uint8_t resp[8];
    uint8_t resp_len;
    int retries = SDO_RETRIES;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        resp_len = 8;
        ret = miraculous_can_recv_timeout(can, CO_COB_SDO_TX + node_id,
                                          resp, &resp_len, timeout_ms);
        if (ret == MRC_ERROR_TIMEOUT) {
            continue;
        }
        if (ret < 0) return ret;

        /* 检查 Abort */
        if ((resp[0] & 0xE0) == CO_SDO_CCS_ABORT) {
            uint32_t abort_code = resp[4] | ((uint32_t)resp[5] << 8)
                                | ((uint32_t)resp[6] << 16)
                                | ((uint32_t)resp[7] << 24);
            fprintf(stderr, "[sdo] write abort: node=%d idx=0x%04X sub=%d "
                    "code=0x%08X\n", node_id, index, subindex, abort_code);
            return MRC_ERROR_CO_SDO_ABORT;
        }

        /* 检查 SCS */
        uint8_t scs = resp[0] & 0xE0;
        if (scs != CO_SDO_SCS_DOWNLOAD_RESPONSE) {  /* 0x60 */
            continue;
        }

        /* 检查 index/subindex 是否匹配 */
        uint16_t resp_idx = (uint16_t)resp[1] | ((uint16_t)resp[2] << 8);
        if (resp_idx != index || resp[3] != subindex) {
            continue;
        }

        return MRC_SUCCESS;
    }

    fprintf(stderr, "[sdo] write timeout after %d retries: node=%d "
            "idx=0x%04X sub=%d\n", retries, node_id, index, subindex);
    return MRC_ERROR_TIMEOUT;
}
