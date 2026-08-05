/**
 * @file    co_pdo.c
 * @brief   PDO (过程数据对象) 配置与收发
 *
 * RPDO 配置流程 (通过 SDO):
 *   1. 禁用 PDO: 写 0x1400+num-1 sub 1 bit31=1 (如 0x80000200)
 *   2. 清空映射: 写 0x1600+num-1 sub 0 = 0
 *   3. 设置映射: 写 0x1600+num-1 sub 1..8
 *   4. 设置映射数: 写 0x1600+num-1 sub 0 = N
 *   5. 设置传输类型: 写 0x1400+num-1 sub 2
 *   6. 启用 PDO: 写 0x1400+num-1 sub 1 = COB-ID (bit31=0)
 */

#include <stdio.h>
#include <stdlib.h>

#define PDO_CONFIG_SDO_TIMEOUT_MS 150
#include <string.h>
#include "miraculous_internal.h"

/* 校验 PDO 映射: 每项长度必须为 8 的倍数, 且总长度不超过 64 位 */
static int validate_mappings(uint8_t mapped_count, const uint32_t *mappings)
{
    uint16_t total_bits = 0;
    for (uint8_t i = 0; i < mapped_count; i++) {
        uint8_t bits = mappings[i] & 0xFF;
        if (bits == 0 || bits > 64 || (bits % 8) != 0) {
            fprintf(stderr, "[pdo] map[%d] invalid bit length %d (must be 8/16/.../64)\n",
                    i, bits);
            return MRC_ERROR_INVALID_PARAM;
        }
        total_bits += bits;
    }
    if (total_bits > 64) {
        fprintf(stderr, "[pdo] total map bit length %d exceeds 64\n", total_bits);
        return MRC_ERROR_INVALID_PARAM;
    }
    return MRC_SUCCESS;
}

int miraculous_co_pdo_rpdo_config(MiraCoMaster *co, uint8_t node_id,
                                   uint8_t pdo_num, uint32_t cob_id,
                                   uint8_t trans_type, uint16_t event_timer_ms,
                                   uint8_t mapped_count,
                                   const uint32_t *mappings)
{
    if (!co || pdo_num < 1 || pdo_num > 4 || mapped_count > 8)
        return MRC_ERROR_INVALID_PARAM;
    if (mapped_count > 0 && !mappings)
        return MRC_ERROR_INVALID_PARAM;

    int ret = validate_mappings(mapped_count, mappings);
    if (ret < 0) return ret;

    uint16_t comm_idx = 0x1400 + (pdo_num - 1); /* 通信参数索引 */
    uint16_t map_idx  = 0x1600 + (pdo_num - 1); /* 映射参数索引 */

    /* Step 1: 禁用 PDO — bit31=1 */
    uint32_t disabled_cob = cob_id | 0x80000000UL;
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x01, &disabled_cob, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) {
        fprintf(stderr, "[pdo] rpdo%d disable failed\n", pdo_num);
        return ret;
    }

    /* Step 2: 设置传输类型 */
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x02, &trans_type, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 3: 清空映射数 */
    uint8_t zero = 0;
    ret = CO_SDO_WRITE(co, node_id, map_idx, 0x00, &zero, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) {
        fprintf(stderr, "[pdo] rpdo%d clear mapping count failed\n", pdo_num);
    }

    /* Step 4: 设置映射条目 */
    for (uint8_t i = 0; i < mapped_count; i++) {
        ret = CO_SDO_WRITE(co, node_id, map_idx, 0x01 + i,
                          &mappings[i], PDO_CONFIG_SDO_TIMEOUT_MS);
        if (ret < 0) {
            fprintf(stderr, "[pdo] rpdo%d map entry %d failed\n",
                    pdo_num, i + 1);
            return ret;
        }
    }

    /* Step 5: 设置映射数 */
    ret = CO_SDO_WRITE(co, node_id, map_idx, 0x00, &mapped_count, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 6: 设置事件定时器 (sub5, 单位 ms, 0=禁用) */
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x05, &event_timer_ms, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 7: 启用 PDO — 写纯 COB-ID (bit31=0) */
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x01, &cob_id, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    printf("[pdo] rpdo%d configured: cob=0x%03X type=%d timer=%dms mappings=%d\n",
           pdo_num, cob_id, trans_type, event_timer_ms, mapped_count);
    return MRC_SUCCESS;
}

int miraculous_co_pdo_tpdo_config(MiraCoMaster *co, uint8_t node_id,
                                   uint8_t pdo_num, uint32_t cob_id,
                                   uint8_t trans_type, uint8_t inhibit_time,
                                   uint16_t event_timer_ms,
                                   uint8_t mapped_count,
                                   const uint32_t *mappings)
{
    if (!co || pdo_num < 1 || pdo_num > 4 || mapped_count > 8)
        return MRC_ERROR_INVALID_PARAM;
    if (mapped_count > 0 && !mappings)
        return MRC_ERROR_INVALID_PARAM;

    int ret = validate_mappings(mapped_count, mappings);
    if (ret < 0) return ret;

    uint16_t comm_idx = 0x1800 + (pdo_num - 1); /* TPDO 通信参数 */
    uint16_t map_idx  = 0x1A00 + (pdo_num - 1); /* TPDO 映射参数 */

    /* Step 1: 禁用 — bit31=1 */
    uint32_t disabled_cob = cob_id | 0x80000000UL;
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x01, &disabled_cob, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) { fprintf(stderr, "[pdo] tpdo%d disable failed\n", pdo_num); return ret; }

    /* Step 2: 设置传输类型 */
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x02, &trans_type, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 3: 清空映射数 */
    uint8_t zero = 0;
    ret = CO_SDO_WRITE(co, node_id, map_idx, 0x00, &zero, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 4: 设置映射条目 */
    for (uint8_t i = 0; i < mapped_count; i++) {
        ret = CO_SDO_WRITE(co, node_id, map_idx, 0x01 + i, &mappings[i], PDO_CONFIG_SDO_TIMEOUT_MS);
        if (ret < 0) { fprintf(stderr, "[pdo] tpdo%d map entry %d failed\n", pdo_num, i + 1); return ret; }
    }

    /* Step 5: 设置映射数 */
    ret = CO_SDO_WRITE(co, node_id, map_idx, 0x00, &mapped_count, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 6: 设置禁止时间 (sub3, 单位 100μs, 0=禁用) */
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x03, &inhibit_time, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 7: 设置事件定时器 (sub5, 单位 ms, 0=禁用) */
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x05, &event_timer_ms, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Step 8: 启用 — 写纯 COB-ID */
    ret = CO_SDO_WRITE(co, node_id, comm_idx, 0x01, &cob_id, PDO_CONFIG_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    printf("[pdo] tpdo%d configured: cob=0x%03X type=%d inhibit=%d timer=%dms mappings=%d\n",
           pdo_num, cob_id, trans_type, inhibit_time, event_timer_ms, mapped_count);
    return MRC_SUCCESS;
}

int miraculous_co_pdo_send(MiraCoMaster *co, uint8_t pdo_num,
                            uint32_t cob_id,
                            const uint8_t *data, uint8_t len)
{
    (void)pdo_num;
    if (!co || !data || len > 8) return MRC_ERROR_INVALID_PARAM;

    MiraCanCtx *can = miraculous_co_get_can(co);
    return miraculous_can_send(can, cob_id, data, len);
}

/* --- TPDO 回调机制 --- */

#define MAX_TPDO_CBS  32

typedef struct {
    uint8_t          node_id;
    uint8_t          pdo_num;
    uint32_t         cob_id;
    MiraTpdoCallback callback;
    void            *user_data;
} TpdoEntry_t;

typedef struct PdoCtx_t {
    TpdoEntry_t entries[MAX_TPDO_CBS];
    int         count;
    pthread_mutex_t lock;
} PdoCtx_t;

PdoCtx_t* co_pdo_create(void)
{
    PdoCtx_t *ctx = calloc(1, sizeof(PdoCtx_t));
    if (!ctx) return NULL;
    if (pthread_mutex_init(&ctx->lock, NULL) != 0) {
        free(ctx);
        return NULL;
    }
    return ctx;
}

void co_pdo_destroy(PdoCtx_t *ctx)
{
    if (!ctx) return;
    pthread_mutex_destroy(&ctx->lock);
    free(ctx);
}

int miraculous_co_pdo_set_tpdo_callback(MiraCoMaster *co,
                                         uint8_t node_id, uint8_t pdo_num,
                                         uint32_t cob_id,
                                         MiraTpdoCallback cb,
                                         void *user_data)
{
    PdoCtx_t *pdo = miraculous_co_get_pdo(co);
    if (!pdo || !cb) return MRC_ERROR_INVALID_PARAM;

    pthread_mutex_lock(&pdo->lock);
    if (pdo->count >= MAX_TPDO_CBS) {
        pthread_mutex_unlock(&pdo->lock);
        return MRC_ERROR_OUT_OF_MEMORY;
    }

    TpdoEntry_t *e = &pdo->entries[pdo->count++];
    e->node_id   = node_id;
    e->pdo_num   = pdo_num;
    e->cob_id    = cob_id;
    e->callback  = cb;
    e->user_data = user_data;

    pthread_mutex_unlock(&pdo->lock);
    return MRC_SUCCESS;
}

int miraculous_co_pdo_remove_tpdo_callbacks(MiraCoMaster *co,
                                             uint8_t node_id,
                                             void *user_data)
{
    PdoCtx_t *pdo = miraculous_co_get_pdo(co);
    if (!pdo || !user_data) return MRC_ERROR_INVALID_PARAM;

    pthread_mutex_lock(&pdo->lock);
    int write_index = 0;
    for (int read_index = 0; read_index < pdo->count; read_index++) {
        TpdoEntry_t *entry = &pdo->entries[read_index];
        if (entry->node_id == node_id && entry->user_data == user_data) {
            continue;
        }
        if (write_index != read_index) {
            pdo->entries[write_index] = *entry;
        }
        write_index++;
    }
    if (write_index < pdo->count) {
        memset(&pdo->entries[write_index], 0,
               (size_t)(pdo->count - write_index) * sizeof(TpdoEntry_t));
    }
    pdo->count = write_index;
    pthread_mutex_unlock(&pdo->lock);
    return MRC_SUCCESS;
}

void co_pdo_handle_tpdo(PdoCtx_t *ctx, uint32_t can_id,
                         const uint8_t *data, uint8_t len)
{
    if (!ctx) return;
    /* Keep the registry stable through callback completion.  Removal during
     * motor close takes the same lock, so no callback can retain a freed
     * MiraMotor user_data pointer. */
    pthread_mutex_lock(&ctx->lock);
    for (int i = 0; i < ctx->count; i++) {
        if (ctx->entries[i].cob_id == can_id
            && ctx->entries[i].callback) {
            ctx->entries[i].callback(
                ctx->entries[i].node_id,
                ctx->entries[i].pdo_num,
                data, len,
                ctx->entries[i].user_data);
        }
    }
    pthread_mutex_unlock(&ctx->lock);
}
