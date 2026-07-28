/**
 * @file    co_heartbeat.c
 * @brief   心跳监听 + NMT 状态跟踪
 *
 * 心跳帧格式 (CAN ID = 0x700 + NodeID, DLC = 1):
 *   Byte 0: NMT 状态 (bit 7=0) 或 心跳 (bit 7=1, bit 6-0=time ms)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "miraculous_internal.h"

#define MAX_HEARTBEAT_NODES  16
#define HEARTBEAT_TIMEOUT_MS 3000   /* 默认心跳超时 */

typedef struct {
    uint8_t              node_id;
    bool                 active;
    CoNmtState_t         state;
    MiraHeartbeatCallback callback;
    void                *user_data;

    /* 超时检测 */
    struct timespec      last_hb;
    int                  timeout_ms;
} HbNode_t;

typedef struct HbCtx_t {
    MiraCanCtx  *can;
    HbNode_t     nodes[MAX_HEARTBEAT_NODES];
    int          count;

    /* 等待状态 */
    uint8_t      waiting_node;
    CoNmtState_t waiting_state;
    bool         waiting_done;
} HbCtx_t;

/* 由 co_master.c 内部分配 */
HbCtx_t* co_heartbeat_create(void)
{
    HbCtx_t *ctx = calloc(1, sizeof(HbCtx_t));
    if (!ctx) return NULL;
    for (int i = 0; i < MAX_HEARTBEAT_NODES; i++) {
        ctx->nodes[i].timeout_ms = HEARTBEAT_TIMEOUT_MS;
    }
    return ctx;
}

void co_heartbeat_destroy(HbCtx_t *ctx)
{
    free(ctx);
}

void co_heartbeat_set_can(HbCtx_t *ctx, MiraCanCtx *can)
{
    ctx->can = can;
}

/* --- 内部：根据 CAN ID 查找/创建节点条目 --- */
static HbNode_t* hb_find_or_create(HbCtx_t *ctx, uint8_t node_id)
{
    for (int i = 0; i < ctx->count; i++) {
        if (ctx->nodes[i].node_id == node_id) return &ctx->nodes[i];
    }
    if (ctx->count >= MAX_HEARTBEAT_NODES) return NULL;
    HbNode_t *n = &ctx->nodes[ctx->count++];
    n->node_id = node_id;
    return n;
}

static HbNode_t* hb_find(HbCtx_t *ctx, uint8_t node_id)
{
    for (int i = 0; i < ctx->count; i++) {
        if (ctx->nodes[i].node_id == node_id) return &ctx->nodes[i];
    }
    return NULL;
}

/* --- 处理收到的心跳帧 --- */
void co_heartbeat_handle(HbCtx_t *ctx, uint32_t can_id,
                          const uint8_t *data, uint8_t len)
{
    if (!ctx || !data || len < 1) return;

    uint8_t node_id = can_id - CO_COB_HEARTBEAT;
    if (node_id > 127) return; /* 非法 node_id */

    HbNode_t *n = hb_find_or_create(ctx, node_id);
    if (!n) return;

    uint8_t byte0 = data[0];
    if (byte0 & 0x80) {
        /* bootup 消息 (byte0 = 0) */
        n->state  = CO_NMT_STATE_INITIALISING;
        n->active = true;
    } else {
        n->state  = (CoNmtState_t)byte0;
        n->active = true;
    }
    clock_gettime(CLOCK_MONOTONIC, &n->last_hb);

    /* 触发回调 */
    if (n->callback) {
        n->callback(node_id, n->state, n->user_data);
    }

    /* 检查等待条件 */
    if (ctx->waiting_node == node_id && ctx->waiting_state == n->state) {
        ctx->waiting_done = true;
    }
}

/* --- 公开 API --- */

int miraculous_co_heartbeat_set_callback(MiraCoMaster *co, uint8_t node_id,
                                          MiraHeartbeatCallback cb,
                                          void *user_data)
{
    extern HbCtx_t* miraculous_co_get_hb(MiraCoMaster *co);
    HbCtx_t *hb = miraculous_co_get_hb(co);
    if (!hb) return MRC_ERROR_NOT_INIT;

    HbNode_t *n = hb_find_or_create(hb, node_id);
    if (!n) return MRC_ERROR_OUT_OF_MEMORY;
    n->callback  = cb;
    n->user_data = user_data;
    return MRC_SUCCESS;
}

int miraculous_co_wait_state(MiraCoMaster *co, uint8_t node_id,
                              CoNmtState_t expected, int timeout_ms)
{
    extern HbCtx_t* miraculous_co_get_hb(MiraCoMaster *co);
    HbCtx_t *hb = miraculous_co_get_hb(co);
    if (!hb) return MRC_ERROR_NOT_INIT;

    /* 先检查当前状态 */
    HbNode_t *n = hb_find(hb, node_id);
    if (!n || !n->active) {
        /* 等一段时间让心跳到达 */
    }

    hb->waiting_node  = node_id;
    hb->waiting_state = expected;
    hb->waiting_done  = false;

    if (n && n->active && n->state == expected) {
        hb->waiting_done = true;
        return MRC_SUCCESS;
    }

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int64_t deadline_ms = timeout_ms > 0 ? (int64_t)timeout_ms
                                         : (int64_t)HEARTBEAT_TIMEOUT_MS;

    while (!hb->waiting_done) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t elapsed = (now.tv_sec - start.tv_sec) * 1000LL
                        + (now.tv_nsec - start.tv_nsec) / 1000000LL;
        if (elapsed >= deadline_ms) {
            hb->waiting_node = 0;
            return MRC_ERROR_CO_NODE_NOT_FOUND;
        }

        int remaining = (int)(deadline_ms - elapsed);
        if (remaining > 100) remaining = 100;
        usleep(remaining * 1000);  /* 接收线程会处理帧, 只需睡眠等心跳状态更新 */
    }

    hb->waiting_node = 0;
    return MRC_SUCCESS;
}

/* --- 超时检测 --- */
void co_heartbeat_check_timeouts(HbCtx_t *ctx)
{
    if (!ctx) return;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    for (int i = 0; i < ctx->count; i++) {
        HbNode_t *n = &ctx->nodes[i];
        if (!n->active) continue;

        int64_t elapsed = (now.tv_sec - n->last_hb.tv_sec) * 1000LL
                        + (now.tv_nsec - n->last_hb.tv_nsec) / 1000000LL;
        if (elapsed > n->timeout_ms) {
            n->active = false;
            n->state  = CO_NMT_STATE_DISCONNECTED;
            if (n->callback) {
                n->callback(n->node_id, CO_NMT_STATE_DISCONNECTED,
                           n->user_data);
            }
        }
    }
}
