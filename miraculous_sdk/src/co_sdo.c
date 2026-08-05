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
 *
 * 线程模型:
 *   每节点独立锁 (sdo_node_lock[node_id]), 不同节点 SDO 可并行。
 *   发送请求后等待 co->sdo_node_cond[node_id];
 *   接收线程收到 SDO 响应后 signal 该条件变量。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>

#include "miraculous_internal.h"

#define SDO_DEFAULT_TIMEOUT_MS  200

/*----------------------------------------------------------------------------
 * 内部辅助: 每节点锁等待 (接收线程模式)
 *----------------------------------------------------------------------------*/

/** 等待 SDO 响应: 锁住 node_id 的 cond, 超时返回 */
static int sdo_wait_node(MiraCoMaster *co, uint8_t node_id,
                          void *data, uint8_t *data_len,
                          int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000LL;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&co->sdo_node_lock[node_id]);

    /* 等待同步 SDO 完成 */
    while (co->sdo_node_busy[node_id]) {
        int r = pthread_cond_timedwait(&co->sdo_node_cond[node_id],
                                        &co->sdo_node_lock[node_id], &ts);
        if (r == ETIMEDOUT) {
            co->sdo_node_busy[node_id] = false;
            pthread_mutex_unlock(&co->sdo_node_lock[node_id]);
            return MRC_ERROR_TIMEOUT;
        }
    }

    /* 检查 SDO 是否被从站拒绝 (Abort, SCS=0x80) */
    uint8_t scs = co->sdo_node_rx_buf[node_id][0];
    if (scs == CO_SDO_SCS_ABORT) {
        uint32_t abort_code = (uint32_t)co->sdo_node_rx_buf[node_id][4]
                            | ((uint32_t)co->sdo_node_rx_buf[node_id][5] << 8)
                            | ((uint32_t)co->sdo_node_rx_buf[node_id][6] << 16)
                            | ((uint32_t)co->sdo_node_rx_buf[node_id][7] << 24);
        pthread_mutex_unlock(&co->sdo_node_lock[node_id]);
        fprintf(stderr, "[sdo] node=%d abort: idx=0x%04X sub=0x%02X code=0x%08X\n",
                node_id,
                (uint16_t)(co->sdo_node_rx_buf[node_id][1]
                         | (co->sdo_node_rx_buf[node_id][2] << 8)),
                co->sdo_node_rx_buf[node_id][3],
                abort_code);
        return MRC_ERROR_CO_SDO_ABORT;
    }

    /* 读取响应数据 — SDO Expedited Upload 响应中, 实际数据在字节 4~7 */
    if (data && data_len) {
        uint8_t copy_len = co->sdo_node_rx_len[node_id];
        if (copy_len > 4) copy_len -= 4; else copy_len = 0;  /* 跳过 cmd+idx+sub */
        if (copy_len > *data_len) copy_len = *data_len;
        if (copy_len > 0)
            memcpy(data, co->sdo_node_rx_buf[node_id] + 4, copy_len);
        *data_len = copy_len;
    }

    pthread_mutex_unlock(&co->sdo_node_lock[node_id]);
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * SDO Expedited Upload (读)
 *----------------------------------------------------------------------------*/

int miraculous_co_sdo_read(MiraCoMaster *co, uint8_t node_id,
                            uint16_t index, uint8_t subindex,
                            void *data, uint8_t *data_len, int timeout_ms)
{
    if (!co || !data || !data_len || *data_len == 0)
        return MRC_ERROR_INVALID_PARAM;
    if (!__atomic_load_n(&co->recv_running, __ATOMIC_ACQUIRE))
        return MRC_ERROR_NOT_INIT;

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

    /* 发送请求, 等待响应 (每节点锁) */
    pthread_mutex_lock(&co->sdo_node_lock[node_id]);
    co->sdo_node_busy[node_id] = true;
    co->sdo_node_pending_idx[node_id] = index;
    co->sdo_node_pending_sub[node_id] = subindex;
    pthread_mutex_unlock(&co->sdo_node_lock[node_id]);

    int ret = miraculous_can_send(can, CO_COB_SDO_RX + node_id, req, 8);
    if (ret < 0) {
        pthread_mutex_lock(&co->sdo_node_lock[node_id]);
        co->sdo_node_busy[node_id] = false;
        pthread_cond_signal(&co->sdo_node_cond[node_id]);
        pthread_mutex_unlock(&co->sdo_node_lock[node_id]);
        return ret;
    }

    return sdo_wait_node(co, node_id, data, data_len, timeout_ms);
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
    if (!__atomic_load_n(&co->recv_running, __ATOMIC_ACQUIRE))
        return MRC_ERROR_NOT_INIT;

    MiraCanCtx *can = miraculous_co_get_can(co);
    if (!can) return MRC_ERROR_NOT_INIT;

    if (timeout_ms <= 0) timeout_ms = SDO_DEFAULT_TIMEOUT_MS;

    uint8_t req[8];
    memset(req, 0, 8);
    req[0] = CO_SDO_CCS_DOWNLOAD_INITIATE | CO_SDO_EXPEDITED
           | CO_SDO_SIZE_INDICATED;
    req[0] |= (uint8_t)((4 - data_len) << 2);
    req[1] = (uint8_t)(index & 0xFF);
    req[2] = (uint8_t)((index >> 8) & 0xFF);
    req[3] = subindex;
    memcpy(&req[4], data, data_len);

    /* 发送请求, 等待响应 (每节点锁) */
    pthread_mutex_lock(&co->sdo_node_lock[node_id]);
    co->sdo_node_busy[node_id] = true;
    co->sdo_node_pending_idx[node_id] = index;
    co->sdo_node_pending_sub[node_id] = subindex;
    pthread_mutex_unlock(&co->sdo_node_lock[node_id]);

    int ret = miraculous_can_send(can, CO_COB_SDO_RX + node_id, req, 8);
    if (ret < 0) {
        pthread_mutex_lock(&co->sdo_node_lock[node_id]);
        co->sdo_node_busy[node_id] = false;
        pthread_cond_signal(&co->sdo_node_cond[node_id]);
        pthread_mutex_unlock(&co->sdo_node_lock[node_id]);
        return ret;
    }

    uint8_t dummy;
    uint8_t dl = 0;
    return sdo_wait_node(co, node_id, &dummy, &dl, timeout_ms);
}

/*----------------------------------------------------------------------------
 * SDO 响应分发 — 由接收线程调用
 * 匹配 SDO 响应并唤醒等待的同步/异步 SDO 调用者
 *----------------------------------------------------------------------------*/

int miraculous_co_sdo_wait_dispatch(MiraCoMaster *co, uint32_t can_id,
                                      const uint8_t *data, uint8_t len)
{
    if (!co || !data) return -1;

    uint8_t node_id = can_id & 0x7F;
    if (node_id > 127) return -1;

    /* 解析响应帧中的 index/subindex 用于匹配 */
    uint16_t resp_idx = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    uint8_t  resp_sub = data[3];

    pthread_mutex_lock(&co->sdo_node_lock[node_id]);

    if (!co->sdo_node_busy[node_id]) {
        /* 无等待中的同步 SDO — 尝试匹配等待队列 */
        pthread_mutex_unlock(&co->sdo_node_lock[node_id]);

        pthread_mutex_lock(&co->sdo_queue_lock);
        for (int i = 0; i < SDO_WAIT_QUEUE_MAX; i++) {
            SdoWaitEntry_t *e = &co->sdo_wait_queue[i];
            if (!e->in_use || e->done) continue;
            if (e->resp_can_id != can_id) continue;

            uint16_t resp_idx = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            if (resp_idx != e->index || data[3] != e->subindex) continue;

            uint8_t copy_len = len > 8 ? 8 : len;
            if (e->data_out && copy_len > 0)
                memcpy(e->data_out, data, copy_len);
            if (e->len_out)
                *e->len_out = copy_len;
            e->done = true;
            e->result = MRC_SUCCESS;
            pthread_cond_signal(&e->cond);
            pthread_mutex_unlock(&co->sdo_queue_lock);
            return 0;
        }
        pthread_mutex_unlock(&co->sdo_queue_lock);

        /* 尝试异步 SDO 任务匹配 */
        pthread_mutex_lock(&co->motor_registry_lock);
        MiraMotor *mot = co->motor_by_node[node_id];
        if (mot) {
            uint16_t resp_idx = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            pthread_mutex_lock(&mot->sdo_async_lock);
            MiraSdoAsyncTask **p = &mot->sdo_async_list;
            while (*p) {
                MiraSdoAsyncTask *t = *p;
                if (t->index == resp_idx && t->subindex == data[3]) {
                    *p = t->next;
                    MiraSdoCallback cb = t->cb;
                    void *usr = t->user_data;
                    int tid = t->tid;
                    free(t);
                    pthread_mutex_unlock(&mot->sdo_async_lock);
                    if (cb)
                        cb(mot, tid, resp_idx, data[3],
                           MIRA_SDO_OK, data, len, usr);
                    pthread_mutex_unlock(&co->motor_registry_lock);
                    return 0;
                }
                p = &(*p)->next;
            }
            pthread_mutex_unlock(&mot->sdo_async_lock);
        }
        pthread_mutex_unlock(&co->motor_registry_lock);
        return -1;
    }

    /* 有等待中的同步 SDO — 校验 index/subindex 匹配 */
    if (resp_idx != co->sdo_node_pending_idx[node_id]
        || resp_sub != co->sdo_node_pending_sub[node_id]) {
        /* 响应与当前请求不匹配 (可能是超时后的迟到响应), 丢弃 */
        pthread_mutex_unlock(&co->sdo_node_lock[node_id]);
        return -1;
    }

    uint8_t copy_len = len > 8 ? 8 : len;
    memcpy(co->sdo_node_rx_buf[node_id], data, copy_len);
    co->sdo_node_rx_len[node_id] = copy_len;
    co->sdo_node_busy[node_id] = false;

    pthread_cond_signal(&co->sdo_node_cond[node_id]);
    pthread_mutex_unlock(&co->sdo_node_lock[node_id]);
    return 0;
}

/*----------------------------------------------------------------------------
 * 异步 SDO 超时检测
 *----------------------------------------------------------------------------*/

void miraculous_co_sdo_timeout_check(MiraCoMaster *co)
{
    if (!co) return;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t now_ms = now.tv_sec * 1000ULL + now.tv_nsec / 1000000ULL;

    pthread_mutex_lock(&co->motor_registry_lock);
    for (int nid = 1; nid <= 127; nid++) {
        MiraMotor *mot = co->motor_by_node[nid];
        if (!mot) continue;

        pthread_mutex_lock(&mot->sdo_async_lock);
        MiraSdoAsyncTask **p = &mot->sdo_async_list;
        while (*p) {
            MiraSdoAsyncTask *t = *p;
            uint64_t exp_ms = t->expire_ts.tv_sec * 1000ULL
                            + t->expire_ts.tv_nsec / 1000000ULL;
            if (now_ms >= exp_ms) {
                *p = t->next;
                MiraSdoCallback cb = t->cb;
                void *usr = t->user_data;
                int tid = t->tid;
                uint16_t idx = t->index;
                uint8_t sub = t->subindex;
                free(t);
                pthread_mutex_unlock(&mot->sdo_async_lock);
                if (cb)
                    cb(mot, tid, idx, sub,
                       MIRA_SDO_TIMEOUT, NULL, 0, usr);
                pthread_mutex_lock(&mot->sdo_async_lock);
                p = &mot->sdo_async_list;
                continue;
            }
            p = &(*p)->next;
        }
        pthread_mutex_unlock(&mot->sdo_async_lock);
    }
    pthread_mutex_unlock(&co->motor_registry_lock);
}
