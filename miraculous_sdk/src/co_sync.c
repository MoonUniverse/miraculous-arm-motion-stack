/**
 * @file    co_sync.c
 * @brief   SYNC 同步帧生成器
 *
 * 使用 Linux timerfd_create() / timerfd_settime() 实现高精度周期定时。
 * 每个周期自动发送一帧 SYNC (CAN ID = 0x080, DLC = 0)。
 *
 * timerfd 的 fd 可通过 miraculous_co_sync_fd() 获取，集成到外部 epoll。
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <stdint.h>

#include "miraculous_internal.h"

/* SyncCtx_t 定义已移至 miraculous_internal.h */

SyncCtx_t* co_sync_create(void)
{
    SyncCtx_t *ctx = calloc(1, sizeof(SyncCtx_t));
    if (!ctx) return NULL;
    ctx->timer_fd = -1;
    pthread_mutex_init(&ctx->lock, NULL);
    return ctx;
}

void co_sync_destroy(SyncCtx_t *ctx)
{
    if (!ctx) return;
    pthread_mutex_lock(&ctx->lock);
    if (ctx->running && ctx->timer_fd >= 0) {
        close(ctx->timer_fd);
        ctx->timer_fd = -1;
    }
    ctx->running = false;
    ctx->refcount = 0;
    pthread_mutex_unlock(&ctx->lock);
    pthread_mutex_destroy(&ctx->lock);
    free(ctx);
}

void co_sync_set_can(SyncCtx_t *ctx, MiraCanCtx *can)
{
    ctx->can = can;
}

int miraculous_co_sync_start(MiraCoMaster *co, uint32_t period_us)
{
    SyncCtx_t *sync = miraculous_co_get_sync(co);
    if (!sync) return MRC_ERROR_NOT_INIT;

    pthread_mutex_lock(&sync->lock);

    /* 已经在运行且周期相同 — 只增加引用计数, 不重建 timerfd */
    if (sync->running && sync->period_us == period_us && sync->timer_fd >= 0) {
        sync->refcount++;
        pthread_mutex_unlock(&sync->lock);
        printf("[sync] refcount++ -> %d (period=%u us already running)\n",
               sync->refcount, period_us);
        return MRC_SUCCESS;
    }

    /* 周期不同或首次启动 — 需要重建 timerfd */
    if (sync->timer_fd >= 0) {
        close(sync->timer_fd);
        sync->timer_fd = -1;
    }

    sync->timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (sync->timer_fd < 0) {
        perror("[sync] timerfd_create");
        pthread_mutex_unlock(&sync->lock);
        return MRC_ERROR_CAN_SOCKET;
    }

    /* 设置周期 */
    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec  = period_us / 1000000;
    its.it_value.tv_nsec = (period_us % 1000000) * 1000;
    its.it_interval.tv_sec  = period_us / 1000000;
    its.it_interval.tv_nsec = (period_us % 1000000) * 1000;

    if (timerfd_settime(sync->timer_fd, 0, &its, NULL) < 0) {
        perror("[sync] timerfd_settime");
        close(sync->timer_fd);
        sync->timer_fd = -1;
        pthread_mutex_unlock(&sync->lock);
        return MRC_ERROR_CAN_SOCKET;
    }

    sync->running = true;
    sync->period_us = period_us;
    sync->refcount = 1;

    /* timerfd 由接收线程的 local epoll 统一监听，到期后调用 co_sync_handle_tick */

    pthread_mutex_unlock(&sync->lock);
    printf("[sync] started: period=%u us (fd=%d)\n",
           period_us, sync->timer_fd);
    return MRC_SUCCESS;
}

int miraculous_co_sync_stop(MiraCoMaster *co)
{
    SyncCtx_t *sync = miraculous_co_get_sync(co);
    if (!sync) return MRC_ERROR_NOT_INIT;

    pthread_mutex_lock(&sync->lock);

    if (sync->refcount <= 0) {
        pthread_mutex_unlock(&sync->lock);
        return MRC_SUCCESS; /* 已经停了 */
    }

    sync->refcount--;
    if (sync->refcount > 0) {
        pthread_mutex_unlock(&sync->lock);
        printf("[sync] refcount-- -> %d (still running)\n", sync->refcount);
        return MRC_SUCCESS;
    }

    /* 最后一个引用者 — 真正停掉 timerfd */
    sync->running = false;
    if (sync->timer_fd >= 0) {
        close(sync->timer_fd);
        sync->timer_fd = -1;
    }
    sync->period_us = 0;
    pthread_mutex_unlock(&sync->lock);
    printf("[sync] stopped\n");
    return MRC_SUCCESS;
}

int miraculous_co_sync_send_once(MiraCoMaster *co)
{
    MiraCanCtx *can = miraculous_co_get_can(co);
    if (!can) return MRC_ERROR_NOT_INIT;

    uint8_t empty = 0;
    return miraculous_can_send(can, CO_COB_SYNC, &empty, 0);
}

int miraculous_co_sync_fd(MiraCoMaster *co)
{
    SyncCtx_t *sync = miraculous_co_get_sync(co);
    return sync ? sync->timer_fd : -1;
}

/* 在接收线程中处理 timerfd 到期事件 (由 co_master.c 调用) */
void co_sync_handle_tick(SyncCtx_t *ctx)
{
    if (!ctx || ctx->timer_fd < 0 || !ctx->running) return;

    uint64_t expirations;
    ssize_t n = read(ctx->timer_fd, &expirations, sizeof(expirations));
    if (n < 0) return;

    /* 发送 SYNC 帧（如果 timerfd 因某种原因积压了多次，只发一次） */
    uint8_t empty = 0;
    miraculous_can_send(ctx->can, CO_COB_SYNC, &empty, 0);
}
