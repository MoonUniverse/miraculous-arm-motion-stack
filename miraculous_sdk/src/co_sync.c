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
#include <stdint.h>

#include "miraculous_internal.h"

typedef struct SyncCtx_t {
    MiraCanCtx *can;
    int         timer_fd;
    bool        running;
} SyncCtx_t;

SyncCtx_t* co_sync_create(void)
{
    SyncCtx_t *ctx = calloc(1, sizeof(SyncCtx_t));
    if (!ctx) return NULL;
    ctx->timer_fd = -1;
    return ctx;
}

void co_sync_destroy(SyncCtx_t *ctx)
{
    if (!ctx) return;
    miraculous_co_sync_stop(NULL); /* 内部 stop */
    if (ctx->timer_fd >= 0) close(ctx->timer_fd);
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

    if (sync->timer_fd >= 0) {
        /* 已启动，先停 */
        close(sync->timer_fd);
        sync->timer_fd = -1;
    }

    sync->timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (sync->timer_fd < 0) {
        perror("[sync] timerfd_create");
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
        return MRC_ERROR_CAN_SOCKET;
    }

    sync->running = true;
    printf("[sync] started: period=%u us (fd=%d)\n",
           period_us, sync->timer_fd);
    return MRC_SUCCESS;
}

int miraculous_co_sync_stop(MiraCoMaster *co)
{
    SyncCtx_t *sync = miraculous_co_get_sync(co);
    if (!sync) return MRC_ERROR_NOT_INIT;

    sync->running = false;
    if (sync->timer_fd >= 0) {
        close(sync->timer_fd);
        sync->timer_fd = -1;
    }
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

/* 在 poll 中处理 timerfd 到期事件 (由 co_master.c 调用) */
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
