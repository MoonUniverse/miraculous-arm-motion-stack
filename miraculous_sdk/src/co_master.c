/**
 * @file    co_master.c
 * @brief   CANopen 主站聚合层 — 将所有 CANopen 子模块组合并驱动
 *
 * 架构:
 *   MiraCoMaster
 *   ├── MiraCanCtx       (传输层)
 *   ├── HbCtx_t           (心跳监控)
 *   ├── PdoCtx_t          (PDO 收发)
 *   ├── SyncCtx_t         (SYNC 生成)
 *   ├── EmcyCtx_t         (EMCY 监听)
 *   └── 全局 CAN 帧分发逻辑
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <net/if.h>

#include "miraculous_internal.h"

/*----------------------------------------------------------------------------
 * 子模块前向声明
 *----------------------------------------------------------------------------*/

typedef struct HbCtx_t   HbCtx_t;
typedef struct PdoCtx_t  PdoCtx_t;
typedef struct SyncCtx_t SyncCtx_t;
typedef struct EmcyCtx_t EmcyCtx_t;

/* 子模块 create/destroy/accessor */
HbCtx_t*   co_heartbeat_create(void);
void       co_heartbeat_destroy(HbCtx_t *ctx);
void       co_heartbeat_set_can(HbCtx_t *ctx, MiraCanCtx *can);
void       co_heartbeat_handle(HbCtx_t *ctx, uint32_t can_id,
                                const uint8_t *data, uint8_t len);
void       co_heartbeat_check_timeouts(HbCtx_t *ctx);

PdoCtx_t*  co_pdo_create(void);
void       co_pdo_destroy(PdoCtx_t *ctx);
void       co_pdo_handle_tpdo(PdoCtx_t *ctx, uint32_t can_id,
                               const uint8_t *data, uint8_t len);

SyncCtx_t* co_sync_create(void);
void       co_sync_destroy(SyncCtx_t *ctx);
void       co_sync_set_can(SyncCtx_t *ctx, MiraCanCtx *can);
void       co_sync_handle_tick(SyncCtx_t *ctx);

EmcyCtx_t* co_emcy_create(void);
void       co_emcy_destroy(EmcyCtx_t *ctx);
void       co_emcy_handle(EmcyCtx_t *ctx, uint32_t can_id,
                           const uint8_t *data, uint8_t len);

/*----------------------------------------------------------------------------
 * MiraCoMaster 结构体
 *----------------------------------------------------------------------------*/

struct MiraCoMaster {
    MiraCanCtx  *can;
    HbCtx_t     *hb;
    PdoCtx_t    *pdo;
    SyncCtx_t   *sync;
    EmcyCtx_t   *emcy;

    int          epoll_fd;   /* epoll 集成: 同步定时器 fd */
    bool         own_can;    /* 是否拥有 CAN 生命周期 */
    int          refcount;   /* 引用计数, 共享同一总线的电机数 */
};

/*----------------------------------------------------------------------------
 * 内部访问器 (供各子模块使用)
 *----------------------------------------------------------------------------*/

MiraCanCtx* miraculous_co_get_can(MiraCoMaster *co)
{
    return co ? co->can : NULL;
}

HbCtx_t* miraculous_co_get_hb(MiraCoMaster *co)
{
    return co ? co->hb : NULL;
}

PdoCtx_t* miraculous_co_get_pdo(MiraCoMaster *co)
{
    return co ? co->pdo : NULL;
}

SyncCtx_t* miraculous_co_get_sync(MiraCoMaster *co)
{
    return co ? co->sync : NULL;
}

EmcyCtx_t* miraculous_co_get_emcy(MiraCoMaster *co)
{
    return co ? co->emcy : NULL;
}

/*----------------------------------------------------------------------------
 * 接收回调：全局 CAN 帧分发
 *----------------------------------------------------------------------------*/

static void co_global_recv_callback(uint32_t can_id, const uint8_t *data,
                                     uint8_t len, void *user_data)
{
    MiraCoMaster *co = (MiraCoMaster *)user_data;
    if (!co) return;

    uint32_t base_id = can_id & 0xFF80; /* 保留高 4 位，屏蔽低 7 位 */

    /* 判断 CAN ID 类别并分发 */
    if (can_id >= CO_COB_HEARTBEAT && can_id <= CO_COB_HEARTBEAT + 127) {
        /* 0x700-0x77F: Heartbeat */
        co_heartbeat_handle(co->hb, can_id, data, len);

    } else if (can_id >= CO_COB_EMCY_BASE && can_id <= CO_COB_EMCY_BASE + 127) {
        /* 0x080-0x0FF: EMCY (与 SYNC 冲突，区分: DLC>0 = EMCY) */
        if (len > 0) {
            co_emcy_handle(co->emcy, can_id, data, len);
        }

    } else if (base_id >= CO_COB_TPDO1 && base_id <= CO_COB_TPDO4 + 127 * 4) {
        /* TPDO 区域: 0x180-0x57F */
        co_pdo_handle_tpdo(co->pdo, can_id, data, len);
    }
    /* 其他帧（如 SDO 响应）由调用层的 recv_timeout() 直接处理 */
}

/*----------------------------------------------------------------------------
 * 生命周期
 *----------------------------------------------------------------------------*/

MiraCoMaster* miraculous_co_init(MiraCanCtx *can_ctx, CiaBaudrate_t baudrate)
{
    if (!can_ctx) return NULL;

    /* 如需配置主机波特率，在启动 CANopen 之前设置 */
    if (baudrate > 0) {
        int ret = miraculous_can_set_bitrate(can_ctx, baudrate);
        if (ret < 0) {
            fprintf(stderr, "[co_master] set bitrate %u failed\n", baudrate);
            return NULL;
        }
    }

    MiraCoMaster *co = calloc(1, sizeof(MiraCoMaster));
    if (!co) return NULL;

    co->can = can_ctx;

    /* 创建子模块 */
    co->hb   = co_heartbeat_create();
    co->pdo  = co_pdo_create();
    co->sync = co_sync_create();
    co->emcy = co_emcy_create();

    if (!co->hb || !co->pdo || !co->sync || !co->emcy) {
        fprintf(stderr, "[co_master] failed to allocate sub-modules\n");
        miraculous_co_free(co);
        return NULL;
    }

    co->own_can = false;    /* 用户提供的 CAN, 不由 master 清理 */

    /* 子模块关联 CAN */
    co_heartbeat_set_can(co->hb, can_ctx);
    co_sync_set_can(co->sync, can_ctx);

    /* 注册全局接收回调 (分发 Heartbeat/EMCY/TPDO) */
    miraculous_can_set_recv_callback(can_ctx, co_global_recv_callback, co);

    co->own_can   = true;  /* co_init 只在内部调用, 始终拥有 CAN */
    co->refcount = 1;

    printf("[co_master] initialized (baudrate=%u)\n", baudrate);
    return co;
}

void miraculous_co_free(MiraCoMaster *co)
{
    if (!co) return;

    co->refcount--;
    if (co->refcount > 0) {
        /* 还有电机引用此 master, 不释放 */
        return;
    }

    if (co->sync)  co_sync_destroy(co->sync);
    if (co->hb)    co_heartbeat_destroy(co->hb);
    if (co->pdo)   co_pdo_destroy(co->pdo);
    if (co->emcy)  co_emcy_destroy(co->emcy);

    if (co->own_can && co->can) {
        miraculous_can_close(co->can);
    }

    free(co);
    printf("[co_master] freed\n");
}

/*----------------------------------------------------------------------------
 * 主轮询
 *----------------------------------------------------------------------------*/

int miraculous_co_poll(MiraCoMaster *co, int timeout_ms)
{
    if (!co) return MRC_ERROR_NOT_INIT;

    /* 先做心跳超时检测 */
    co_heartbeat_check_timeouts(co->hb);

    /* epoll 等待 CAN 帧 + SYNC timer */
    int can_fd   = miraculous_can_fd(co->can);
    int sync_fd  = miraculous_co_sync_fd(co);

    struct epoll_event events[4];
    int epoll_fd = epoll_create1(0);
    if (epoll_fd < 0) {
        /* 回退到只有 CAN 的轮询 */
        return miraculous_can_poll(co->can, timeout_ms);
    }

    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.fd = can_fd;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, can_fd, &ev);

    if (sync_fd >= 0) {
        ev.data.fd = sync_fd;
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sync_fd, &ev);
    }

    int nfds = epoll_wait(epoll_fd, events, 4, timeout_ms);
    close(epoll_fd);

    if (nfds < 0) {
        return MRC_ERROR_CAN_RECV;
    }

    int handled = 0;
    for (int i = 0; i < nfds; i++) {
        if (events[i].data.fd == can_fd) {
            /* CAN 帧到达 — 调 can poll 处理（会触发回调） */
            int r = miraculous_can_poll(co->can, 0);
            if (r > 0) handled += r;
        } else if (events[i].data.fd == sync_fd && sync_fd >= 0) {
            /* SYNC 定时器到期 */
            co_sync_handle_tick(co->sync);
            handled++;
        }
    }

    return handled;
}

/*----------------------------------------------------------------------------
 * 引用计数访问器 (供 motion_state.c 使用)
 *----------------------------------------------------------------------------*/

void miraculous_co_ref_inc(MiraCoMaster *co)
{
    if (co) co->refcount++;
}

int miraculous_co_ref_count(MiraCoMaster *co)
{
    return co ? co->refcount : 0;
}

/*----------------------------------------------------------------------------
 * Bootstrap
 *----------------------------------------------------------------------------*/

int miraculous_co_bootstrap(MiraCoMaster *co, uint8_t node_id,
                             int timeout_ms)
{
    if (!co || node_id == 0 || node_id > 127)
        return MRC_ERROR_INVALID_PARAM;

    printf("[bootstrap] node %d starting...\n", node_id);

    /* NMT Start 将节点置为 Operational。
     * 注意: 不发送 NMT Reset —— 那会导致节点 CAN 控制器重启,
     * 进而使主机 CAN 控制器进入 bus-off/TX-full 状态,
     * 需要 sudo 重启接口才能恢复。
     * 如果节点不在 Operational, NMT Start 自动将其转入;
     * 如果节点已 Operational, NMT Start 是空操作。
     */
    printf("[bootstrap] sending NMT start to node %d...\n", node_id);
    int ret = miraculous_co_nmt_start(co, node_id);
    if (ret < 0) {
        fprintf(stderr, "[bootstrap] nmt start failed: %s\n",
                mrc_strerror(ret));
        fprintf(stderr, "[bootstrap] check that node %d is powered and "
                "on the CAN bus\n", node_id);
        return ret;
    }

    /* 等待节点就绪: poll 一段时间, 处理收到的帧 (心跳/TPDO 等),
     * 同时给节点足够时间完成启动。
     * 不依赖心跳判断状态, 因为节点可能未启用心跳(0x1017=0)。 */
    int poll_time = timeout_ms > 0 ? timeout_ms : 200;
    int remained = poll_time;
    while (remained > 0) {
        int step = remained > 50 ? 50 : remained;
        int n = miraculous_co_poll(co, step);
        if (n < 0) return n;
        remained -= step;
    }

    printf("[bootstrap] node %d started.\n", node_id);
    return MRC_SUCCESS;
}
