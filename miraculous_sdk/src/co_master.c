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
#include <pthread.h>
#include <sched.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <errno.h>
#include <linux/can.h>
#include <net/if.h>

#include "miraculous_internal.h"

/*----------------------------------------------------------------------------
 * 子模块前向声明 — 均已移至 miraculous_internal.h
 *----------------------------------------------------------------------------*/

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
 * MiraCoMaster 结构体 — 定义已移至 miraculous_internal.h
 * 此处仅保留 struct tag，用于 static 函数前向声明
 *----------------------------------------------------------------------------*/

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

    } else if (base_id >= CO_COB_TPDO1 && base_id <= CO_COB_TPDO4) {
        /* TPDO 区域: 0x180-0x4FF */
        co_pdo_handle_tpdo(co->pdo, can_id, data, len);
    }
    /* 注意: SDO 响应不由回调分发, 接收线程或 recv_timeout 直接处理 */
}

/*----------------------------------------------------------------------------
 * 生命周期
 *----------------------------------------------------------------------------*/

MiraCoMaster* miraculous_co_init(MiraCanCtx *can_ctx, CiaBaudrate_t baudrate,
                                  bool start_recv_thread)
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

    co->own_can   = true;  /* co_init 只在内部调用, 始终拥有 CAN */

    /* 子模块关联 CAN */
    co_heartbeat_set_can(co->hb, can_ctx);
    co_sync_set_can(co->sync, can_ctx);

    /* 注册全局接收回调 (分发 Heartbeat/EMCY/TPDO) */
    miraculous_can_set_recv_callback(can_ctx, co_global_recv_callback, co);

    /* 初始化 SDO 队列锁 */
    pthread_mutex_init(&co->sdo_queue_lock, NULL);
    memset(co->sdo_wait_queue, 0, sizeof(co->sdo_wait_queue));

    /* 初始化每节点 SDO 锁 */
    for (int i = 0; i < 128; i++) {
        pthread_mutex_init(&co->sdo_node_lock[i], NULL);
        pthread_cond_init(&co->sdo_node_cond[i], NULL);
        co->sdo_node_busy[i] = false;
    }
    co->refcount = 1;
    co->recv_running = false;
    co->recv_stop = false;

    printf("[co_master] initialized (baudrate=%u)\n", baudrate);

    if (start_recv_thread) {
        int r = miraculous_co_recv_start(co);
        if (r < 0) {
            printf("[co_master] recv thread start failed\n");
        }
    }

    return co;
}

void miraculous_co_free(MiraCoMaster *co)
{
    if (!co) return;

    if (__atomic_sub_fetch(&co->refcount, 1, __ATOMIC_SEQ_CST) > 0) {
        /* 还有电机引用此 master, 不释放 */
        return;
    }

    /* 停止接收线程 */
    if (co->recv_running) {
        miraculous_co_recv_stop(co);
    }

    if (co->sync)  co_sync_destroy(co->sync);
    if (co->hb)    co_heartbeat_destroy(co->hb);
    if (co->pdo)   co_pdo_destroy(co->pdo);
    if (co->emcy)  co_emcy_destroy(co->emcy);

    /* 销毁每节点 SDO 锁 */
    for (int i = 0; i < 128; i++) {
        pthread_mutex_destroy(&co->sdo_node_lock[i]);
        pthread_cond_destroy(&co->sdo_node_cond[i]);
    }

    pthread_mutex_destroy(&co->sdo_queue_lock);

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
    miraculous_co_sdo_timeout_check(co);

    /* CAN 帧轮询 — 复用底层的 epoll (不自建 epoll) */
    int handled = miraculous_can_poll(co->can, timeout_ms);
    if (handled < 0) return handled;

    /* SYNC 定时器到期检查 */
    int sync_fd = miraculous_co_sync_fd(co);
    if (sync_fd >= 0) {
        struct epoll_event ev;
        if (epoll_wait(sync_fd, &ev, 1, 0) > 0) {
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
    if (co) __atomic_add_fetch(&co->refcount, 1, __ATOMIC_SEQ_CST);
}

int miraculous_co_ref_count(MiraCoMaster *co)
{
    if (!co) return 0;
    return __atomic_load_n(&co->refcount, __ATOMIC_SEQ_CST);
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

/*----------------------------------------------------------------------------
 * 接收线程 — 专用 epoll 循环, 持续读取 CAN 帧并分发
 *----------------------------------------------------------------------------*/
static void* co_recv_thread_func(void *arg)
{
    MiraCoMaster *co = (MiraCoMaster *)arg;
    if (!co) return NULL;

    int can_fd = miraculous_can_fd(co->can);
    int hb_check_counter = 0;

    /* 创建本地 epoll, 监听 can_fd + sync timerfd */
    int local_ep = epoll_create1(0);
    if (local_ep < 0) return NULL;

    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.fd = can_fd;
    if (epoll_ctl(local_ep, EPOLL_CTL_ADD, can_fd, &ev) < 0) {
        fprintf(stderr, "[recv_thread] epoll_ctl ADD can_fd failed: %s\n",
                strerror(errno));
        close(local_ep);
        return NULL;
    }

    /* 标记 fd 身份, 用于 epoll 事件区分 */
    ev.data.fd = can_fd;        /* data.fd == can_fd → CAN 帧 */
    epoll_ctl(local_ep, EPOLL_CTL_MOD, can_fd, &ev);

    co->recv_running = true;

    while (!co->recv_stop) {
        struct epoll_event events[2];
        int nfds = epoll_wait(local_ep, events, 2, 100);
        if (nfds < 0) {
            if (errno == EINTR) continue;
            break;
        }

        /* 定期心跳超时检测 (~100ms 周期) */
        hb_check_counter++;
        if (hb_check_counter >= 10) {
            hb_check_counter = 0;
            co_heartbeat_check_timeouts(co->hb);
            miraculous_co_sdo_timeout_check(co);
        }

        if (nfds == 0) continue;

        for (int i = 0; i < nfds; i++) {
            if (events[i].data.fd == can_fd) {
                /* CAN 帧到达 */
                struct can_frame frame;
                ssize_t n = read(can_fd, &frame, CAN_MTU);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EINTR) continue;
                    break;
                }

                uint32_t recv_id = frame.can_id & CAN_SFF_MASK;
                uint8_t dlc = frame.can_dlc;
                if (dlc > 8) dlc = 8;

                /* SDO 响应 → 队列匹配 */
                if (recv_id >= CO_COB_SDO_TX && recv_id <= CO_COB_SDO_TX + 127) {
                    if (miraculous_co_sdo_wait_dispatch(co, recv_id,
                                                         frame.data, dlc) == 0)
                        continue;
                }

                /* 非 SDO → 全局回调 (心跳/EMCY/TPDO) */
                co_global_recv_callback(recv_id, frame.data, dlc, co);

            } else {
                /* 非 can_fd: 检查是否是 sync timerfd */
                SyncCtx_t *sync = miraculous_co_get_sync(co);
                if (sync && sync->timer_fd == events[i].data.fd
                    && sync->running) {
                    co_sync_handle_tick(sync);
                } else {
                    /* 未知 fd, 排空以避免重复唤醒 */
                    uint64_t exp;
                    ssize_t rd = read(events[i].data.fd, &exp, sizeof(exp));
                    (void)rd;
                }
            }
        }

        /* 动态检查 sync timerfd 是否已启动/变更 */
        SyncCtx_t *sync = miraculous_co_get_sync(co);
        if (sync && sync->timer_fd >= 0 && sync->running) {
            /* 检查是否已在 epoll 中 (通过尝试 MOD, 如果失败则 ADD) */
            ev.events = EPOLLIN;
            ev.data.fd = sync->timer_fd;
            epoll_ctl(local_ep, EPOLL_CTL_MOD, sync->timer_fd, &ev);
            /* MOD 失败说明不在 epoll 中, 尝试 ADD */
            if (errno == ENOENT) {
                epoll_ctl(local_ep, EPOLL_CTL_ADD, sync->timer_fd, &ev);
            }
        }
    }

    co->recv_running = false;
    close(local_ep);
    return NULL;
}


int miraculous_co_recv_start(MiraCoMaster *co)
{
    if (!co) return MRC_ERROR_INVALID_PARAM;
    if (co->recv_running) return MRC_SUCCESS;

    co->recv_stop = false;

    /* 尝试创建线程, 优先实时调度 (SCHED_FIFO), 失败则回退到普通调度 */
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    struct sched_param sp = { .sched_priority = 30 };
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &sp);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    int ret = pthread_create(&co->recv_thread, &attr,
                              co_recv_thread_func, co);
    if (ret == EPERM) {
        pthread_attr_destroy(&attr);
        pthread_attr_init(&attr);
        ret = pthread_create(&co->recv_thread, &attr,
                              co_recv_thread_func, co);
    }
    pthread_attr_destroy(&attr);
    if (ret != 0) return MRC_ERROR_UNKNOWN;

    /* 等待线程完成初始化 (最大 1s) */
    for (int i = 0; i < 100 && !co->recv_running; i++)
        usleep(10000);
    if (!co->recv_running) {
        co->recv_stop = true;
        pthread_join(co->recv_thread, NULL);
        return MRC_ERROR_UNKNOWN;
    }
    return MRC_SUCCESS;
}

void miraculous_co_recv_stop(MiraCoMaster *co)
{
    if (!co || !co->recv_running) return;
    co->recv_stop = true;
    pthread_join(co->recv_thread, NULL);
    co->recv_stop = false;
}
