/**
 * @file    can_socket.c
 * @brief   SocketCAN 传输层实现
 *
 * 使用 Linux SocketCAN (PF_CAN, CAN_RAW) 进行 CAN 帧收发。
 * 提供 epoll 事件循环、回调分发、超时接收等机制。
 *
 * 依赖: Linux 内核 >= 3.6 (CAN_RAW), glibc
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>

#include "miraculous_internal.h"


/*----------------------------------------------------------------------------
 * 内部数据结构
 *----------------------------------------------------------------------------*/

#define MAX_EPOLL_EVENTS  16
#define MAX_CALLBACKS      8

typedef struct {
    uint32_t               can_id;
    MiraCanRecvCallback    callback;
    void                  *user_data;
} CanCallbackEntry_t;

struct MiraCanCtx {
    int              sock_fd;                        /* SocketCAN fd */
    int              epoll_fd;                       /* epoll fd */
    pthread_mutex_t  lock;                           /* 收发互斥锁 */
    char             ifname[IFNAMSIZ];               /* 接口名 */
    bool             running;

    /* 回调表 */
    CanCallbackEntry_t  callbacks[MAX_CALLBACKS];
    int              callback_count;
    MiraCanRecvCallback global_callback;             /* 全局回调 (匹配所有帧) */
    void            *global_user_data;

    /* 超时接收 */
    bool             waiting_timeout;
    uint32_t         wait_can_id;
    uint8_t         *wait_data;
    uint8_t         *wait_len;
    int              wait_result;
};

/*----------------------------------------------------------------------------
 * 内部辅助
 *----------------------------------------------------------------------------*/

/** 确保 CAN 接口为 up 状态 (已 up 则跳过, 非 root 也可用) */
static int can_if_up(const char *ifname)
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) return -1;

    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        close(sock);
        return -1;
    }
    /* 如果已 up, 不需要写 ioctl (非 root 用户也可用) */
    if (ifr.ifr_flags & IFF_UP) {
        close(sock);
        return 0;
    }
    ifr.ifr_flags |= IFF_UP;
    if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
        close(sock);
        /* 告诉用户接口存在但需要 sudo 配置 */
        fprintf(stderr, "[can_socket] interface %s exists but is DOWN.\n"
                "  Run once: sudo ip link set %s type can bitrate 1000000\n"
                "            sudo ip link set up %s\n", ifname, ifname, ifname);
        return -1;
    }
    close(sock);
    return 0;
}

/** 通过 ip 命令配置 CAN 接口波特率 (接口名级别) */
static int set_bitrate_by_name(const char *ifname, uint16_t baudrate)
{
    /* 校验波特率 */
    switch (baudrate) {
    case 50: case 100: case 125:
    case 250: case 500: case 800: case 1000:
        break;
    default:
        return MRC_ERROR_INVALID_PARAM;
    }

    char cmd[256];
    int n = snprintf(cmd, sizeof(cmd),
                     "ip link set %s down 2>/dev/null && "
                     "ip link set %s type can bitrate %u 2>/dev/null && "
                     "ip link set %s up 2>/dev/null",
                     ifname, ifname, (unsigned)(baudrate * 1000), ifname);
    if (n < 0 || (size_t)n >= sizeof(cmd)) {
        return MRC_ERROR_INVALID_PARAM;
    }

    int ret = system(cmd);
    if (ret != 0) {
        fprintf(stderr, "[can_socket] failed to set bitrate %u on %s "
                "(ret=%d)\n", baudrate, ifname, ret);
        return MRC_ERROR_CAN_IOCTL;
    }

    printf("[can_socket] bitrate set to %u kbps on %s\n",
           baudrate, ifname);
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 公开 API
 *----------------------------------------------------------------------------*/

MiraCanCtx* miraculous_can_open(const char *ifname, CiaBaudrate_t baudrate)
{
    if (!ifname || !*ifname) return NULL;

    /* 如需配置波特率，在创建 socket 之前设置硬件 */
    if (baudrate > 0) {
        int ret = set_bitrate_by_name(ifname, baudrate);
        if (ret < 0) return NULL;
    }

    /* 确保接口 up */
    if (can_if_up(ifname) < 0) {
        fprintf(stderr, "[can_socket] failed to bring up interface %s: %s\n",
                ifname, strerror(errno));
        return NULL;
    }

    MiraCanCtx *ctx = calloc(1, sizeof(MiraCanCtx));
    if (!ctx) return NULL;

    strncpy(ctx->ifname, ifname, IFNAMSIZ - 1);
    ctx->sock_fd = -1;
    ctx->epoll_fd = -1;

    /* --- 创建 CAN RAW socket --- */
    ctx->sock_fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (ctx->sock_fd < 0) {
        perror("[can_socket] socket(PF_CAN, SOCK_RAW)");
        free(ctx);
        return NULL;
    }

    /* --- 关闭回环: 不接收自己发送的帧 --- */
    int no_loopback = 0;
    setsockopt(ctx->sock_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
               &no_loopback, sizeof(no_loopback));

    /* --- 绑定到接口 --- */
    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(ctx->sock_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("[can_socket] ioctl(SIOCGIFINDEX)");
        close(ctx->sock_fd);
        free(ctx);
        return NULL;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(ctx->sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[can_socket] bind()");
        close(ctx->sock_fd);
        free(ctx);
        return NULL;
    }

    /* --- 内核级 CAN ID 过滤 --- */
    /* 掩码 0x780: 保留 bit[10:7] (高 4 位), 匹配 128 个节点 ID (1~127) */
    struct can_filter filters[] = {
        { 0x000, 0x780 },  /* NMT */
        { 0x080, 0x780 },  /* EMCY 0x080-0x0FF */
        { 0x180, 0x780 },  /* TPDO1 0x180-0x1FF */
        { 0x280, 0x780 },  /* TPDO2 0x280-0x2FF */
        { 0x380, 0x780 },  /* TPDO3 0x380-0x3FF */
        { 0x480, 0x780 },  /* TPDO4 0x480-0x4FF */
        { 0x580, 0x780 },  /* SDO 响应 0x580-0x5FF */
        { 0x700, 0x780 },  /* Heartbeat 0x700-0x77F */
    };
    setsockopt(ctx->sock_fd, SOL_CAN_RAW, CAN_RAW_FILTER,
               filters, sizeof(filters));

    /* --- 创建 epoll --- */
    ctx->epoll_fd = epoll_create1(0);
    if (ctx->epoll_fd < 0) {
        perror("[can_socket] epoll_create1()");
        close(ctx->sock_fd);
        free(ctx);
        return NULL;
    }

    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.ptr = NULL;
    if (epoll_ctl(ctx->epoll_fd, EPOLL_CTL_ADD, ctx->sock_fd, &ev) < 0) {
        perror("[can_socket] epoll_ctl(ADD)");
        close(ctx->epoll_fd);
        close(ctx->sock_fd);
        free(ctx);
        return NULL;
    }

    ctx->running = true;
    pthread_mutex_init(&ctx->lock, NULL);
    printf("[can_socket] opened %s (fd=%d)\n", ifname, ctx->sock_fd);
    return ctx;
}

void miraculous_can_close(MiraCanCtx *ctx)
{
    if (!ctx) return;
    ctx->running = false;

    if (ctx->epoll_fd >= 0) close(ctx->epoll_fd);
    if (ctx->sock_fd >= 0) close(ctx->sock_fd);
    pthread_mutex_destroy(&ctx->lock);
    printf("[can_socket] closed %s\n", ctx->ifname);
    free(ctx);
}

int miraculous_can_fd(MiraCanCtx *ctx)
{
    return ctx ? ctx->sock_fd : -1;
}

int miraculous_can_get_epoll_fd(MiraCanCtx *ctx)
{
    return ctx ? ctx->epoll_fd : -1;
}

/*----------------------------------------------------------------------------
 * 发送
 *----------------------------------------------------------------------------*/

int miraculous_can_send(MiraCanCtx *ctx, uint32_t can_id,
                        const uint8_t *data, uint8_t len)
{
    if (!ctx || !data || len > 8) return MRC_ERROR_INVALID_PARAM;
    if (!ctx->running) return MRC_ERROR_NOT_INIT;

    
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = can_id & CAN_SFF_MASK;  /* 11-bit 标准帧 */
    frame.can_dlc = len;
    if (len > 0) memcpy(frame.data, data, len);
    pthread_mutex_lock(&ctx->lock);
    int n = write(ctx->sock_fd, &frame, sizeof(frame));
    pthread_mutex_unlock(&ctx->lock);
    if (n < 0) {
        if (errno == ENOBUFS || errno == EAGAIN) {
            return MRC_ERROR_CAN_TX_FULL;
        }
        perror("[can_socket] write()");
        return MRC_ERROR_CAN_SEND;
    }
    if ((size_t)n != sizeof(frame)) {
        return MRC_ERROR_CAN_SEND;
    }
    return MRC_SUCCESS;
}

int miraculous_can_send_frame(MiraCanCtx *ctx, const MiraCanFrame_t *frame)
{
    if (!ctx || !frame) return MRC_ERROR_INVALID_PARAM;
    if (!ctx->running) return MRC_ERROR_NOT_INIT;

    uint8_t len = frame->can_dlc;
    if (len > 8) len = 8;

    struct can_frame cf;
    memset(&cf, 0, sizeof(cf));
    cf.can_id  = frame->can_id & CAN_SFF_MASK;
    cf.can_dlc = len;
    memcpy(cf.data, frame->data, len);

    pthread_mutex_lock(&ctx->lock);
    int n = write(ctx->sock_fd, &cf, sizeof(cf));
    pthread_mutex_unlock(&ctx->lock);
    if (n < 0) {
        if (errno == ENOBUFS || errno == EAGAIN)
            return MRC_ERROR_CAN_TX_FULL;
        return MRC_ERROR_CAN_SEND;
    }
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 接收回调机制
 *----------------------------------------------------------------------------*/

int miraculous_can_set_recv_callback(MiraCanCtx *ctx,
                                      MiraCanRecvCallback cb,
                                      void *user_data)
{
    if (!ctx || !cb) return MRC_ERROR_INVALID_PARAM;

    /* 简单策略: 优先使用全局回调 */
    ctx->global_callback  = cb;
    ctx->global_user_data = user_data;
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 超时接收 (内部辅助)
 *----------------------------------------------------------------------------*/

int miraculous_can_recv_timeout(MiraCanCtx *ctx, uint32_t can_id,
                                 uint8_t *data_out, uint8_t *len_out,
                                 int timeout_ms)
{
    if (!ctx || !data_out || !len_out) return MRC_ERROR_INVALID_PARAM;
    if (!ctx->running) return MRC_ERROR_NOT_INIT;

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int64_t deadline_ms = (int64_t)timeout_ms;
    if (timeout_ms == 0) deadline_ms = INT64_MAX; /* 无超时 */

    while (1) {
        /* 计算剩余超时 */
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t elapsed = (now.tv_sec - start.tv_sec) * 1000LL
                        + (now.tv_nsec - start.tv_nsec) / 1000000LL;
        int64_t remaining = deadline_ms - elapsed;
        if (remaining <= 0) {
            return MRC_ERROR_TIMEOUT;
        }
        if (remaining > INT_MAX) remaining = INT_MAX;

        /* epoll 等待 (不持锁, epoll_wait 不会与 send 冲突) */
        struct epoll_event events[1];
        int nfds = epoll_wait(ctx->epoll_fd, events, 1, (int)remaining);
        if (nfds < 0) {
            if (errno == EINTR) continue;
            return MRC_ERROR_CAN_RECV;
        }
        if (nfds == 0) {
            return MRC_ERROR_TIMEOUT;
        }

        /* 读取数据 (不持锁, read 与 write 不冲突) */
        struct can_frame frame;
        ssize_t n = read(ctx->sock_fd, &frame, CAN_MTU);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return MRC_ERROR_CAN_RECV;
        }

        uint32_t recv_id = frame.can_id & CAN_SFF_MASK;
        uint8_t dlc = frame.can_dlc;
        if (dlc > 8) dlc = 8;

        if (recv_id != can_id) {
            /* 不匹配的帧分发给全局回调（更新 TPDO 缓存等） */
            /* 注意: 回调内可能调 can_send, 所以绝不能在 ctx->lock 内调 */
            if (ctx->global_callback) {
                ctx->global_callback(recv_id, frame.data, dlc,
                                     ctx->global_user_data);
            }
            continue;
        }

        /* 匹配成功 */
        if (dlc > *len_out) dlc = *len_out;
        memcpy(data_out, frame.data, dlc);
        *len_out = dlc;
        return MRC_SUCCESS;
    }
}

/*----------------------------------------------------------------------------
 * 波特率配置
 *----------------------------------------------------------------------------*/

int miraculous_can_set_bitrate(MiraCanCtx *ctx, CiaBaudrate_t baudrate)
{
    if (!ctx) return MRC_ERROR_INVALID_PARAM;
    return set_bitrate_by_name(ctx->ifname, baudrate);
}

int miraculous_can_poll(MiraCanCtx *ctx, int timeout_ms)
{
    if (!ctx || !ctx->running) return MRC_ERROR_NOT_INIT;

    /* Phase 1: epoll_wait + 批量读取 (不持锁, 仅 read 不与 send 冲突) */
    struct epoll_event events[MAX_EPOLL_EVENTS];
    int nfds = epoll_wait(ctx->epoll_fd, events, MAX_EPOLL_EVENTS, timeout_ms);
    if (nfds < 0) {
        if (errno == EINTR) return 0;
        return MRC_ERROR_CAN_RECV;
    }

    /* 收集帧到栈缓冲区, 避免在锁内做任何可能阻塞的操作 */
    struct { uint32_t id; uint8_t dlc; uint8_t data[8]; } batch[64];
    int count = 0;

    for (int i = 0; i < nfds && count < 64; i++) {
        if (!(events[i].events & EPOLLIN)) continue;
        while (count < 64) {
            struct can_frame frame;
            ssize_t n = read(ctx->sock_fd, &frame, CAN_MTU);
            if (n < 0) {
                if (errno == EAGAIN || errno == EINTR) break;
                return MRC_ERROR_CAN_RECV;
            }
            batch[count].id = frame.can_id & CAN_SFF_MASK;
            batch[count].dlc = frame.can_dlc > 8 ? 8 : frame.can_dlc;
            memcpy(batch[count].data, frame.data, batch[count].dlc);
            count++;
        }
    }

    /* Phase 2: 分发回调 (不持锁, 回调内可安全调 can_send) */
    for (int i = 0; i < count; i++) {
        if (ctx->global_callback) {
            ctx->global_callback(batch[i].id, batch[i].data,
                                batch[i].dlc, ctx->global_user_data);
        }
        for (int j = 0; j < ctx->callback_count; j++) {
            if (ctx->callbacks[j].can_id == batch[i].id
                && ctx->callbacks[j].callback) {
                ctx->callbacks[j].callback(batch[i].id, batch[i].data,
                                           batch[i].dlc,
                                           ctx->callbacks[j].user_data);
            }
        }
    }
    return count;
}
