/**
 * @file    test_raw_can.c
 * @brief   原始 CAN 数据收发示例
 *
 * 演示通过 SDK 的 CAN 传输层直接发送/接收原始 CAN 帧，
 * 并注册发送/接收回调监控总线数据。
 *
 * 用法: ./example_raw_can [can_if]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;

static void signal_handler(int sig)
{
    (void)sig;
    g_quit = true;
}

/* 接收回调：打印收到的 CAN 帧 */
static void on_can_recv(uint32_t can_id, const uint8_t *data,
                         uint8_t len, void *user_data)
{
    (void)user_data;
    printf("[RECV] 0x%03X [%d] ", can_id, len);
    for (uint8_t i = 0; i < len; i++)
        printf("%02X ", data[i]);
    printf("\n");
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* --- 打开 CAN 接口 --- */
    MiraCanCtx *can = miraculous_can_open(ifname, 0);
    if (!can) {
        fprintf(stderr, "Failed to open %s\n", ifname);
        return -1;
    }

    /* --- 注册接收回调 --- */
    miraculous_can_set_recv_callback(can, on_can_recv, NULL);

    printf("Raw CAN mode — sending heartbeat request every 1s on %s\n", ifname);
    printf("Press Ctrl+C to exit.\n");

    /* --- 循环发送 NMT 心跳请求 + 轮询接收 --- */
    while (!g_quit) {
        /* 发送 NMT 心跳请求 (广播) */
        uint8_t nmt_data[] = {0x00, 0x00}; /* CS + Node-ID */
        miraculous_can_send(can, 0x000, nmt_data, 2);
        usleep(100000);
    }

    miraculous_can_close(can);
    return 0;
}
