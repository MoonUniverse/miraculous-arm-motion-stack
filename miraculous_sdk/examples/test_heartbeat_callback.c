/**
 * @file    test_heartbeat_callback.c
 * @brief   演示心跳 (Heartbeat) 回调的注册和使用
 *
 * 电机启用心跳后, 会以固定周期发送心跳帧。
 * 通过注册回调可实时监控节点的在线/离线状态。
 *
 * 用法: ./example_heartbeat_callback [can_if] [node_id]
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;

static void signal_handler(int sig) { (void)sig; g_quit = true; }

/*====================================================================
 * 心跳回调: 节点 NMT 状态变化时触发
 *====================================================================*/
static void on_heartbeat(uint8_t node_id, CoNmtState_t state,
                          void *user_data)
{
    (void)user_data;

    const char *desc;
    switch (state) {
    case CO_NMT_STATE_INITIALISING:     desc = "Initialising";         break;
    case CO_NMT_STATE_DISCONNECTED:     desc = "DISCONNECTED";         break;
    case CO_NMT_STATE_CONNECTING:       desc = "Connecting/Preparing";  break;
    case CO_NMT_STATE_STOPPED:          desc = "Stopped";              break;
    case CO_NMT_STATE_OPERATIONAL:      desc = "Operational";          break;
    case CO_NMT_STATE_PRE_OPERATIONAL:  desc = "Pre-Operational";      break;
    default:                            desc = "Unknown";              break;
    }

    printf("[HB] node %d -> %s\n", node_id, desc);
}

/*====================================================================
 * 主函数
 *====================================================================*/
int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    int node_id = (argc > 2) ? atoi(argv[2]) : 1;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* --- 打开电机 --- */
    MiraMotor *motor = miraculous_motor_open(ifname, 0, (uint8_t)node_id);
    if (!motor) {
        fprintf(stderr, "Failed to open motor on %s node %d\n",
                ifname, node_id);
        return -1;
    }
    printf("Motor opened: %s node %d\n", ifname, node_id);

    /* --- 启用心跳: 电机每 100ms 发送一次心跳帧 --- */
    if (miraculous_motor_set_heartbeat(motor, 100) < 0) {
        fprintf(stderr, "Failed to set heartbeat period\n");
        miraculous_motor_close(motor);
        return -1;
    }
    printf("Heartbeat enabled: period=100ms\n");

    /* --- 注册心跳回调: 节点状态变化时 on_heartbeat 被调用 --- */
    if (miraculous_motor_set_heartbeat_callback(motor, on_heartbeat, NULL) < 0) {
        fprintf(stderr, "Failed to set heartbeat callback\n");
        miraculous_motor_close(motor);
        return -1;
    }
    printf("Heartbeat callback registered.\n");
    printf("Waiting for heartbeat events... Press Ctrl+C to exit.\n\n");

    /* --- 等待 Heartbeat 事件 (由 recv 线程处理) --- */
    while (!g_quit) {
        usleep(100000);
    }

    /* --- 关闭电机 (自动停止心跳) --- */
    miraculous_motor_close(motor);
    return 0;
}
