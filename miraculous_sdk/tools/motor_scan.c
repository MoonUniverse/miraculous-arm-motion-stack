/**
 * @file    motor_scan.c
 * @brief   CANopen 总线扫描 — SDO 探测 + 自动开启心跳
 *
 * 1. 对节点 1~16 读 0x1017 (心跳周期)
 * 2. 若为 0 则设为 5ms
 * 3. 切到心跳监听模式持续显示
 *
 * 用法: ./motor_scan [can_if]
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;
static void signal_handler(int sig) { (void)sig; g_quit = true; }

static uint8_t found[128] = {0};
static void hb_callback(uint8_t node_id, CoNmtState_t state, void *user_data)
{
    (void)user_data;
    if (!found[node_id]) {
        found[node_id] = 1;
        printf("  Node %3d  →  NMT State: 0x%02X", node_id, state);
        switch (state) {
        case CO_NMT_STATE_OPERATIONAL:     printf(" (Operational)\n"); break;
        case CO_NMT_STATE_PRE_OPERATIONAL: printf(" (Pre-Operational)\n"); break;
        case CO_NMT_STATE_STOPPED:         printf(" (Stopped)\n"); break;
        case CO_NMT_STATE_INITIALISING:    printf(" (Bootup)\n"); break;
        default:                           printf("\n"); break;
        }
    }
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "can0";
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 打开一个基准 motor (持有 master, 后续 motor 复用) */
    MiraMotor *base = miraculous_motor_open(ifname, 0, 1);
    if (!base) { fprintf(stderr, "Failed to open %s\n", ifname); return -1; }

    printf("Probing %s for motors...\n", ifname);

    /* SDO 探测 + 开启心跳 */
    int found_count = 0;
    for (int nid = 1; nid <= 16 && !g_quit; nid++) {
        /* 临时 motor (同 ifname 复用 master, refcount++) */
        MiraMotor *probe = miraculous_motor_open(ifname, 0, nid);
        if (!probe) continue;

        uint16_t period = 0;
        uint8_t len = 2;
        int ret = miraculous_motor_sdo_read(probe, 0x1017, 0, &period, &len);
        if (ret == 0) {
            printf("  Node %3d: heartbeat=%u ms", nid, period);
            found[nid] = 1;
            found_count++;
            if (period == 0) {
                uint16_t enable = 5;
                miraculous_motor_sdo_write(probe, 0x1017, 0, &enable, 2);
                printf(" → enabled 5ms");
            }
            printf("\n");
        }
        miraculous_motor_close(probe);
    }

    if (found_count > 0) {
        /* 清空 found, 让心跳回调重新检测 */
        memset(found, 0, sizeof(found));
        printf("\nMonitoring heartbeats on %s (10s)...\n", ifname);
        miraculous_motor_set_heartbeat_scan(base, hb_callback, NULL);
        time_t start = time(NULL);
        while (!g_quit && (time(NULL) - start) < 10) {
            usleep(200000);
        }
    } else {
        printf("  No motors found.\n");
    }

    printf("\nDone.\n");
    miraculous_motor_close(base);
    return 0;
}
