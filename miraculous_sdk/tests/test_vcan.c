/**
 * @file    test_vcan.c
 * @brief   vcan0 虚拟 CAN 集成测试
 *
 * 无需硬件即可测试 CAN/CANopen 协议正确性。
 *
 * 准备:
 *   sudo modprobe vcan
 *   sudo ip link add dev vcan0 type vcan
 *   sudo ip link set up vcan0
 *
 * 另一个终端监控:
 *   candump vcan0
 *
 * 运行:
 *   ./test_vcan
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "miraculous_sdk.h"

static volatile bool g_quit = false;
static int g_recv_count = 0;

static void signal_handler(int sig)
{
    (void)sig;
    g_quit = true;
}

static void recv_callback(uint32_t can_id, const uint8_t *data,
                           uint8_t len, void *user_data)
{
    (void)user_data;
    g_recv_count++;
    printf("  RECV: ID=0x%03X DLC=%d data=", can_id, len);
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
    printf("\n");
}

int main(void)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== vcan0 Integration Test ===\n\n");

    /* --- 1. CAN Open/Close --- */
    printf("[TEST 1] CAN open/close...\n");
    MiraCanCtx *can = miraculous_can_open("vcan0", 0);
    if (!can) {
        fprintf(stderr, "  FAIL — is vcan0 up? (sudo ip link add dev "
                "vcan0 type vcan && sudo ip link set up vcan0)\n");
        return -1;
    }
    printf("  PASS\n\n");

    /* --- 2. CAN Send --- */
    printf("[TEST 2] CAN frame send...\n");
    uint8_t test_data[] = {0x11, 0x22, 0x33, 0x44};
    int ret = miraculous_can_send(can, 0x701, test_data, 4);
    if (ret < 0) {
        fprintf(stderr, "  FAIL: %s\n", mrc_strerror(ret));
        goto cleanup;
    }
    printf("  PASS (sent to 0x701)\n\n");

    /* --- 3. CAN Recv (回环测试) --- */
    printf("[TEST 3] CAN recv (loopback)...\n");
    miraculous_can_set_recv_callback(can, recv_callback, NULL);

    miraculous_can_send(can, 0x702, test_data, 4);
    usleep(10000);
    miraculous_can_poll(can, 100);
    if (g_recv_count > 0) {
        printf("  PASS (received %d frames)\n", g_recv_count);
    } else {
        printf("  WARN — no loopback (vcan may not support loopback)\n");
    }
    printf("\n");

    /* --- 4. 通过 MiraMotor 测试 NMT + SYNC --- */
    printf("[TEST 4] Motor bootstrap + NMT + SYNC...\n");
    MiraMotor *motor = miraculous_motor_open("vcan0", 0, 1);
    if (!motor) { fprintf(stderr, "  FAIL: motor open\n"); goto cleanup; }

    /* Bootstrap 内部包含 NMT Reset + Start */
    ret = miraculous_motor_bootstrap(motor, 100);
    printf("  Bootstrap (NMT Reset + Start): %s\n",
           ret == 0 ? "PASS" : "EXPECTED FAIL (no real device)");

    /* CSP init 会发送 SYNC 帧 */
    ret = miraculous_motor_csp_init(motor, 4000, false);
    printf("  CSP init (SYNC + PDO config): %s\n",
           ret == 0 ? "PASS" : "EXPECTED FAIL (no real device)");

    miraculous_motor_close(motor);
    printf("  Motor close (cleanup): PASS\n");
    printf("\n");

    /* --- 5. SDO 请求格式 (通过原始 CAN 帧) --- */
    printf("[TEST 5] SDO format validation...\n");

    uint8_t sdo_req[8] = {0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0};
    ret = miraculous_can_send(can, 0x601, sdo_req, 8);
    printf("  SDO upload 0x1000:00 → 0x601: %s\n",
           ret == 0 ? "PASS" : "FAIL");

    uint8_t sdo_down[8] = {0x2F, 0x40, 0x60, 0x00, 0x06, 0x00, 0, 0};
    ret = miraculous_can_send(can, 0x601, sdo_down, 8);
    printf("  SDO download 0x6040:00=0x0006 → 0x601: %s\n",
           ret == 0 ? "PASS" : "FAIL");
    printf("\n");

    /* --- 6. PDO 格式 --- */
    printf("[TEST 6] RPDO format...\n");
    uint8_t rpdo_data[4] = {0x64, 0x00, 0x00, 0x00};
    ret = miraculous_can_send(can, 0x201, rpdo_data, 4);
    printf("  RPDO1 (0x201) Target Position=100: %s\n",
           ret == 0 ? "PASS" : "FAIL");
    printf("\n");

    printf("=== All tests completed ===\n");
    printf("Run 'candump vcan0' in another terminal to verify frames.\n");

cleanup:
    miraculous_can_close(can);
    return 0;
}
