/**
 * @file    motion_state.c
 * @brief   CiA402 PDS 状态机控制 + 通用工具
 *
 * 通过 SDO 写 0x6040 (controlword) 和读 0x6041 (statusword) 来实现
 * CiA402 状态转换。
 *
 * 状态转换表 (CiA 402, Table 10):
 *   Trans 2:  Shutdown       0x06 → Switch On Disabled
 *   Trans 3:  Switch On      0x07 → Ready to Switch On
 *   Trans 4:  Enable Op      0x0F → Operation Enabled
 *   Trans 5:  Disable Op     0x07 (bit3=0) → Switched On
 *   Trans 6:  Shutdown(SO)   0x06 → Ready to Switch On
 *   Trans 7:  Quick Stop     0x02 → Switch On Disabled (from Ready)
 *   Trans 8:  Shutdown(OE)   0x06 → Ready to Switch On
 *   Trans 9:  Disable Volt   0x00 → Switch On Disabled
 *   Trans 10: 故障发生 → Fault Reaction Active → Fault
 *   Trans 14: Fault Reset bit7=1 (上升沿) → Switch On Disabled
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <net/if.h>
#include "miraculous_internal.h"


/*----------------------------------------------------------------------------
 * 全局 CAN 总线注册表: 同一 ifname 复用同一个 master
 *----------------------------------------------------------------------------*/
#define MAX_BUS_REGISTRY  16
static struct {
    char          ifname[IFNAMSIZ];
    MiraCoMaster *master;
} g_bus_reg[MAX_BUS_REGISTRY];
static pthread_mutex_t g_bus_lock = PTHREAD_MUTEX_INITIALIZER;

/* 持锁版本 (调用者已持 g_bus_lock) */
static void unreg_master_locked(MiraCoMaster *co)
{
    for (int i = 0; i < MAX_BUS_REGISTRY; i++) {
        if (g_bus_reg[i].master == co) {
            g_bus_reg[i].master = NULL;
            g_bus_reg[i].ifname[0] = '\0';
            return;
        }
    }
}

/* 持锁版本 (调用者已持 g_bus_lock) */
static MiraCoMaster* find_master_locked(const char *ifname)
{
    for (int i = 0; i < MAX_BUS_REGISTRY; i++) {
        if (g_bus_reg[i].master &&
            strcmp(g_bus_reg[i].ifname, ifname) == 0) {
            return g_bus_reg[i].master;
        }
    }
    return NULL;
}

static int reg_master_locked(const char *ifname, MiraCoMaster *co)
{
    for (int i = 0; i < MAX_BUS_REGISTRY; i++) {
        if (!g_bus_reg[i].master) {
            strncpy(g_bus_reg[i].ifname, ifname, IFNAMSIZ - 1);
            g_bus_reg[i].ifname[IFNAMSIZ - 1] = '\0';
            g_bus_reg[i].master = co;
            return 0;
        }
    }
    return -1;
}

/* Release one motor's master reference while serializing against opens and
 * other closes.  miraculous_co_init() creates the first motor reference;
 * subsequent motors increment it. */
static void release_master(MiraCoMaster *co)
{
    if (!co) return;
    pthread_mutex_lock(&g_bus_lock);
    if (miraculous_co_ref_count(co) <= 1) {
        unreg_master_locked(co);
    }
    miraculous_co_free(co);
    pthread_mutex_unlock(&g_bus_lock);
}

/*----------------------------------------------------------------------------
 * 内部辅助
 *----------------------------------------------------------------------------*/

static int motor_sdo_write_u16(MiraMotor *m, uint16_t idx, uint8_t sub,
                                uint16_t val)
{
    return CO_SDO_WRITE(m->co, m->node_id, idx, sub, &val, MOTION_SDO_TIMEOUT_MS);
}


static int motor_sdo_write_i16(MiraMotor *m, uint16_t idx, uint8_t sub,
                                int16_t val)
{
    return CO_SDO_WRITE(m->co, m->node_id, idx, sub, &val, MOTION_SDO_TIMEOUT_MS);
}

static int motor_sdo_read_u16(MiraMotor *m, uint16_t idx, uint8_t sub,
                               uint16_t *val)
{
    uint8_t len = 2;
    int ret = miraculous_co_sdo_read(m->co, m->node_id, idx, sub,
                                   val, &len, MOTION_SDO_TIMEOUT_MS);
    return ret;
}

static int motor_sdo_write_i32(MiraMotor *m, uint16_t idx, uint8_t sub,
                                int32_t val)
{
    return CO_SDO_WRITE(m->co, m->node_id, idx, sub, &val, MOTION_SDO_TIMEOUT_MS);
}

static int motor_sdo_read_i16(MiraMotor *m, uint16_t idx, uint8_t sub,
                               int16_t *val)
{
    uint8_t len = 2;
    return miraculous_co_sdo_read(m->co, m->node_id, idx, sub,
                                   val, &len, MOTION_SDO_TIMEOUT_MS);
}

static int32_t decode_i32_le(const uint8_t *data)
{
    uint32_t raw = (uint32_t)data[0]
                 | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16)
                 | ((uint32_t)data[3] << 24);
    int32_t value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static int16_t decode_i16_le(const uint8_t *data)
{
    uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    int16_t value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/*----------------------------------------------------------------------------
 * 状态解析
 *----------------------------------------------------------------------------*/

static Cia402State_t sw_to_state(uint16_t sw)
{
    uint16_t masked = sw & CIA402_SW_STATE_MASK;
    /* 根据位模式映射 */
    if (masked == 0x0000) return CIA_STATE_NOT_READY_TO_SWITCH_ON;
    if (masked == 0x0040) return CIA_STATE_SWITCH_ON_DISABLED;
    if (masked == 0x0021) return CIA_STATE_READY_TO_SWITCH_ON;
    if (masked == 0x0023) return CIA_STATE_SWITCHED_ON;
    if ((masked & 0x006F) == 0x0027) return CIA_STATE_OPERATION_ENABLED;
    if (masked == 0x0007) return CIA_STATE_QUICK_STOP_ACTIVE;
    if (masked == 0x000F) return CIA_STATE_FAULT_REACTION_ACTIVE;
    if (masked == 0x0008) return CIA_STATE_FAULT;
    /* MCSDK 可能用 0x0037 表示 Operation Enabled */
    if (masked == 0x0037) return CIA_STATE_OPERATION_ENABLED;
    return CIA_STATE_NOT_READY_TO_SWITCH_ON;
}

/*----------------------------------------------------------------------------
 * TPDO 接收回调: 缓存实际位置 (0x6064) + 实际速度 (0x606C) + 实际转矩 (0x6077)
 *----------------------------------------------------------------------------*/
static void on_tpdo1(uint8_t node_id, uint8_t pdo_num,
                      const uint8_t *data, uint8_t len,
                      void *user_data)
{
    (void)node_id; (void)len;
    MiraMotor *motor = (MiraMotor *)user_data;
    if (!motor) return;

    if (pdo_num == 2 && len >= 8) {
    /* TPDO1(0x280+nid) 映射: 0x6064(4B) + 0x606C(4B), LE */
    __atomic_store_n(&motor->pdo_pos, decode_i32_le(data), __ATOMIC_RELAXED);
    __atomic_store_n(&motor->pdo_vel, decode_i32_le(data + 4), __ATOMIC_RELAXED);
    __atomic_store_n(&motor->pdo_valid, true, __ATOMIC_RELEASE);
    } else if (pdo_num == 3 && len >= 2) {
        /* TPDO3(0x380+nid) 映射: 0x6077 实际转矩 (16位) */
        __atomic_store_n(&motor->pdo_torque, decode_i16_le(data), __ATOMIC_RELEASE);
    }

    /* Snapshot the callback pair under the lock, then invoke it unlocked.
     * The surrounding PDO registry keeps MiraMotor alive through this handler;
     * invoking user code while holding callback_lock would deadlock if the
     * callback updates or clears its own registration. */
    pthread_mutex_lock(&motor->callback_lock);
    MiraTpdoCallback callback = motor->tpdo_user_cb;
    void *callback_data = motor->tpdo_user_data;
    pthread_mutex_unlock(&motor->callback_lock);
    if (callback) {
        callback(node_id, pdo_num, data, len, callback_data);
    }
}

/*----------------------------------------------------------------------------
 * 生命周期
 *----------------------------------------------------------------------------*/

MiraMotor* miraculous_motor_open(const char *ifname,
                                  CiaBaudrate_t baudrate,
                                  uint8_t node_id)
{
    if (!ifname || node_id == 0 || node_id > 127) return NULL;

    /* 检查是否已有同一 ifname 的 master，并在注册表锁内取得引用。 */
    pthread_mutex_lock(&g_bus_lock);
    MiraCoMaster *co = find_master_locked(ifname);
    if (co) {
        miraculous_co_ref_inc(co);
    }
    pthread_mutex_unlock(&g_bus_lock);

    if (!co) {
        /* 没有: 创建新 CAN + CANopen 主站 */
        MiraCanCtx *can = miraculous_can_open(ifname, baudrate);
        if (!can) {
            fprintf(stderr, "[motor] can_open(%s) failed\n", ifname);
            return NULL;
        }

        co = miraculous_co_init(can, baudrate, true);
        if (!co) {
            fprintf(stderr, "[motor] co_init failed\n");
            return NULL;
        }
        /* 双检锁: 持锁确认无其他线程抢先注册 */
        pthread_mutex_lock(&g_bus_lock);
        MiraCoMaster *existing = find_master_locked(ifname);
        if (existing) {
            /* 其他线程已注册, 释放新创建的, 使用已有的 */
            miraculous_co_ref_inc(existing);
            pthread_mutex_unlock(&g_bus_lock);
            miraculous_co_free(co);
            co = existing;
        } else {
            /* 注册到全局表 */
            if (reg_master_locked(ifname, co) != 0) {
                pthread_mutex_unlock(&g_bus_lock);
                fprintf(stderr, "[motor] bus registry full\n");
                miraculous_co_free(co);
                return NULL;
            }
            pthread_mutex_unlock(&g_bus_lock);
        }
    }
    /* 创建电机句柄 */
    MiraMotor *motor = calloc(1, sizeof(MiraMotor));
    if (!motor) {
        release_master(co);
        return NULL;
    }

    if (pthread_mutex_init(&motor->sdo_async_lock, NULL) != 0) {
        free(motor);
        release_master(co);
        return NULL;
    }
    if (pthread_mutex_init(&motor->callback_lock, NULL) != 0) {
        pthread_mutex_destroy(&motor->sdo_async_lock);
        free(motor);
        release_master(co);
        return NULL;
    }

    motor->co      = co;
    motor->node_id = node_id;
    motor->encoder_bw = 19;  /* 默认 19-bit (524288 counts/rev), 用户可改 */
    motor->reduction_ratio = 100.0f; /* 默认减速比 100 */

    pthread_mutex_lock(&co->motor_registry_lock);
    if (co->motor_by_node[node_id]) {
        pthread_mutex_unlock(&co->motor_registry_lock);
        fprintf(stderr, "[motor] node %d is already open on %s\n", node_id, ifname);
        pthread_mutex_destroy(&motor->callback_lock);
        pthread_mutex_destroy(&motor->sdo_async_lock);
        free(motor);
        release_master(co);
        return NULL;
    }
    co->motor_by_node[node_id] = motor;
    pthread_mutex_unlock(&co->motor_registry_lock);

    /* 注册 TPDO2(0x280+nid) 回调: 固件映射 0x6064(位置)+0x606C(速度) */
    int ret2 = miraculous_co_pdo_set_tpdo_callback(co, node_id, 2,
                                         CO_COB_TPDO2 + node_id,
                                         on_tpdo1, motor);
    if (ret2 < 0) {
        fprintf(stderr, "[motor] TPDO2 callback register failed for node %d: %s\n",
                node_id, mrc_strerror(ret2));
    }
    /* 注册 TPDO3(0x380+nid) 回调: 固件映射 0x6077(转矩) */
    int ret3 = miraculous_co_pdo_set_tpdo_callback(co, node_id, 3,
                                         CO_COB_TPDO3 + node_id,
                                         on_tpdo1, motor);
    if (ret3 < 0) {
        fprintf(stderr, "[motor] TPDO3 callback register failed for node %d: %s\n",
                node_id, mrc_strerror(ret3));
    }

    if (ret2 < 0 || ret3 < 0) {
        miraculous_co_pdo_remove_tpdo_callbacks(co, node_id, motor);
        pthread_mutex_lock(&co->motor_registry_lock);
        if (co->motor_by_node[node_id] == motor) {
            co->motor_by_node[node_id] = NULL;
        }
        pthread_mutex_unlock(&co->motor_registry_lock);
        pthread_mutex_destroy(&motor->callback_lock);
        pthread_mutex_destroy(&motor->sdo_async_lock);
        free(motor);
        release_master(co);
        return NULL;
    }

    printf("[motor] opened: iface=%s node=%d (refcount=%d)\n",
           ifname, node_id, miraculous_co_ref_count(co));
    return motor;
}

void miraculous_motor_close(MiraMotor *motor)
{
    if (!motor) return;

    uint8_t nid = motor->node_id;
    MiraCoMaster *co = motor->co;

    /* 如果该电机启动过 SYNC, 递减 SYNC 引用计数
     * 只有最后一个引用者才真正停掉 timerfd */
    if (motor->sync_active && co) {
        miraculous_co_sync_stop(co);
    }

    /* Remove SDK callbacks first. The PDO registry lock waits for any in-flight
     * TPDO callback before the motor handle can be freed. */
    if (co) {
        miraculous_co_pdo_remove_tpdo_callbacks(co, nid, motor);
    }

    /* Remove the handle from asynchronous SDO dispatch and clean its tasks
     * while holding the master registry lock. */
    if (co) pthread_mutex_lock(&co->motor_registry_lock);
    if (co && co->motor_by_node[nid] == motor) {
        co->motor_by_node[nid] = NULL;
    }
    pthread_mutex_lock(&motor->sdo_async_lock);
    {
        MiraSdoAsyncTask *t = motor->sdo_async_list;
        while (t) {
            MiraSdoAsyncTask *next = t->next;
            free(t);
            t = next;
        }
        motor->sdo_async_list = NULL;
    }
    pthread_mutex_unlock(&motor->sdo_async_lock);
    if (co) pthread_mutex_unlock(&co->motor_registry_lock);

    pthread_mutex_lock(&motor->callback_lock);
    motor->tpdo_user_cb = NULL;
    motor->tpdo_user_data = NULL;
    pthread_mutex_unlock(&motor->callback_lock);
    pthread_mutex_destroy(&motor->callback_lock);
    pthread_mutex_destroy(&motor->sdo_async_lock);
    free(motor);

    release_master(co);

    printf("[motor] closed node %d\n", nid);
}

int miraculous_motor_set_heartbeat_callback(MiraMotor *motor,
                                             MiraHeartbeatCallback callback,
                                             void *user_data)
{
    if (!motor || !motor->co) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_heartbeat_set_callback(motor->co, motor->node_id,
                                                 callback, user_data);
}

int miraculous_motor_set_heartbeat_scan(MiraMotor *motor,
                                         MiraHeartbeatCallback callback,
                                         void *user_data)
{
    if (!motor || !motor->co) return MRC_ERROR_INVALID_PARAM;
    for (int nid = 1; nid <= 127; nid++) {
        int ret = miraculous_co_heartbeat_set_callback(motor->co, nid,
                                                        callback, user_data);
        if (ret < 0 && ret != MRC_ERROR_OUT_OF_MEMORY) return ret;
    }
    return MRC_SUCCESS;
}

int miraculous_motor_set_tpdo_callback(MiraMotor *motor,
                                        MiraTpdoCallback callback,
                                        void *user_data)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    pthread_mutex_lock(&motor->callback_lock);
    motor->tpdo_user_cb = callback;
    motor->tpdo_user_data = user_data;
    pthread_mutex_unlock(&motor->callback_lock);
    return MRC_SUCCESS;
}

int miraculous_motor_set_emcy_callback(MiraMotor *motor,
                                        MiraEmcyCallback callback,
                                        void *user_data)
{
    if (!motor || !motor->co) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_emcy_set_callback(motor->co, callback, user_data);
}

int miraculous_motor_rpdo_config(MiraMotor *motor, uint8_t pdo_num,
                                  uint32_t cob_id, uint8_t trans_type,
                                  uint16_t event_timer_ms,
                                  uint8_t mapped_count,
                                  const uint32_t *mappings)
{
    if (!motor || !motor->co) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_pdo_rpdo_config(motor->co, motor->node_id,
                                          pdo_num, cob_id, trans_type,
                                          event_timer_ms,
                                          mapped_count, mappings);
}

int miraculous_motor_rpdo_send(MiraMotor *motor, uint8_t pdo_num,
                                const uint8_t *data, uint8_t len)
{
    if (!motor || !motor->co) return MRC_ERROR_INVALID_PARAM;
    /* 根据 pdo_num 计算 RPDO COB-ID: 0x200/0x300/0x400/0x500 + node_id */
    uint32_t cob_id = (0x200U + (pdo_num - 1) * 0x80U) + motor->node_id;
    return miraculous_co_pdo_send(motor->co, pdo_num, cob_id, data, len);
}

int miraculous_motor_tpdo_config(MiraMotor *motor, uint8_t pdo_num,
                                  uint32_t cob_id, uint8_t trans_type,
                                  uint8_t inhibit_time, uint16_t event_timer_ms,
                                  uint8_t mapped_count,
                                  const uint32_t *mappings)
{
    if (!motor || !motor->co) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_pdo_tpdo_config(motor->co, motor->node_id,
                                          pdo_num, cob_id, trans_type,
                                          inhibit_time, event_timer_ms,
                                          mapped_count, mappings);
}

MiraCanCtx* miraculous_motor_get_can_ctx(MiraMotor *motor)
{
    if (!motor || !motor->co) return NULL;
    return miraculous_co_get_can(motor->co);
}

uint8_t miraculous_motor_get_node_id(MiraMotor *motor)
{
    return motor ? motor->node_id : 0;
}

int miraculous_motor_bootstrap(MiraMotor *motor, int timeout_ms)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_bootstrap(motor->co, motor->node_id, timeout_ms);
}

int miraculous_motor_nmt_send(MiraMotor *motor, CoNmtCommand_t cmd)
{
    if (!motor || !motor->co) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_nmt_send(motor->co, motor->node_id, cmd);
}

/*----------------------------------------------------------------------------
 * 状态机控制
 *----------------------------------------------------------------------------*/

int miraculous_motor_shutdown(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    uint16_t cw = CIA402_CW_SHUTDOWN;
    return CO_SDO_WRITE(motor->co, motor->node_id,
                        CIA402_OD_CONTROLWORD, 0, &cw, SHUTDOWN_SDO_TIMEOUT_MS);
}

int miraculous_motor_switch_on(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_write_u16(motor, CIA402_OD_CONTROLWORD, 0,
                                CIA402_CW_SWITCH_ON);
}

int miraculous_motor_enable(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_write_u16(motor, CIA402_OD_CONTROLWORD, 0,
                                CIA402_CW_ENABLE_OPERATION);
}

int miraculous_motor_disable(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    /* Disable Operation: 写 0x0007 (bit0-2=1, bit3=0) */
    return motor_sdo_write_u16(motor, CIA402_OD_CONTROLWORD, 0,
                                CIA402_CW_DISABLE_OPERATION);
}

int miraculous_motor_disable_voltage(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_write_u16(motor, CIA402_OD_CONTROLWORD, 0,
                                CIA402_CW_DISABLE_VOLTAGE);
}

int miraculous_motor_quick_stop(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_write_u16(motor, CIA402_OD_CONTROLWORD, 0,
                                CIA402_CW_QUICK_STOP);
}

int miraculous_motor_fault_reset(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    /* 先检查是否在 Fault */
    Cia402State_t state;
    int ret = miraculous_motor_get_state(motor, &state);
    if (ret < 0) return ret;
    if (state != CIA_STATE_FAULT) {
        return MRC_SUCCESS; /* 不在故障状态，无需复位 */
    }

    /* 上升沿: 先设 0x00 再设 0x80 */
    uint16_t cw = 0x0000;
    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_CONTROLWORD, 0, &cw, MOTION_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    cw = CIA402_CW_FAULT_RESET;
    ret = CO_SDO_WRITE(motor->co, motor->node_id,
                       CIA402_OD_CONTROLWORD, 0, &cw, MOTION_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* 清 bit7 */
    cw = 0x0000;
    return CO_SDO_WRITE(motor->co, motor->node_id,
                        CIA402_OD_CONTROLWORD, 0, &cw, MOTION_SDO_TIMEOUT_MS);
}

int miraculous_motor_full_enable(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int ret;
    Cia402State_t initial;

    /* 先检查当前状态 (SDO 可能在总线繁忙时超时, 重试一次) */
    ret = miraculous_motor_get_state(motor, &initial);
    if (ret < 0) {
        usleep(50000);
        ret = miraculous_motor_get_state(motor, &initial);
    }
    if (ret < 0) return ret;

    if (initial == CIA_STATE_FAULT || initial == CIA_STATE_FAULT_REACTION_ACTIVE) {
        printf("[motor] node %d in Fault state, performing fault reset...\n",
               motor->node_id);
        /* 直接发 Fault Reset 序列 (0x00→0x80→0x00)，不再调用
         * fault_reset() 避免其内部的 get_state 被残留帧干扰 */
        uint16_t cw = 0x0000;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_CONTROLWORD, 0, &cw, MOTION_SDO_TIMEOUT_MS);
        if (ret < 0) return ret;
        cw = CIA402_CW_FAULT_RESET;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_CONTROLWORD, 0, &cw, MOTION_SDO_TIMEOUT_MS);
        if (ret < 0) return ret;
        cw = 0x0000;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_CONTROLWORD, 0, &cw, MOTION_SDO_TIMEOUT_MS);
        if (ret < 0) return ret;

        /* 等待进入 Switch On Disabled */
        ret = miraculous_motor_wait_state(motor, CIA_STATE_SWITCH_ON_DISABLED, 1000);
        if (ret < 0) return ret;
        initial = CIA_STATE_SWITCH_ON_DISABLED;
    }

    /* 根据初始状态智能选择转换路径，避免从错误的状态发 Shutdown
     * (如 Ready to Switch On 发 0x06 会回退到 Switch On Disabled) */
retry_state:
    switch (initial) {
    case CIA_STATE_OPERATION_ENABLED:
        printf("[motor] node %d already enabled\n", motor->node_id);
        return MRC_SUCCESS;

    case CIA_STATE_SWITCHED_ON:
        /* 已处于 Switched On，跳过 Step 1 和 Step 2 */
        goto step3;

    case CIA_STATE_READY_TO_SWITCH_ON:
        /* 已处于 Ready to Switch On，跳过 Step 1 */
        goto step2;

    case CIA_STATE_SWITCH_ON_DISABLED:
    case CIA_STATE_QUICK_STOP_ACTIVE:
        /* 从 Step 1 开始正常序列 */
        break;

    case CIA_STATE_NOT_READY_TO_SWITCH_ON:
        /* 设备初始化为瞬态, 最多重试 3 次 */
        for (int attempt = 0; attempt < 3; attempt++) {
            printf("[motor] node %d not ready, waiting 200ms...\n",
                   motor->node_id);
            usleep(200000);
            ret = miraculous_motor_get_state(motor, &initial);
            if (ret < 0) return ret;
            if (initial != CIA_STATE_NOT_READY_TO_SWITCH_ON)
                goto retry_state;
        }
        printf("[motor] node %d still not ready after 3 attempts\n",
               motor->node_id);
        return MRC_ERROR_TIMEOUT;

    default:
        break;
    }

    /* Step 1: Shutdown → Ready to Switch On (0x06) */
    ret = miraculous_motor_shutdown(motor);
    if (ret < 0) return ret;
    ret = miraculous_motor_wait_state(motor, CIA_STATE_READY_TO_SWITCH_ON, 1000);
    if (ret < 0) return ret;

step2:
    /* Step 2: Switch On → Switched On (0x07) */
    ret = miraculous_motor_switch_on(motor);
    if (ret < 0) return ret;
    ret = miraculous_motor_wait_state(motor, CIA_STATE_SWITCHED_ON, 1000);
    if (ret < 0) return ret;

step3:
    /* Step 3: Enable Operation → Operation Enabled (0x0F) */
    {
        uint16_t cw = CIA402_CW_ENABLE_OPERATION;
        ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_CONTROLWORD, 0, &cw, MOTION_SDO_TIMEOUT_MS);
    }
    if (ret < 0) return ret;
    ret = miraculous_motor_wait_state(motor, CIA_STATE_OPERATION_ENABLED, 4000);
    if (ret < 0) return ret;

    printf("[motor] node %d fully enabled\n", motor->node_id);
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 状态读取
 *----------------------------------------------------------------------------*/

int miraculous_motor_get_state(MiraMotor *motor, Cia402State_t *state)
{
    if (!motor || !state) return MRC_ERROR_INVALID_PARAM;

    uint16_t sw;
    int ret = motor_sdo_read_u16(motor, CIA402_OD_STATUSWORD, 0, &sw);
    if (ret < 0) return ret;

    *state = sw_to_state(sw);
    return MRC_SUCCESS;
}

int miraculous_motor_get_statusword(MiraMotor *motor, uint16_t *sw)
{
    if (!motor || !sw) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_read_u16(motor, CIA402_OD_STATUSWORD, 0, sw);
}

int miraculous_motor_wait_state(MiraMotor *motor, Cia402State_t expected,
                                 int timeout_ms)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    if (timeout_ms <= 0) timeout_ms = 10;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        /* 检查当前状态 */
        Cia402State_t cur;
        int ret = miraculous_motor_get_state(motor, &cur);
        if (ret == 0) {
            if (cur == expected) return MRC_SUCCESS;
            /* 检测到 Fault 立即返回 */
            if (cur == CIA_STATE_FAULT || cur == CIA_STATE_FAULT_REACTION_ACTIVE) {
                return MRC_ERROR_MOTION_FAULT;
            }
        }
        /* SDO 超时也继续等, 不退出 */

        /* 超时检查 */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t elapsed = (now.tv_sec - start.tv_sec) * 1000LL
                        + (now.tv_nsec - start.tv_nsec) / 1000000LL;
        if (elapsed >= timeout_ms) {
            return MRC_ERROR_TIMEOUT;
        }

        usleep(5000);
    }
}

/*----------------------------------------------------------------------------
 * 操作模式
 *----------------------------------------------------------------------------*/

int miraculous_motor_set_mode(MiraMotor *motor, Cia402Mode_t mode)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int8_t m = (int8_t)mode;
    int ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_MODES_OF_OPERATION, 0,
                           &m, SET_MODE_SDO_TIMEOUT_MS);
    if (ret < 0) {
        fprintf(stderr, "[motor] set_mode node=%d mode=0x%02X failed: %s (err=%d)\n",
                motor->node_id, (unsigned)mode, mrc_strerror(ret), ret);
    }
    return ret;
}

int miraculous_motor_get_mode(MiraMotor *motor, Cia402Mode_t *mode)
{
    if (!motor || !mode) return MRC_ERROR_INVALID_PARAM;
    int8_t m;
    uint8_t len = 1;
    int ret = miraculous_co_sdo_read(motor->co, motor->node_id,
                                      CIA402_OD_MODES_OF_OPERATION_DISPLAY,
                                      0, &m, &len, MOTION_SDO_TIMEOUT_MS);
    if (ret < 0) return ret;
    *mode = (Cia402Mode_t)m;
    return MRC_SUCCESS;
}

int miraculous_motor_get_supported_modes(MiraMotor *motor, uint32_t *modes)
{
    if (!motor || !modes) return MRC_ERROR_INVALID_PARAM;
    uint8_t len = 4;
    return miraculous_co_sdo_read(motor->co, motor->node_id,
                                   CIA402_OD_SUPPORTED_DRIVE_MODES,
                                   0, modes, &len, MOTION_SDO_TIMEOUT_MS);
}

/*----------------------------------------------------------------------------
 * 实际值读取
 *----------------------------------------------------------------------------*/

int miraculous_motor_get_position(MiraMotor *motor, int32_t *pos)
{
    if (!motor || !pos) return MRC_ERROR_INVALID_PARAM;
    if (__atomic_load_n(&motor->pdo_valid, __ATOMIC_ACQUIRE)) {
        *pos = __atomic_load_n(&motor->pdo_pos, __ATOMIC_RELAXED);
        return MRC_SUCCESS;
    }
    return MRC_ERROR_TIMEOUT;
}

int miraculous_motor_get_position_ex(MiraMotor *motor, float *pos, PosUnit_t unit)
{
    if (!motor || !pos) return MRC_ERROR_INVALID_PARAM;

    int32_t raw_pos;
    int ret = miraculous_motor_get_position(motor, &raw_pos);
    if (ret < 0) return ret;

    if (unit == POS_UNIT_RADIAN) {
        *pos = POS_TO_RAD(raw_pos, motor->encoder_bw);
    } else {
        *pos = POS_TO_DEG(raw_pos, motor->encoder_bw);
    }
    return MRC_SUCCESS;
}

int miraculous_motor_set_zero_position(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    /* 1. 先复位 Home Offset 为 0, 确保读取的是原始编码器位置 */
    int ret = motor_sdo_write_i32(motor, CIA402_OD_HOME_OFFSET, 0, 0);
    if (ret < 0) return ret;

    /* 2. 读取当前位置 (0x6064) — 此时不含任何偏移 */
    int32_t current_pos;
    ret = miraculous_motor_get_position(motor, &current_pos);
    if (ret < 0) return ret;

    /* 3. 写入 Home Offset (0x607C): offset = current_position
     *    使当前位置成为新的零位参考点
     *    ActualPosition = RawPosition - HomeOffset => 当前位置归零 */
    return motor_sdo_write_i32(motor, CIA402_OD_HOME_OFFSET, 0, current_pos);
}

int miraculous_motor_set_target_position_ex(MiraMotor *motor, float pos, PosUnit_t unit)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;

    int32_t pulses;
    int ret = miraculous_position_to_counts(pos, unit, motor->encoder_bw, &pulses);
    if (ret < 0) return ret;

    return motor_sdo_write_i32(motor, CIA402_OD_TARGET_POSITION, 0, pulses);
}

int miraculous_motor_set_encoder_bw(MiraMotor *motor, uint8_t bw)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    if (bw == 0 || bw > 31) return MRC_ERROR_INVALID_PARAM;
    motor->encoder_bw = bw;
    return MRC_SUCCESS;
}

int miraculous_motor_set_reduction_ratio(MiraMotor *motor, float ratio)
{
    if (!motor || !isfinite(ratio) || ratio <= 0) return MRC_ERROR_INVALID_PARAM;
    motor->reduction_ratio = ratio;
    return MRC_SUCCESS;
}

int miraculous_motor_sync_start(MiraMotor *motor, uint32_t period_us)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_sync_start(motor->co, period_us);
}

int miraculous_motor_sync_stop(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_sync_stop(motor->co);
}

int miraculous_motor_sync_send(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_sync_send_once(motor->co);
}

int miraculous_motor_get_velocity(MiraMotor *motor, int32_t *vel)
{
    if (!motor || !vel) return MRC_ERROR_INVALID_PARAM;
    if (__atomic_load_n(&motor->pdo_valid, __ATOMIC_ACQUIRE)) {
        *vel = __atomic_load_n(&motor->pdo_vel, __ATOMIC_RELAXED);
        return MRC_SUCCESS;
    }
    return MRC_ERROR_TIMEOUT;
}

int miraculous_motor_get_velocity_ex(MiraMotor *motor, float *vel,
                                      VelSide_t side, VelUnit_t unit)
{
    if (!motor || !vel) return MRC_ERROR_INVALID_PARAM;

    int32_t rpm;
    int ret = miraculous_motor_get_velocity(motor, &rpm);
    if (ret < 0) return ret;

    float v = (float)rpm;
    if (side == VEL_SIDE_LOAD)
        v /= motor->reduction_ratio;

    if (unit == VEL_UNIT_RAD_S)
        v *= (float)(6.283185307179586 / 60.0);  /* RPM → rad/s */

    *vel = v;
    return MRC_SUCCESS;
}

int miraculous_motor_get_torque(MiraMotor *motor, int16_t *torque)
{
    if (!motor || !torque) return MRC_ERROR_INVALID_PARAM;
    /* 转矩通过 TPDO3 缓存, 不依赖 SDO */
    *torque = __atomic_load_n(&motor->pdo_torque, __ATOMIC_ACQUIRE);
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 温度
 *----------------------------------------------------------------------------*/

int miraculous_motor_get_temperature(MiraMotor *motor,
                                      int16_t *motor_temp,
                                      int16_t *mos_temp)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int ret;

    if (motor_temp) {
        ret = motor_sdo_read_i16(motor, CIA402_OD_SERVO_TEMPERATURE, 0x01,
                                  motor_temp);
        if (ret < 0) return ret;
    }
    if (mos_temp) {
        ret = motor_sdo_read_i16(motor, CIA402_OD_SERVO_TEMPERATURE, 0x02,
                                  mos_temp);
        if (ret < 0) return ret;
    }
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 电流环 PI (厂商特定 0x2002)
 * sub1: kp_reg (I16), sub2: kp_divisor (U16 = 2^N 分母指数)
 * sub3: ki_reg (I16), sub4: ki_divisor (U16 = 2^N 分母指数)
 * 实际增益 = 寄存器值 / (2^divisor)
 *----------------------------------------------------------------------------*/

int miraculous_motor_set_current_pi(MiraMotor *motor,
                                     int16_t kp_reg, uint16_t kp_divisor,
                                     int16_t ki_reg, uint16_t ki_divisor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int ret;
    ret = motor_sdo_write_i16(motor, 0x2002, 0x01, kp_reg);
    if (ret < 0) return ret;
    ret = motor_sdo_write_u16(motor, 0x2002, 0x02, kp_divisor);
    if (ret < 0) return ret;
    ret = motor_sdo_write_i16(motor, 0x2002, 0x03, ki_reg);
    if (ret < 0) return ret;
    ret = motor_sdo_write_u16(motor, 0x2002, 0x04, ki_divisor);
    return ret;
}

int miraculous_motor_get_current_pi(MiraMotor *motor,
                                     int16_t *kp_reg, uint16_t *kp_divisor,
                                     int16_t *ki_reg, uint16_t *ki_divisor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int ret;
    if (kp_reg) {
        ret = motor_sdo_read_i16(motor, 0x2002, 0x01, kp_reg);
        if (ret < 0) return ret;
    }
    if (kp_divisor) {
        ret = motor_sdo_read_u16(motor, 0x2002, 0x02, kp_divisor);
        if (ret < 0) return ret;
    }
    if (ki_reg) {
        ret = motor_sdo_read_i16(motor, 0x2002, 0x03, ki_reg);
        if (ret < 0) return ret;
    }
    if (ki_divisor) {
        ret = motor_sdo_read_u16(motor, 0x2002, 0x04, ki_divisor);
        if (ret < 0) return ret;
    }
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 速度环 PI (厂商特定 0x2003)
 *----------------------------------------------------------------------------*/

int miraculous_motor_set_velocity_pi(MiraMotor *motor,
                                      int16_t kp_reg, uint16_t kp_divisor,
                                      int16_t ki_reg, uint16_t ki_divisor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int ret;
    ret = motor_sdo_write_i16(motor, 0x2003, 0x01, kp_reg);
    if (ret < 0) return ret;
    ret = motor_sdo_write_u16(motor, 0x2003, 0x02, kp_divisor);
    if (ret < 0) return ret;
    ret = motor_sdo_write_i16(motor, 0x2003, 0x03, ki_reg);
    if (ret < 0) return ret;
    ret = motor_sdo_write_u16(motor, 0x2003, 0x04, ki_divisor);
    return ret;
}

int miraculous_motor_get_velocity_pi(MiraMotor *motor,
                                      int16_t *kp_reg, uint16_t *kp_divisor,
                                      int16_t *ki_reg, uint16_t *ki_divisor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int ret;
    if (kp_reg) {
        ret = motor_sdo_read_i16(motor, 0x2003, 0x01, kp_reg);
        if (ret < 0) return ret;
    }
    if (kp_divisor) {
        ret = motor_sdo_read_u16(motor, 0x2003, 0x02, kp_divisor);
        if (ret < 0) return ret;
    }
    if (ki_reg) {
        ret = motor_sdo_read_i16(motor, 0x2003, 0x03, ki_reg);
        if (ret < 0) return ret;
    }
    if (ki_divisor) {
        ret = motor_sdo_read_u16(motor, 0x2003, 0x04, ki_divisor);
        if (ret < 0) return ret;
    }
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 位置环 PI (厂商特定 0x2004)
 * 注意: ki_reg 为 U16 (不同于电流/速度环的 I16)
 *----------------------------------------------------------------------------*/

int miraculous_motor_set_position_pi(MiraMotor *motor,
                                      int16_t kp_reg, uint16_t kp_divisor,
                                      uint16_t ki_reg, uint16_t ki_divisor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int ret;
    ret = motor_sdo_write_i16(motor, 0x2004, 0x01, kp_reg);
    if (ret < 0) return ret;
    ret = motor_sdo_write_u16(motor, 0x2004, 0x02, kp_divisor);
    if (ret < 0) return ret;
    ret = CO_SDO_WRITE(motor->co, motor->node_id, 0x2004, 0x03,
                       &ki_reg, 300);
    if (ret < 0) return ret;
    ret = motor_sdo_write_u16(motor, 0x2004, 0x04, ki_divisor);
    return ret;
}

int miraculous_motor_get_position_pi(MiraMotor *motor,
                                      int16_t *kp_reg, uint16_t *kp_divisor,
                                      uint16_t *ki_reg, uint16_t *ki_divisor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    int ret;
    if (kp_reg) {
        ret = motor_sdo_read_i16(motor, 0x2004, 0x01, kp_reg);
        if (ret < 0) return ret;
    }
    if (kp_divisor) {
        ret = motor_sdo_read_u16(motor, 0x2004, 0x02, kp_divisor);
        if (ret < 0) return ret;
    }
    if (ki_reg) {
        uint8_t len = 2;
        ret = miraculous_co_sdo_read(motor->co, motor->node_id,
                                     0x2004, 0x03, ki_reg, &len, 300);
        if (ret < 0) return ret;
    }
    if (ki_divisor) {
        ret = motor_sdo_read_u16(motor, 0x2004, 0x04, ki_divisor);
        if (ret < 0) return ret;
    }
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 波特率配置 (0x2001 sub2)
 *----------------------------------------------------------------------------*/

int miraculous_motor_get_baudrate(MiraMotor *motor, uint16_t *baudrate)
{
    if (!motor || !baudrate) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_read_u16(motor,
                               CIA402_OD_SERVO_PARAMETERS,
                               CIA402_OD_SERVO_PARAMETERS_BAUDRATE,
                               baudrate);
}

int miraculous_motor_set_baudrate(MiraMotor *motor, uint16_t baudrate)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    /* 校验支持的波特率 (依据固件 fdcan.c baudrateTable) */
    switch (baudrate) {
    case 50: case 100: case 125:
    case 250: case 500: case 800: case 1000:
        break;
    default:
        return MRC_ERROR_INVALID_PARAM;
    }
    return motor_sdo_write_u16(motor,
                                CIA402_OD_SERVO_PARAMETERS,
                                CIA402_OD_SERVO_PARAMETERS_BAUDRATE,
                                baudrate);
}

/*----------------------------------------------------------------------------
 * 心跳周期 (0x1017)
 *----------------------------------------------------------------------------*/

int miraculous_motor_get_heartbeat(MiraMotor *motor, uint16_t *period_ms)
{
    if (!motor || !period_ms) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_read_u16(motor, 0x1017, 0, period_ms);
}

int miraculous_motor_set_heartbeat(MiraMotor *motor, uint16_t period_ms)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return motor_sdo_write_u16(motor, 0x1017, 0, period_ms);
}

/*----------------------------------------------------------------------------
 * 底层 SDO 访问
 *----------------------------------------------------------------------------*/

int miraculous_motor_sdo_read(MiraMotor *motor,
                               uint16_t index, uint8_t subindex,
                               void *data, uint8_t *len)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_sdo_read(motor->co, motor->node_id,
                                   index, subindex, data, len,
                                   MOTION_SDO_TIMEOUT_MS);
}

int miraculous_motor_sdo_write(MiraMotor *motor,
                                uint16_t index, uint8_t subindex,
                                const void *data, uint8_t len)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    return miraculous_co_sdo_write(motor->co, motor->node_id,
                                    index, subindex, data, len,
                                    MOTION_SDO_TIMEOUT_MS);
}

/*----------------------------------------------------------------------------
 * 异步 SDO 读写
 *----------------------------------------------------------------------------*/

/** 分配异步任务节点 */
static MiraSdoAsyncTask* async_sdo_alloc(MiraMotor *motor)
{
    MiraSdoAsyncTask *t = calloc(1, sizeof(MiraSdoAsyncTask));
    if (!t) return NULL;

    pthread_mutex_lock(&motor->sdo_async_lock);
    t->tid = motor->sdo_async_tid_seed++;
    t->next = motor->sdo_async_list;
    motor->sdo_async_list = t;
    pthread_mutex_unlock(&motor->sdo_async_lock);
    return t;
}

/** 释放异步任务 (从链表移除) */
static void async_sdo_free(MiraMotor *motor, MiraSdoAsyncTask *task)
{
    if (!task) return;
    pthread_mutex_lock(&motor->sdo_async_lock);
    MiraSdoAsyncTask **p = &motor->sdo_async_list;
    while (*p) {
        if (*p == task) {
            *p = task->next;
            free(task);
            break;
        }
        p = &(*p)->next;
    }
    pthread_mutex_unlock(&motor->sdo_async_lock);
}

int miraculous_motor_sdo_read_async(MiraMotor *motor, uint16_t idx,
                                     uint8_t subidx, MiraSdoCallback cb,
                                     void *user_data, int *tid_out)
{
    if (!motor || !cb || !tid_out) return MRC_ERROR_INVALID_PARAM;

    MiraSdoAsyncTask *task = async_sdo_alloc(motor);
    if (!task) return MRC_ERROR_RESOURCE_BUSY;

    task->index = idx;
    task->subindex = subidx;
    task->is_write = false;
    task->cb = cb;
    task->user_data = user_data;
    clock_gettime(CLOCK_MONOTONIC, &task->expire_ts);
    task->expire_ts.tv_sec += 1; /* 默认 1s 超时 */

    uint8_t req[8] = { CO_SDO_CCS_UPLOAD_INITIATE,
                       (uint8_t)(idx & 0xFF), (uint8_t)((idx >> 8) & 0xFF),
                       subidx, 0, 0, 0, 0 };
    int ret = miraculous_can_send(miraculous_co_get_can(motor->co), 
                                   CO_COB_SDO_RX + motor->node_id, req, 8);
    if (ret < 0) {
        async_sdo_free(motor, task);
        return ret;
    }

    *tid_out = task->tid;
    return MRC_SUCCESS;
}

int miraculous_motor_sdo_write_async(MiraMotor *motor, uint16_t idx,
                                      uint8_t subidx, const void *data,
                                      uint8_t len, MiraSdoCallback cb,
                                      void *user_data, int *tid_out)
{
    if (!motor || !cb || !tid_out || len > 4)
        return MRC_ERROR_INVALID_PARAM;

    MiraSdoAsyncTask *task = async_sdo_alloc(motor);
    if (!task) return MRC_ERROR_RESOURCE_BUSY;

    task->index = idx;
    task->subindex = subidx;
    task->is_write = true;
    task->cb = cb;
    task->user_data = user_data;
    clock_gettime(CLOCK_MONOTONIC, &task->expire_ts);
    task->expire_ts.tv_sec += 1;

    uint8_t req[8];
    memset(req, 0, 8);
    req[0] = CO_SDO_CCS_DOWNLOAD_INITIATE | CO_SDO_EXPEDITED
           | CO_SDO_SIZE_INDICATED | (uint8_t)((4 - len) << 2);
    req[1] = (uint8_t)(idx & 0xFF);
    req[2] = (uint8_t)((idx >> 8) & 0xFF);
    req[3] = subidx;
    memcpy(&req[4], data, len);

    int ret = miraculous_can_send(miraculous_co_get_can(motor->co),
                                   CO_COB_SDO_RX + motor->node_id, req, 8);
    if (ret < 0) {
        async_sdo_free(motor, task);
        return ret;
    }

    *tid_out = task->tid;
    return MRC_SUCCESS;
}

/*----------------------------------------------------------------------------
 * 参数存储 / 恢复默认 (0x1010 / 0x1011)
 * Flash 操作耗时较长，超时设为 100ms
 *----------------------------------------------------------------------------*/
#define SDO_PERSIST_TIMEOUT_MS  300

int miraculous_motor_save_config(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    /* CiA 301 标准签名: "save" = 0x65766173 */
    const uint32_t save_sig = 0x65766173U;
    int ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_STORE_PARAMETERS, 1,
                           &save_sig, SDO_PERSIST_TIMEOUT_MS);
    if (ret < 0) return ret;
    printf("[motor] node %d: config saved to non-volatile memory\n",
           motor->node_id);
    return MRC_SUCCESS;
}

int miraculous_motor_restore_defaults(MiraMotor *motor)
{
    if (!motor) return MRC_ERROR_INVALID_PARAM;
    /* CiA 301 标准签名: "load" = 0x64616F6C */
    const uint32_t load_sig = 0x64616F6CU;
    int ret = CO_SDO_WRITE(motor->co, motor->node_id,
                           CIA402_OD_RESTORE_DEFAULTS, 1,
                           &load_sig, SDO_PERSIST_TIMEOUT_MS);
    if (ret < 0) return ret;
    printf("[motor] node %d: factory defaults restored (NMT reset may be required)\n",
           motor->node_id);
    return MRC_SUCCESS;
}
