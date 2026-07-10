/**
 * @file    miraculous_internal.h
 * @brief   内部共享类型和定义 — 仅供 SDK 源文件使用
 *
 * 注意：此头文件不应包含在公开 API 头文件中，也不应被用户代码包含。
 */
#ifndef MIRACULOUS_INTERNAL_H
#define MIRACULOUS_INTERNAL_H

#include "miraculous_sdk.h"

/*----------------------------------------------------------------------------
 * 位置单位换算 — 内部使用
 * 分辨率 = 2^BW counts/rev, BW 存储在 motor->encoder_bw 中
 *----------------------------------------------------------------------------*/
#define POS_TO_DEG(pos, bw)     ((float)(pos) * (360.0f / (float)((uint32_t)1 << (bw))))
#define POS_TO_RAD(pos, bw)     ((float)(pos) * (6.283185307179586f / (float)((uint32_t)1 << (bw))))
#define DEG_TO_POS(deg, bw)     ((int32_t)((deg) * (float)((uint32_t)1 << (bw)) / 360.0f))
#define RAD_TO_POS(rad, bw)     ((int32_t)((rad) * (float)((uint32_t)1 << (bw)) / 6.283185307179586f))

/*----------------------------------------------------------------------------
 * CANopen 预定义 COB-ID 偏移 (CiA 301) — 内部使用
 *----------------------------------------------------------------------------*/
#define CO_COB_NMT         0x000U
#define CO_COB_SYNC        0x080U
#define CO_COB_EMCY_BASE   0x080U
#define CO_COB_TIME        0x100U
#define CO_COB_TPDO1       0x180U
#define CO_COB_RPDO1       0x200U
#define CO_COB_TPDO2       0x280U
#define CO_COB_RPDO2       0x300U
#define CO_COB_TPDO3       0x380U
#define CO_COB_RPDO3       0x400U
#define CO_COB_TPDO4       0x480U
#define CO_COB_RPDO4       0x500U
#define CO_COB_SDO_TX      0x580U
#define CO_COB_SDO_RX      0x600U
#define CO_COB_HEARTBEAT   0x700U
#define CO_COB_LSS_SLV     0x7E4U
#define CO_COB_LSS_MST     0x7E5U

/*----------------------------------------------------------------------------
 * SDO 命令说明符 (CiA 301) — 内部使用
 *----------------------------------------------------------------------------*/
#define CO_SDO_CCS_DOWNLOAD_INITIATE   0x20
#define CO_SDO_CCS_DOWNLOAD_SEGMENT    0x00
#define CO_SDO_CCS_UPLOAD_INITIATE     0x40
#define CO_SDO_CCS_UPLOAD_SEGMENT      0x60
#define CO_SDO_CCS_ABORT               0x80

#define CO_SDO_SCS_DOWNLOAD_RESPONSE   0x60
#define CO_SDO_SCS_UPLOAD_RESPONSE     0x40
#define CO_SDO_SCS_ABORT               0x80

#define CO_SDO_EXPEDITED       0x02
#define CO_SDO_SIZE_INDICATED  0x01
#define CO_SDO_TOGGLE_MASK     0x10

/*----------------------------------------------------------------------------
 * 厂商特定 OD 子索引 — 内部使用
 *----------------------------------------------------------------------------*/
#define CIA402_OD_SERVO_PARAMETERS_NODEID   0x01U
#define CIA402_OD_SERVO_PARAMETERS_BAUDRATE 0x02U

/* CiA 301 标准 */
#define CIA402_OD_COMM_CYCLE_PERIOD  0x1006U   /* Communication Cycle Period */


/*----------------------------------------------------------------------------
 * 控制字/状态字位定义 — 内部使用
 *----------------------------------------------------------------------------*/
#define CIA402_CW_SWITCH_ON_BIT        0
#define CIA402_CW_ENABLE_VOLTAGE_BIT   1
#define CIA402_CW_QUICK_STOP_BIT       2
#define CIA402_CW_ENABLE_OPERATION_BIT 3
#define CIA402_CW_NEW_SET_POINT_BIT    4
#define CIA402_CW_CHANGE_SET_IMM_BIT   5
#define CIA402_CW_ABS_REL_BIT          6
#define CIA402_CW_FAULT_RESET_BIT      7
#define CIA402_CW_HALT_BIT             8

#define CIA402_CW_SWITCH_ON_MASK        (1U << CIA402_CW_SWITCH_ON_BIT)
#define CIA402_CW_ENABLE_VOLTAGE_MASK   (1U << CIA402_CW_ENABLE_VOLTAGE_BIT)
#define CIA402_CW_QUICK_STOP_MASK       (1U << CIA402_CW_QUICK_STOP_BIT)
#define CIA402_CW_ENABLE_OPERATION_MASK (1U << CIA402_CW_ENABLE_OPERATION_BIT)
#define CIA402_CW_NEW_SET_POINT_MASK    (1U << CIA402_CW_NEW_SET_POINT_BIT)
#define CIA402_CW_CHANGE_SET_IMM_MASK   (1U << CIA402_CW_CHANGE_SET_IMM_BIT)
#define CIA402_CW_ABS_REL_MASK          (1U << CIA402_CW_ABS_REL_BIT)
#define CIA402_CW_FAULT_RESET_MASK      (1U << CIA402_CW_FAULT_RESET_BIT)
#define CIA402_CW_HALT_MASK             (1U << CIA402_CW_HALT_BIT)

#define CIA402_SW_READY_TO_SWITCH_ON_BIT 0
#define CIA402_SW_SWITCHED_ON_BIT        1
#define CIA402_SW_OPERATION_ENABLED_BIT  2
#define CIA402_SW_FAULT_BIT              3
#define CIA402_SW_VOLTAGE_ENABLED_BIT    4
#define CIA402_SW_QUICK_STOP_BIT         5
#define CIA402_SW_SWITCH_ON_DISABLED_BIT 6
#define CIA402_SW_WARNING_BIT            7
#define CIA402_SW_REMOTE_BIT             9
#define CIA402_SW_TARGET_REACHED_BIT     10
#define CIA402_SW_INTERNAL_LIMIT_ACTIVE  11

#define CIA402_SW_STATE_MASK  0x006FU

/*----------------------------------------------------------------------------
 * 内部类型 (外部不可见)
 *----------------------------------------------------------------------------*/
typedef struct MiraCoMaster MiraCoMaster;
typedef struct SyncCtx_t SyncCtx_t;
typedef struct PdoCtx_t PdoCtx_t;

/*----------------------------------------------------------------------------
 * MiraMotor 结构体
 *----------------------------------------------------------------------------*/
struct MiraMotor {
    MiraCoMaster *co;         /* CANopen 主站 (含 CAN) */
    uint8_t       node_id;    /* CANopen 节点 ID */
    uint8_t       encoder_bw; /* 编码器位宽 (默认 19, 分辨率 = 2^BW counts/rev) */
    float         reduction_ratio; /* 减速比 (默认 100) */
    bool          sync_active;/* SYNC 是否在运行 (close 时需要 stop) */
    int32_t       pdo_pos;    /* TPDO 缓存的实际位置 */
    int32_t       pdo_vel;    /* TPDO 缓存的实际速度 */
    int16_t       pdo_torque; /* TPDO 缓存的实际转矩 */
    bool          pdo_valid;  /* TPDO 缓存是否有效 */
    MiraTpdoCallback tpdo_user_cb;    /* 用户 TPDO 回调 */
    void            *tpdo_user_data;  /* 用户 TPDO 回调数据 */
};

/*----------------------------------------------------------------------------
 * CANopen 主站内部 API (对外不可见)
 *----------------------------------------------------------------------------*/
MiraCoMaster* miraculous_co_init(MiraCanCtx *can_ctx, CiaBaudrate_t baudrate);
void miraculous_co_free(MiraCoMaster *co);

int  miraculous_co_nmt_send(MiraCoMaster *co, uint8_t node_id, CoNmtCommand_t cmd);

#define miraculous_co_nmt_start(co, nid) \
    miraculous_co_nmt_send(co, nid, CO_NMT_START_NODE)
#define miraculous_co_nmt_stop(co, nid) \
    miraculous_co_nmt_send(co, nid, CO_NMT_STOP_NODE)
#define miraculous_co_nmt_preop(co, nid) \
    miraculous_co_nmt_send(co, nid, CO_NMT_ENTER_PRE_OPERATIONAL)
#define miraculous_co_nmt_reset(co, nid) \
    miraculous_co_nmt_send(co, nid, CO_NMT_RESET_NODE)
#define miraculous_co_nmt_reset_comm(co, nid) \
    miraculous_co_nmt_send(co, nid, CO_NMT_RESET_COMMUNICATION)
int  miraculous_co_heartbeat_set_callback(MiraCoMaster *co, uint8_t node_id,
                                           MiraHeartbeatCallback cb, void *user_data);
int  miraculous_co_wait_state(MiraCoMaster *co, uint8_t node_id,
                               CoNmtState_t expected, int timeout_ms);

int  miraculous_co_sdo_read(MiraCoMaster *co, uint8_t node_id,
                             uint16_t index, uint8_t subindex,
                             void *data, uint8_t *data_len, int timeout_ms);
int  miraculous_co_sdo_write(MiraCoMaster *co, uint8_t node_id,
                              uint16_t index, uint8_t subindex,
                              const void *data, uint8_t data_len, int timeout_ms);

#define CO_SDO_READ(co, nid, idx, sub, val, timeout) \
    __extension__({ uint8_t _l = sizeof(*(val)); \
       int _r = miraculous_co_sdo_read(co, nid, idx, sub, val, &_l, timeout); \
       _r; })
#define CO_SDO_WRITE(co, nid, idx, sub, pval, timeout) \
    miraculous_co_sdo_write(co, nid, idx, sub, pval, sizeof(*(pval)), timeout)

int  miraculous_co_pdo_rpdo_config(MiraCoMaster *co, uint8_t node_id,
                                    uint8_t pdo_num, uint32_t cob_id,
                                    uint8_t trans_type, uint16_t event_timer_ms,
                                    uint8_t mapped_count,
                                    const uint32_t *mappings);
int  miraculous_co_pdo_tpdo_config(MiraCoMaster *co, uint8_t node_id,
                                    uint8_t pdo_num, uint32_t cob_id,
                                    uint8_t trans_type, uint8_t inhibit_time,
                                    uint16_t event_timer_ms,
                                    uint8_t mapped_count,
                                    const uint32_t *mappings);
int  miraculous_co_pdo_send(MiraCoMaster *co, uint8_t pdo_num, uint32_t cob_id,
                             const uint8_t *data, uint8_t len);
int  miraculous_co_pdo_set_tpdo_callback(MiraCoMaster *co, uint8_t node_id,
                                          uint8_t pdo_num, uint32_t cob_id,
                                          MiraTpdoCallback cb, void *user_data);

int  miraculous_co_sync_start(MiraCoMaster *co, uint32_t period_us);
int  miraculous_co_sync_stop(MiraCoMaster *co);
int  miraculous_co_sync_send_once(MiraCoMaster *co);
int  miraculous_co_sync_fd(MiraCoMaster *co);

int  miraculous_co_emcy_set_callback(MiraCoMaster *co, MiraEmcyCallback cb,
                                      void *user_data);
int  miraculous_co_poll(MiraCoMaster *co, int timeout_ms);
int  miraculous_co_bootstrap(MiraCoMaster *co, uint8_t node_id, int timeout_ms);

MiraCanCtx* miraculous_co_get_can(MiraCoMaster *co);
SyncCtx_t*  miraculous_co_get_sync(MiraCoMaster *co);
PdoCtx_t*   miraculous_co_get_pdo(MiraCoMaster *co);
void        miraculous_co_ref_inc(MiraCoMaster *co);
int         miraculous_co_ref_count(MiraCoMaster *co);

#endif /* MIRACULOUS_INTERNAL_H */
