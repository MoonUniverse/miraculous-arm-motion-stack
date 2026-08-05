/**
 * @file    miraculous_sdk.h
 * @brief   miraculous 电机驱动 SDK — 唯一对外公开头文件
 *
 * 使用方法:
 *   #include "miraculous_sdk.h"
 *
 * 本 SDK 围绕 MiraMotor 句柄设计，用户只需:
 *   1. miraculous_motor_open()  -- 打开电机 (自动创建 CAN + CANopen)
 *   2. 运动控制 (PP/PV/PT/CSP/CSV/CST/MIT)
 *   3. miraculous_motor_close() -- 关闭电机 (自动释放全部资源)
 *
 * 编译链接:
 *   gcc -o my_app my_app.c -I/path/to/include -L/path/to/lib -lmiraculous_sdk -lpthread -lrt
 */

#ifndef MIRACULOUS_SDK_H
#define MIRACULOUS_SDK_H

/*--- SDK 版本 ---*/
#define MIRACULOUS_SDK_VERSION_MAJOR  1
#define MIRACULOUS_SDK_VERSION_MINOR  1
#define MIRACULOUS_SDK_VERSION_PATCH  0
#define MIRACULOUS_SDK_VERSION        "1.1.0"


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
 /*============================================================================
 * 错误码定义
 *============================================================================*/

/*--- 通用 ---*/
typedef enum {
    MRC_SUCCESS               =  0,  /*!< 操作成功 */
    MRC_ERROR_UNKNOWN         = -1,  /*!< 未知错误 */
    MRC_ERROR_INVALID_PARAM   = -2,  /*!< 参数无效 */
    MRC_ERROR_NOT_INIT        = -3,  /*!< 未初始化 */
    MRC_ERROR_TIMEOUT         = -4,  /*!< 操作超时 */
    MRC_ERROR_NOT_SUPPORTED   = -5,  /*!< 不支持的操作 */
    MRC_ERROR_OUT_OF_MEMORY   = -6,  /*!< 内存分配失败 */

    /*--- CAN 传输层 (-10 ~ -19) ---*/
    MRC_ERROR_CAN_OPEN      = -10,  /*!< SocketCAN 打开失败 */
    MRC_ERROR_CAN_SEND      = -11,  /*!< CAN 帧发送失败 */
    MRC_ERROR_CAN_RECV      = -12,  /*!< CAN 帧接收失败 */
    MRC_ERROR_CAN_SOCKET    = -13,  /*!< Socket 操作失败 */
    MRC_ERROR_CAN_IOCTL     = -14,  /*!< ioctl 调用失败 */
    MRC_ERROR_CAN_BIND      = -15,  /*!< Socket bind 失败 */
    MRC_ERROR_CAN_TX_FULL   = -16,  /*!< 发送缓冲区满 */

    /*--- CANopen 协议层 (-20 ~ -29) ---*/
    MRC_ERROR_CO_SDO_UPLOAD_TIMEOUT   = -20,  /*!< SDO 读超时 */
    MRC_ERROR_CO_SDO_DOWNLOAD_TIMEOUT = -21,  /*!< SDO 写超时 */
    MRC_ERROR_CO_SDO_ABORT           = -22,  /*!< SDO 被从站中止 */
    MRC_ERROR_CO_SDO_TOGGLE          = -23,  /*!< SDO toggle 位错误 */
    MRC_ERROR_CO_NODE_NOT_FOUND      = -24,  /*!< 节点未找到 (无心跳) */
    MRC_ERROR_CO_NMT_FAILED          = -25,  /*!< NMT 命令失败 */
    MRC_ERROR_CO_PDO_CONFIG          = -26,  /*!< PDO 配置失败 */
    MRC_ERROR_CO_HEARTBEAT_LOST      = -27,  /*!< 心跳丢失 */
    MRC_ERROR_CO_WRONG_NMT_STATE     = -28,  /*!< 当前 NMT 状态不允许此操作 */
    MRC_ERROR_RESOURCE_BUSY           = -29,  /*!< 资源繁忙 (异步任务池满) */

    /*--- CiA402 运动控制层 (-30 ~ -39) ---*/
    MRC_ERROR_MOTION_STATE_TRANSITION = -30,  /*!< 状态转换失败 (超时) */
    MRC_ERROR_MOTION_FAULT           = -31,  /*!< 电机处于故障状态 */
    MRC_ERROR_MOTION_MODE_REJECTED   = -32,  /*!< 模式切换被拒绝 */
    MRC_ERROR_MOTION_NOT_ENABLED     = -33,  /*!< 电机未使能 (非 Operation Enabled) */
    MRC_ERROR_MOTION_QUICK_STOP      = -34,  /*!< 电机处于急停状态 */
    MRC_ERROR_MOTION_TARGET_TIMEOUT  = -35,  /*!< 目标位置/速度到达超时 */
} MiraculousError_t;

/**
 * @brief 将错误码转为可读字符串
 * @param err 错误码 (负值)
 * @return 静态字符串描述
 */
const char* mrc_strerror(int err);

/*============================================================================*
 * 公共类型定义 — CANopen 协议常量 / CiA402 常量 / 数据结构
 *============================================================================*/

/*--- CAN 波特率枚举 (kbps) ---*/
typedef enum {
    CIA_BAUDRATE_1000 = 1000,  /*!< 1000 kbps (1 Mbps) */
    CIA_BAUDRATE_800  = 800,   /*!< 800 kbps */
    CIA_BAUDRATE_500  = 500,   /*!< 500 kbps */
    CIA_BAUDRATE_250  = 250,   /*!< 250 kbps */
    CIA_BAUDRATE_125  = 125,   /*!< 125 kbps */
    CIA_BAUDRATE_100  = 100,   /*!< 100 kbps */
    CIA_BAUDRATE_50   = 50,    /*!< 50 kbps */
} CiaBaudrate_t;

/*--- NMT 命令说明符 ---*/
typedef enum {
    CO_NMT_START_NODE            = 0x01,  /*!< 启动节点 */
    CO_NMT_STOP_NODE             = 0x02,  /*!< 停止节点 */
    CO_NMT_ENTER_PRE_OPERATIONAL = 0x80,  /*!< 进入预操作 */
    CO_NMT_RESET_NODE            = 0x81,  /*!< 复位节点 */
    CO_NMT_RESET_COMMUNICATION   = 0x82,  /*!< 复位通信 */
} CoNmtCommand_t;

/*--- NMT 从站状态 ---*/
typedef enum {
    CO_NMT_STATE_INITIALISING     = 0x00,  /*!< 初始化中 */
    CO_NMT_STATE_DISCONNECTED     = 0x01,  /*!< 掉线 */
    CO_NMT_STATE_CONNECTING       = 0x02,  /*!< 连接中 */
    CO_NMT_STATE_PREPARING        = 0x02,  /*!< 准备中 */
    CO_NMT_STATE_STOPPED          = 0x04,  /*!< 已停止 */
    CO_NMT_STATE_OPERATIONAL      = 0x05,  /*!< 操作状态 */
    CO_NMT_STATE_PRE_OPERATIONAL  = 0x7F,  /*!< 预操作 */
    CO_NMT_STATE_UNKNOWN          = 0xFF,  /*!< 未知 */
} CoNmtState_t;

/*--- CiA402 对象字典索引 ---*/
typedef enum {
    /*--- 标准 CiA402 对象 ---*/
    CIA402_OD_CONTROLWORD               = 0x6040,
    CIA402_OD_STATUSWORD                = 0x6041,
    CIA402_OD_MODES_OF_OPERATION        = 0x6060,
    CIA402_OD_MODES_OF_OPERATION_DISPLAY= 0x6061,
    CIA402_OD_ERROR_CODE                = 0x603F,
    CIA402_OD_TARGET_POSITION           = 0x607A,
    CIA402_OD_ACTUAL_POSITION           = 0x6064,
    CIA402_OD_POSITION_DEMAND           = 0x6062,
    CIA402_OD_TARGET_VELOCITY           = 0x60FF,
    CIA402_OD_ACTUAL_VELOCITY           = 0x606C,
    CIA402_OD_VELOCITY_DEMAND           = 0x606B,
    CIA402_OD_TARGET_TORQUE             = 0x6071,
    CIA402_OD_ACTUAL_TORQUE             = 0x6077,
    CIA402_OD_TORQUE_DEMAND             = 0x6074,
    CIA402_OD_PROFILE_VELOCITY          = 0x6081,
    CIA402_OD_PROFILE_ACCELERATION      = 0x6083,
    CIA402_OD_PROFILE_DECELERATION      = 0x6084,
    CIA402_OD_QUICK_STOP_DECELERATION   = 0x6085,
    CIA402_OD_MOTION_PROFILE_TYPE       = 0x6086,
    CIA402_OD_TORQUE_SLOPE              = 0x6087,
    CIA402_OD_SUPPORTED_DRIVE_MODES     = 0x6502,

    /*--- 存储/恢复 (CiA 301) ---*/
    CIA402_OD_STORE_PARAMETERS          = 0x1010,
    CIA402_OD_RESTORE_DEFAULTS          = 0x1011,

    /*--- 厂商特定 OD 索引 ---*/
    CIA402_OD_SERVO_PARAMETERS          = 0x2001,
    CIA402_OD_CURRENT_LOOP_PI           = 0x2002,
    CIA402_OD_VELOCITY_LOOP_PI          = 0x2003,
    CIA402_OD_POSITION_LOOP_PI          = 0x2004,
    CIA402_OD_MIT_CONTROL               = 0x2005,
    CIA402_OD_SERVO_TEMPERATURE         = 0x2006,
    CIA402_OD_MIT_TORQUE_LIMIT          = 0x2102,
    CIA402_OD_HOME_OFFSET               = 0x607C,
    CIA402_OD_ENC_ZERO_POINT            = 0x2007,
    CIA402_OD_ENC_ZERO_KI               = 0x2008,
} Cia402OdIndex_t;

/*--- CiA402 操作模式 ---*/
typedef enum {
    CIA_MODE_NONE  = 0x00,   /*!< 无模式 */
    CIA_MODE_PP    = 0x01,   /*!< 轮廓位置模式 */
    CIA_MODE_VEL   = 0x02,   /*!< 速度模式 */
    CIA_MODE_PV    = 0x03,   /*!< 轮廓速度模式 */
    CIA_MODE_PT    = 0x04,   /*!< 轮廓转矩模式 */
    CIA_MODE_HM    = 0x06,   /*!< 归零模式 */
    CIA_MODE_IP    = 0x07,   /*!< 插补位置模式 */
    CIA_MODE_CSP   = 0x08,   /*!< 周期同步位置模式 */
    CIA_MODE_CSV   = 0x09,   /*!< 周期同步速度模式 */
    CIA_MODE_CST   = 0x0A,   /*!< 周期同步转矩模式 */
    CIA_MODE_CSTCA = 0x0B,   /*!< 周期同步转矩 (带加速) */
    CIA_MODE_DEBUG = 0x7F,   /*!< 调试模式 */
    CIA_MODE_MIT   = 0xFE,   /*!< MIT 力位混合模式 (厂商特定) */
} Cia402Mode_t;

/*--- 运动曲线类型 (0x6086) ---*/
typedef enum {
    CIA_PROFILE_LINEAR = 0,  /*!< T 型曲线 (梯形) */
    CIA_PROFILE_SCURVE = 1,  /*!< S 型曲线 (正弦) */
} CiaProfileType_t;

/**
 * @brief 位置单位枚举
 *
 * 用于 miraculous_motor_get_position_ex() 指定返回值的单位。
 */
typedef enum {
    POS_UNIT_DEGREE = 0,  /*!< 角度 (度) */
    POS_UNIT_RADIAN = 1,  /*!< 弧度 (rad) */
} PosUnit_t;

/*--- 速度侧枚举 ---*/
typedef enum {
    VEL_SIDE_MOTOR = 0,  /*!< 电机侧 */
    VEL_SIDE_LOAD  = 1,  /*!< 负载侧 (需除以减速比) */
} VelSide_t;

/*--- 速度单位枚举 ---*/
typedef enum {
    VEL_UNIT_RPM   = 0,  /*!< 转/分钟 (RPM) */
    VEL_UNIT_RAD_S = 1,  /*!< 弧度/秒 (rad/s) */
} VelUnit_t;

/*--- CiA402 PDS 状态 ---*/
typedef enum {
    CIA_STATE_NOT_READY_TO_SWITCH_ON = 0x00,
    CIA_STATE_SWITCH_ON_DISABLED     = 0x01,
    CIA_STATE_READY_TO_SWITCH_ON     = 0x02,
    CIA_STATE_SWITCHED_ON            = 0x03,
    CIA_STATE_OPERATION_ENABLED      = 0x04,
    CIA_STATE_QUICK_STOP_ACTIVE      = 0x05,
    CIA_STATE_FAULT_REACTION_ACTIVE  = 0x06,
    CIA_STATE_FAULT                  = 0x07,
} Cia402State_t;

/*--- 控制字命令 ---*/
typedef enum {
    CIA402_CW_SHUTDOWN          = 0x0006,
    CIA402_CW_SWITCH_ON         = 0x0007,
    CIA402_CW_DISABLE_VOLTAGE   = 0x0000,
    CIA402_CW_QUICK_STOP        = 0x0002,
    CIA402_CW_DISABLE_OPERATION = 0x0007,
    CIA402_CW_ENABLE_OPERATION  = 0x000F,
    CIA402_CW_FAULT_RESET       = 0x0080,
} Cia402ControlWord_t;

/*--- CiA402 错误码 ---*/
typedef enum {
    CIA_ERROR_NONE        = 0x0000,
    CIA_ERROR_GENERAL     = 0x1000,
    CIA_ERROR_CURRENT     = 0x2310,
    CIA_ERROR_VOLTAGE     = 0x2320,
    CIA_ERROR_TEMPERATURE = 0x2330,
    CIA_ERROR_DEVICEPROG  = 0x6300,
    CIA_ERROR_MONITORING  = 0xFF00,
} Cia402ErrorCode_t;


/*--- 回调函数类型 ---*/

/**
 * @brief 紧急事件 (EMCY) 回调函数类型
 *
 * 注册方式: 通过 miraculous_motor 配套的 emcy 设置接口注册。
 * 当电机发生故障时, 驱动会主动发送 EMCY 帧,
 * 回调中可获取错误码和制造商诊断数据。
 *
 * 使用示例:
 * @code{.c}
 * void my_emcy(uint8_t node_id, uint16_t error_code,
 *               uint8_t error_reg, const uint8_t *mfg_data,
 *               uint8_t mfg_len, void *user_data)
 * {
 *     printf("EMCY: node=%d code=0x%04X reg=0x%02X\n",
 *            node_id, error_code, error_reg);
 * }
 * @endcode
 *
 * @param node_id   发送 EMCY 的节点 ID
 * @param error_code 错误码 (CiA 402 标准定义, 如 0x2310=过流, 0x2320=过压)
 * @param error_reg  错误寄存器 (0x1001)
 * @param mfg_data   制造商特定诊断数据
 * @param mfg_len    制造商数据长度
 * @param user_data  用户自定义数据
 */
typedef void (*MiraEmcyCallback)(uint8_t node_id, uint16_t error_code,
                                  uint8_t error_reg, const uint8_t *mfg_data,
                                  uint8_t mfg_len, void *user_data);

/**
 * @brief 心跳超时/状态变更回调函数类型
 *
 * 注册方式: 通过 miraculous_motor_set_heartbeat_callback() 设置。
 * 当电机节点的心跳帧超时或状态切换时触发。
 *
 * 使用示例:
 * @code{.c}
 * void my_hb(uint8_t node_id, CoNmtState_t state, void *user_data)
 * {
 *     if (state == CO_NMT_STATE_OPERATIONAL)
 *         printf("node %d is ONLINE\n", node_id);
 *     else if (state == CO_NMT_STATE_DISCONNECTED)
 *         printf("node %d is OFFLINE!\n", node_id);
 * }
 *
 * miraculous_motor_set_heartbeat_callback(motor, my_hb, NULL);
 * @endcode
 *
 * @param node_id   节点 ID
 * @param state     当前 NMT 状态 (CoNmtState_t)
 * @param user_data 用户自定义数据
 */
typedef void (*MiraHeartbeatCallback)(uint8_t node_id, CoNmtState_t state,
                                       void *user_data);

/**
 * @brief TPDO 接收回调函数类型
 *
 * 注册方式: 通过 miraculous_motor_set_tpdo_callback() 设置。
 * 当电机通过 TPDO 主动上报数据时触发。
 *
 * 使用示例:
 * @code{.c}
 * void my_tpdo(uint8_t node_id, uint8_t pdo_num,
 *               const uint8_t *data, uint8_t len, void *user_data)
 * {
 *     printf("TPDO%u from node %d: ", pdo_num, node_id);
 *     for (int i = 0; i < len; i++)
 *         printf("%02X ", data[i]);
 *     printf("\n");
 * }
 *
 * miraculous_motor_set_tpdo_callback(motor, my_tpdo, NULL);
 * @endcode
 *
 * @param node_id  节点 ID
 * @param pdo_num  PDO 编号 (1~4)
 * @param data     PDO 数据
 * @param len      PDO 数据长度 (字节)
 * @param user_data 用户自定义数据
 */
typedef void (*MiraTpdoCallback)(uint8_t node_id, uint8_t pdo_num,
                                  const uint8_t *data, uint8_t len,
                                  void *user_data);

/*============================================================================*
 * CAN 传输层 API (高级用法, 一般用不到)
 *
 * 大多数用户只需使用 CiA402 运动控制 API, 无需直接调用 CAN 传输层函数。
 * 这些接口面向需要自定义 CAN 帧收发的高级用户。
 *============================================================================*/

typedef struct MiraCanCtx MiraCanCtx;

/**
 * @brief 打开 CAN 接口并初始化
 *
 * @param ifname   CAN 接口名称, 如 "can0"
 * @param baudrate 波特率 (kbps), 传入 0 表示不修改当前波特率
 * @return CAN 上下文句柄, 失败返回 NULL
 */
MiraCanCtx* miraculous_can_open(const char *ifname, CiaBaudrate_t baudrate);

/**
 * @brief 关闭 CAN 接口并释放所有资源
 *
 * @param ctx CAN 上下文句柄
 */
void miraculous_can_close(MiraCanCtx *ctx);

/**
 * @brief 设置 CAN 接口波特率
 *
 * @param ctx      CAN 上下文句柄
 * @param baudrate 波特率 (kbps), 如 1000, 500, 250 等
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_can_set_bitrate(MiraCanCtx *ctx, CiaBaudrate_t baudrate);

/**
 * @brief 获取 CAN socket 文件描述符
 *
 * @param ctx CAN 上下文句柄
 * @return socket fd (>=0), 无效上下文返回 -1
 */
int miraculous_can_fd(MiraCanCtx *ctx);
int miraculous_can_get_epoll_fd(MiraCanCtx *ctx);

/**
 * @brief 发送标准 CAN 帧
 *
 * @param ctx    CAN 上下文句柄
 * @param can_id CAN ID (标准帧 11 位或扩展帧 29 位)
 * @param data   数据缓冲区
 * @param len    数据长度, 不超过 8 字节 (CAN) 或 64 字节 (CAN FD)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_can_send(MiraCanCtx *ctx, uint32_t can_id,
                        const uint8_t *data, uint8_t len);

/**
 * @brief 同步方式接收匹配指定 ID 的 CAN 帧 (带超时)
 *
 * @param ctx       CAN 上下文句柄
 * @param can_id    期望的 CAN ID (仅接收该 ID 的帧)
 * @param data_out  接收数据缓冲区
 * @param len_out   输出: 实际接收的数据长度
 * @param timeout_ms 超时时间, 单位 ms, 0 表示无限等待
 * @return 成功返回 0, 超时返回负值
 */
int miraculous_can_recv_timeout(MiraCanCtx *ctx, uint32_t can_id,
                                 uint8_t *data_out, uint8_t *len_out,
                                 int timeout_ms);

/**
 * @brief 轮询 CAN 总线事件 (处理接收回调)
 *
 * @param ctx       CAN 上下文句柄
 * @param timeout_ms 超时时间, 单位 ms
 * @return 处理的事件数
 */
int miraculous_can_poll(MiraCanCtx *ctx, int timeout_ms);

/**
 * @brief 设置 CAN 帧接收回调 (异步事件驱动模式)
 *
 * 注册的回调将在调用 miraculous_can_poll() 或接收线程中,
 * 收到匹配的 CAN 帧时被调用。
 *
 * @param ctx       CAN 上下文句柄
 * @param callback  回调函数, 设置为 NULL 可取消之前的注册
 * @param user_data 用户自定义数据, 透传给回调
 * @return 成功返回 0, 失败返回负值
 */
int miraculous_can_set_recv_callback(MiraCanCtx *ctx,
                                     void (*callback)(uint32_t can_id,
                                                       const uint8_t *data,
                                                       uint8_t len,
                                                       void *user_data),
                                     void *user_data);

/*============================================================================*
 * CiA402 运动控制 API
 *============================================================================*/

typedef struct MiraMotor MiraMotor;  /*!< 电机控制句柄 (不透明类型) */

/*--- 生命周期 ---*/

/**
 * @brief 打开电机 (一键创建 CAN + CANopen + 电机句柄)
 * @param ifname   CAN 接口名称, 如 "can0"
 * @param baudrate 波特率 (kbps), 0=不修改
 * @param node_id  CANopen 节点 ID (1~127)
 * @return 电机句柄, 失败返回 NULL
 *
 * 内部自动完成 CAN 和 CANopen 初始化, 关闭时调用 miraculous_motor_close()
 * 自动释放全部资源。多电机场景下每条总线需各自调用 motor_open。
 */
MiraMotor* miraculous_motor_open(const char *ifname,
                                  CiaBaudrate_t baudrate,
                                  uint8_t node_id);

/**
 * @brief 关闭电机并释放所有资源 (含 CAN + CANopen)
 * @param motor 电机句柄
 */
void miraculous_motor_close(MiraMotor *motor);

/**
 * @brief 获取 SDK 版本信息
 *
 * @return 版本字符串, 如 "1.1.0"
 */
const char* miraculous_sdk_version(void);

/**
 * @brief 获取 SDK 编译时间
 *
 * @return 编译时间字符串, 如 "Jun 30 2026 10:30:00"
 */
const char* miraculous_sdk_build_time(void);

/**
 * @brief 一键初始化电机
 *
 * 执行 NMT Reset 节点, 等待节点重新上线进入 Operational 状态,
 * 使电机具备 PDS 状态机操作条件。
 *
 * @param motor     电机句柄
 * @param timeout_ms 等待超时, 单位 ms
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_bootstrap(MiraMotor *motor, int timeout_ms);

/**
 * @brief 向电机节点发送 NMT 命令
 *
 * @param motor 电机句柄
 * @param cmd   NMT 命令 (如 CO_NMT_STOP_NODE / CO_NMT_RESET_NODE 等)
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * miraculous_motor_nmt_send(motor, CO_NMT_STOP_NODE);   // 停止节点
 * miraculous_motor_nmt_send(motor, CO_NMT_RESET_NODE);  // 复位节点
 * @endcode
 */
int miraculous_motor_nmt_send(MiraMotor *motor, CoNmtCommand_t cmd);

/*--- SYNC 同步帧管理 ---*/

/**
 * @brief 启动周期 SYNC 帧发送
 *
 * 使用 timerfd 高精度定时器, 以指定周期持续发送 SYNC 帧 (CAN ID=0x080)。
 * 用于 CSP/CSV/CST/MIT 等周期同步模式。
 * 重复调用会重新设置周期。
 *
 * @param motor     电机句柄
 * @param period_us SYNC 周期, 单位 us (微秒), 如 10000 = 10ms
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_sync_start(MiraMotor *motor, uint32_t period_us);

/**
 * @brief 停止周期 SYNC 帧发送
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_sync_stop(MiraMotor *motor);

/**
 * @brief 手动发送一帧 SYNC
 *
 * 在 manual 模式下, 由用户主动调用此函数发送 SYNC 帧,
 * 触发从站采样输入并更新输出。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_sync_send(MiraMotor *motor);

/*--- CiA402 PDS 状态机控制 ---*/

/**
 * @brief 关机 (Shutdown)
 *
 * 发送控制字 0x0006, 将电机从 Operation Enabled / Switched On
 * 切换到 Ready to Switch On 状态。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_shutdown(MiraMotor *motor);

/**
 * @brief 上高压 (Switch On)
 *
 * 发送控制字 0x0007, 将电机从 Ready to Switch On
 * 切换到 Switched On 状态 (高压开启但未使能)。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_switch_on(MiraMotor *motor);

/**
 * @brief 上使能 (Enable Operation)
 *
 * 发送控制字 0x000F, 将电机从 Switched On
 * 切换到 Operation Enabled 状态, 电机开始输出转矩。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_enable(MiraMotor *motor);

/**
 * @brief 下使能 (Disable Operation)
 *
 * 将电机从 Operation Enabled 切换到 Switched On 状态,
 * 切断转矩输出但保持高压供电。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_disable(MiraMotor *motor);

/**
 * @brief 断开高压 (Disable Voltage)
 *
 * 发送控制字 0x0000, 将电机切换到 Switch On Disabled 状态,
 * 切断高压供电。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_disable_voltage(MiraMotor *motor);

/**
 * @brief 急停 (Quick Stop)
 *
 * 发送控制字 0x0002, 电机按急停减速度立即减速停止,
 * 进入 Quick Stop Active 状态。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_quick_stop(MiraMotor *motor);

/**
 * @brief 故障复位 (Fault Reset)
 *
 * 发送控制字 0x0080 (上升沿), 将电机从 Fault 状态
 * 恢复到 Switch On Disabled 状态。
 * 注意: 仅在电机处于 Fault 状态时有效。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_fault_reset(MiraMotor *motor);

/**
 * @brief 一键上使能
 *
 * 自动执行 Shutdown(0x06) → Switch On(0x07) → Enable Operation(0x0F)
 * 三步序列, 将电机从任意状态切换到 Operation Enabled。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_full_enable(MiraMotor *motor);

/**
 * @brief 读取电机当前 PDS 状态
 *
 * @param motor 电机句柄
 * @param state 输出: 当前 PDS 状态 (Cia402State_t)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_state(MiraMotor *motor, Cia402State_t *state);

/**
 * @brief 读取电机状态字 (0x6041)
 *
 * @param motor 电机句柄
 * @param sw    输出: 原始状态字值 (16 位)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_statusword(MiraMotor *motor, uint16_t *sw);

/**
 * @brief 等待电机到达指定 PDS 状态 (带超时)
 *
 * @param motor      电机句柄
 * @param expected   期望到达的状态
 * @param timeout_ms 超时时间, 单位 ms, <=0 时默认 3000ms
 * @return 成功返回 0, 超时返回 MRC_ERROR_TIMEOUT,
 *         进入 Fault 返回 MRC_ERROR_MOTION_FAULT
 */
int miraculous_motor_wait_state(MiraMotor *motor, Cia402State_t expected,
                                 int timeout_ms);

/*--- 操作模式 ---*/

/**
 * @brief 设置电机操作模式
 *
 * 电机必须已在 Operation Enabled 状态下才能切换模式。
 * 设置模式后需等待驱动器确认, 可通过 get_mode 验证。
 *
 * @param motor 电机句柄
 * @param mode  操作模式 (Cia402Mode_t), 如 CIA_MODE_PP, CIA_MODE_CSP 等
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_set_mode(MiraMotor *motor, Cia402Mode_t mode);

/**
 * @brief 读取电机当前操作模式 (0x6061)
 *
 * @param motor 电机句柄
 * @param mode  输出: 当前操作模式
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_mode(MiraMotor *motor, Cia402Mode_t *mode);

/**
 * @brief 读取电机支持的操作模式位掩码 (0x6502)
 *
 * @param motor 电机句柄
 * @param modes 输出: 32 位支持模式掩码, 位号对应 Cia402Mode_t 值
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_supported_modes(MiraMotor *motor, uint32_t *modes);

/*--- PP 轮廓位置模式 ---*/

/**
 * @brief 轮廓位置控制 (PP)
 *
 * 该模式下电机内部根据给定的目标位置、速度、加/减速参数
 * 自动生成梯形/S 形轨迹并执行。调用前需通过 set_mode 设为 CIA_MODE_PP。
 *
 * @param motor      电机句柄
 * @param target_pos 目标位置, 单位脉冲
 * @param profile_vel 运行速度, 单位脉冲/s
 * @param acc         加速度, 单位脉冲/s²
 * @param dec         减速度, 单位脉冲/s²
 * @param relative    是否相对定位: true=相对, false=绝对
 * @param immediate   是否立即生效: true=打断当前运动, false=排队等待
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_pp_move(MiraMotor *motor,
                              int32_t target_pos,
                              uint32_t profile_vel,
                              uint32_t acc,
                              uint32_t dec,
                              bool relative,
                              bool immediate);

/**
 * @brief 等待 PP 模式目标到位
 *
 * 轮询状态字中的 target_reached 位, 直到电机到达目标位置或超时。
 *
 * @param motor      电机句柄
 * @param timeout_ms 超时时间, 单位 ms
 * @return 成功返回 0, 超时返回 MRC_ERROR_MOTION_TARGET_TIMEOUT
 */
int miraculous_motor_pp_wait_target(MiraMotor *motor, int timeout_ms);

/*--- PV 轮廓速度模式 ---*/

/**
 * @brief 轮廓速度控制 (PV)
 *
 * 该模式下电机以给定的目标速度匀速运行,
 * 通过加/减速斜坡平滑启动和停止。调用前需通过 set_mode 设为 CIA_MODE_PV。
 *
 * @param motor        电机句柄
 * @param target_vel   目标速度, 单位脉冲/s, 正/负值控制方向
 * @param acc          加速度, 单位脉冲/s²
 * @param dec          减速度, 单位脉冲/s²
 * @param profile_type 运动曲线类型 (CiaProfileType_t): CIA_PROFILE_LINEAR 或 CIA_PROFILE_SCURVE
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_pv_move(MiraMotor *motor,
                              int32_t target_vel,
                              uint32_t acc,
                              uint32_t dec,
                              CiaProfileType_t profile_type);

/*--- PT 轮廓转矩模式 ---*/

/**
 * @brief 轮廓转矩控制 (PT)
 *
 * 该模式下电机输出指定的目标转矩,
 * 通过转矩斜率平滑过渡。调用前需通过 set_mode 设为 CIA_MODE_PT。
 *
 * @param motor        电机句柄
 * @param target_torque 目标转矩, 单位0.01A
 * @param slope         转矩斜率, 单位1A/s
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_pt_move(MiraMotor *motor,
                              int16_t target_torque,
                              uint16_t slope);

/*--- CSP 周期同步位置模式 ---*/

/**
 * @brief 周期同步位置模式初始化 (CSP)
 *
 * 配置 PDO 映射并启动 SYNC 信号, 使电机进入周期同步位置模式。
 * 初始化后通过 csp_set_target 周期性写入目标位置。
 *
 * @param motor          电机句柄
 * @param sync_period_us SYNC 周期, 单位 us (微秒). manual=false 时有效.
 * @param manual         true=手动模式(不发SYNC), false=定时器模式
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_csp_init(MiraMotor *motor, uint32_t sync_period_us, bool manual);

/**
 * @brief CSP 模式设置目标位置
 *
 * 通过 PDO 向电机写入下一周期的目标位置。需在 csp_init 之后调用,
 * 且需以不低于 SYNC 频率的速率周期性调用。
 *
 * @param motor      电机句柄
 * @param target_pos 目标位置, 单位脉冲
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_csp_set_target(MiraMotor *motor, int32_t target_pos);

/**
 * @brief CSP 模式设置目标位置, 支持角度/弧度单位
 *
 * 将角度或弧度值转换为编码器脉冲后, 通过 PDO 写入目标位置。
 * 转换公式:
 *   脉冲数 = 角度 × 2^bw / 360°
 *   脉冲数 = 弧度 × 2^bw / 2π
 *
 * @param motor      电机句柄
 * @param target_pos 目标位置, 根据 unit 参数决定单位
 * @param unit       单位枚举: POS_UNIT_DEGREE 或 POS_UNIT_RADIAN
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * miraculous_motor_csp_set_target_ex(motor, 90.0f, POS_UNIT_DEGREE);
 * @endcode
 */
int miraculous_motor_csp_set_target_ex(MiraMotor *motor, float target_pos, PosUnit_t unit);

/*--- CSV 周期同步速度模式 ---*/

/**
 * @brief 周期同步速度模式初始化 (CSV)
 *
 * 配置 PDO 映射并启动 SYNC 信号, 使电机进入周期同步速度模式。
 * 初始化后通过 csv_set_target 周期性写入目标速度。
 * 调用前需通过 set_mode 设为 CIA_MODE_CSV。
 *
 * @param motor          电机句柄
 * @param sync_period_us SYNC 周期, 单位 us (微秒)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_csv_init(MiraMotor *motor, uint32_t sync_period_us);

/**
 * @brief CSV 模式设置目标速度
 *
 * 通过 PDO 向电机写入下一周期的目标速度。
 *
 * @param motor      电机句柄
 * @param target_vel 目标速度, 单位脉冲/s
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_csv_set_target(MiraMotor *motor, int32_t target_vel);

/*--- CST 周期同步转矩模式 ---*/

/**
 * @brief 周期同步转矩模式初始化 (CST)
 *
 * 配置 PDO 映射并启动 SYNC 信号, 使电机进入周期同步转矩模式。
 * 初始化后通过 cst_set_target 周期性写入目标转矩。
 * 调用前需通过 set_mode 设为 CIA_MODE_CST。
 *
 * @param motor          电机句柄
 * @param sync_period_us SYNC 周期, 单位 us (微秒)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_cst_init(MiraMotor *motor, uint32_t sync_period_us);

/**
 * @brief CST 模式设置目标转矩
 *
 * 通过 PDO 向电机写入下一周期的目标转矩。
 *
 * @param motor         电机句柄
 * @param target_torque 目标转矩, 单位千分之一额定转矩
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_cst_set_target(MiraMotor *motor, int16_t target_torque);

/*--- MIT 力位混合模式 (厂商特定) ---*/

/**
 * @brief MIT 模式初始化
 *
 * 配置 PDO 映射并启动 SYNC 信号, 使电机进入 MIT 力位混合模式。
 * 初始化后通过 mit_set_* 设置刚度、阻尼、转矩限位和目标位置。
 * 调用前需通过 set_mode 设为 CIA_MODE_MIT。
 *
 * @param motor          电机句柄
 * @param sync_period_us SYNC 周期, 单位 us (微秒)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_mit_init(MiraMotor *motor, uint32_t sync_period_us);

/**
 * @brief MIT 模式设置刚度系数 (Kp)
 *
 * 刚度系数控制位置跟踪的硬度, 值越大位置误差产生的恢复力越强。
 *
 * @param motor 电机句柄
 * @param kp    刚度系数 (浮点数), 无量纲
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_mit_set_stiffness(MiraMotor *motor, float kp);

/**
 * @brief MIT 模式设置阻尼系数 (Kd)
 *
 * 阻尼系数控制速度的抑制程度, 值越大运动越平稳但响应越慢。
 *
 * @param motor 电机句柄
 * @param kd    阻尼系数 (浮点数), 无量纲
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_mit_set_damping(MiraMotor *motor, float kd);

/**
 * @brief MIT 模式设置输出转矩上限
 *
 * 限制 MIT 模式下的最大输出转矩, 防止电流过载。
 *
 * @param motor 电机句柄
 * @param limit 转矩上限, 单位千分之一额定转矩
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_mit_set_torque_limit(MiraMotor *motor, int16_t limit);

/**
 * @brief MIT 模式设置目标位置
 *
 * 通过 PDO 向电机写入目标位置, 电机根据当前刚度/阻尼进行力位混合控制。
 *
 * @param motor      电机句柄
 * @param target_pos 目标位置, 单位脉冲
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_mit_set_target(MiraMotor *motor, int32_t target_pos);

/*--- 实际值读取 ---*/

/**
 * @brief 读取电机实际位置 (0x6064)
 *
 * @param motor 电机句柄
 * @param pos   输出: 实际位置, 单位脉冲
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_position(MiraMotor *motor, int32_t *pos);

/**
 * @brief 读取电机实际位置并转换单位 (0x6064)
 *
 * 根据指定的单位 (度/弧度) 返回实际位置。
 * 转换公式:
 *   角度 = 脉冲数 × 360° / 2^bw
 *   弧度 = 脉冲数 × 2π / 2^bw
 * 其中 bw 为编码器位宽, 默认 19 (524288 counts/rev),
 * 可通过 miraculous_motor_set_encoder_bw() 修改。
 *
 * @param motor 电机句柄
 * @param pos   输出: 实际位置 (float), 根据 unit 参数决定单位
 * @param unit  单位枚举: POS_UNIT_DEGREE 或 POS_UNIT_RADIAN
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * float pos_deg, pos_rad;
 * miraculous_motor_get_position_ex(motor, &pos_deg, POS_UNIT_DEGREE);
 * miraculous_motor_get_position_ex(motor, &pos_rad, POS_UNIT_RADIAN);
 * @endcode
 */
int miraculous_motor_get_position_ex(MiraMotor *motor, float *pos, PosUnit_t unit);

/**
 * @brief 设置电机目标位置并转换单位 (0x607A)
 *
 * 根据指定的单位 (度/弧度) 设置目标位置。
 * 内部将角度值转换为编码器脉冲后写入 0x607A。
 * 转换公式:
 *   脉冲数 = 角度 × 2^bw / 360°
 *   脉冲数 = 弧度 × 2^bw / 2π
 * 其中 bw 为编码器位宽, 默认 19 (524288 counts/rev)。
 *
 * @param motor 电机句柄
 * @param pos   目标位置, 根据 unit 参数决定单位
 * @param unit  单位枚举: POS_UNIT_DEGREE 或 POS_UNIT_RADIAN
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * miraculous_motor_set_target_position_ex(motor, 90.0f, POS_UNIT_DEGREE);  // 转 90°
 * miraculous_motor_set_target_position_ex(motor, 3.14159f, POS_UNIT_RADIAN); // 转 π rad
 * @endcode
 */
int miraculous_motor_set_target_position_ex(MiraMotor *motor, float pos, PosUnit_t unit);

/**
 * @brief 将当前位置设为零点 (0x607C)
 *
 * 先将 CiA402 Home Offset (0x607C) 复位为 0, 读取当前实际位置 (0x6064),
 * 再将读到的位置值写入 Home Offset。
 * 固件计算公式: ActualPosition = RawPosition - HomeOffset,
 * 因此 HomeOffset = 当前 RawPosition 即可使当前位置归零。
 *
 * 如需掉电保持, 请在调用后执行 miraculous_motor_save_config()。
 *
 * @param motor 电机句柄
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * int ret = miraculous_motor_set_zero_position(motor);
 * if (ret == 0) {
 *     printf("Zero position set\n");
 *     miraculous_motor_save_config(motor);  // 掉电保持
 * }
 * @endcode
 */
int miraculous_motor_set_zero_position(MiraMotor *motor);

/**
 * @brief 设置编码器位宽 (分辨率)
 *
 * 配置用于 miraculous_motor_get_position_ex() 位置单位转换的编码器分辨率。
 * 分辨率 = 2^bw counts/revolution。
 * 默认值为 19 (即 524288 counts/rev), 通常无需调用。
 * 仅当编码器硬件位宽与默认值不同时才需设置。
 *
 * @param motor 电机句柄
 * @param bw    编码器位宽 (有效范围 1~31)
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * miraculous_motor_set_encoder_bw(motor, 17);  // 17-bit, 131072 counts/rev
 * @endcode
 */
int miraculous_motor_set_encoder_bw(MiraMotor *motor, uint8_t bw);

/**
 * @brief 设置减速比
 *
 * 用于 miraculous_motor_get_velocity_ex() 的负载侧速度换算。
 * 负载侧速度 = 电机侧速度 / 减速比。
 * 默认值为 100。
 *
 * @param motor 电机句柄
 * @param ratio 减速比 (如 100.0)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_set_reduction_ratio(MiraMotor *motor, float ratio);

/**
 * @brief 读取电机实际速度 (0x606C)
 *
 * @param motor 电机句柄
 * @param vel   输出: 实际速度, 单位 RPM
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_velocity(MiraMotor *motor, int32_t *vel);

/**
 * @brief 读取电机实际速度并转换单位 (0x606C)
 *
 * 可指定电机侧或负载侧、RPM 或 rad/s。
 * 负载侧速度 = 电机侧速度 / 减速比。
 * 减速比默认 100, 可通过 miraculous_motor_set_reduction_ratio() 修改。
 *
 * @param motor 电机句柄
 * @param vel   输出: 实际速度 (float)
 * @param side  侧: VEL_SIDE_MOTOR(电机侧) 或 VEL_SIDE_LOAD(负载侧)
 * @param unit  单位: VEL_UNIT_RPM 或 VEL_UNIT_RAD_S
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * float rpm_motor, rads_load;
 * miraculous_motor_get_velocity_ex(motor, &rpm_motor, VEL_SIDE_MOTOR, VEL_UNIT_RPM);
 * miraculous_motor_get_velocity_ex(motor, &rads_load, VEL_SIDE_LOAD, VEL_UNIT_RAD_S);
 * @endcode
 */
int miraculous_motor_get_velocity_ex(MiraMotor *motor, float *vel,
                                      VelSide_t side, VelUnit_t unit);

/**
 * @brief 读取电机实际转矩 (0x6077)
 *
 * @param motor 电机句柄
 * @param torque 输出: 实际转矩, 单位千分之一额定转矩
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_torque(MiraMotor *motor, int16_t *torque);

/*--- 厂商特定 (温度 / PID / 波特率) ---*/

/**
 * @brief 读取电机和 MOS 管温度 (0x2006)
 *
 * @param motor      电机句柄
 * @param motor_temp 输出: 电机绕组温度, 单位 0.1°C (如 255 表示 25.5°C), 可为 NULL
 * @param mos_temp   输出: MOS 管温度, 单位 0.1°C, 可为 NULL
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_temperature(MiraMotor *motor,
                                      int16_t *motor_temp,
                                      int16_t *mos_temp);

/**
 * @brief 设置电流环 PI 参数 (0x2002)
 *
 * 实际增益 = 寄存器值 / (2^divisor)
 *
 * @param motor       电机句柄
 * @param kp_reg      Kp 寄存器值 (I16)
 * @param kp_divisor  Kp 分母指数 (U16 = 2^N)
 * @param ki_reg      Ki 寄存器值 (I16)
 * @param ki_divisor  Ki 分母指数 (U16 = 2^N)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_set_current_pi(MiraMotor *motor,
                                     int16_t kp_reg, uint16_t kp_divisor,
                                     int16_t ki_reg, uint16_t ki_divisor);

/**
 * @brief 读取电流环 PI 参数 (0x2002)
 *
 * @param motor       电机句柄
 * @param kp_reg      输出: Kp 寄存器值, 可为 NULL
 * @param kp_divisor  输出: Kp 分母指数, 可为 NULL
 * @param ki_reg      输出: Ki 寄存器值, 可为 NULL
 * @param ki_divisor  输出: Ki 分母指数, 可为 NULL
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_current_pi(MiraMotor *motor,
                                     int16_t *kp_reg, uint16_t *kp_divisor,
                                     int16_t *ki_reg, uint16_t *ki_divisor);

/**
 * @brief 设置速度环 PI 参数 (0x2003)
 *
 * 实际增益 = 寄存器值 / (2^divisor)
 *
 * @param motor       电机句柄
 * @param kp_reg      Kp 寄存器值 (I16)
 * @param kp_divisor  Kp 分母指数 (U16 = 2^N)
 * @param ki_reg      Ki 寄存器值 (I16)
 * @param ki_divisor  Ki 分母指数 (U16 = 2^N)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_set_velocity_pi(MiraMotor *motor,
                                      int16_t kp_reg, uint16_t kp_divisor,
                                      int16_t ki_reg, uint16_t ki_divisor);

/**
 * @brief 读取速度环 PI 参数 (0x2003)
 *
 * @param motor       电机句柄
 * @param kp_reg      输出: Kp 寄存器值, 可为 NULL
 * @param kp_divisor  输出: Kp 分母指数, 可为 NULL
 * @param ki_reg      输出: Ki 寄存器值, 可为 NULL
 * @param ki_divisor  输出: Ki 分母指数, 可为 NULL
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_velocity_pi(MiraMotor *motor,
                                      int16_t *kp_reg, uint16_t *kp_divisor,
                                      int16_t *ki_reg, uint16_t *ki_divisor);

/**
 * @brief 设置位置环 PI 参数 (0x2004)
 *
 * 注意: 位置环 ki_reg 类型为 U16 (不同于电流/速度环的 I16)
 * 实际增益 = 寄存器值 / (2^divisor)
 *
 * @param motor       电机句柄
 * @param kp_reg      Kp 寄存器值 (I16)
 * @param kp_divisor  Kp 分母指数 (U16 = 2^N)
 * @param ki_reg      Ki 寄存器值 (U16)
 * @param ki_divisor  Ki 分母指数 (U16 = 2^N)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_set_position_pi(MiraMotor *motor,
                                      int16_t kp_reg, uint16_t kp_divisor,
                                      uint16_t ki_reg, uint16_t ki_divisor);

/**
 * @brief 读取位置环 PI 参数 (0x2004)
 *
 * @param motor       电机句柄
 * @param kp_reg      输出: Kp 寄存器值, 可为 NULL
 * @param kp_divisor  输出: Kp 分母指数, 可为 NULL
 * @param ki_reg      输出: Ki 寄存器值, 可为 NULL
 * @param ki_divisor  输出: Ki 分母指数, 可为 NULL
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_position_pi(MiraMotor *motor,
                                      int16_t *kp_reg, uint16_t *kp_divisor,
                                      uint16_t *ki_reg, uint16_t *ki_divisor);

/**
 * @brief 读取电机波特率 (0x2001 sub2)
 *
 * @param motor    电机句柄
 * @param baudrate 输出: 波特率, 单位 kbps (如 1000, 500 等)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_baudrate(MiraMotor *motor, uint16_t *baudrate);

/**
 * @brief 设置电机波特率 (0x2001 sub2)
 *
 * 写入后需保存配置并重新上电或复位才能生效。
 * 支持的波特率: 50, 100, 125, 250, 500, 800, 1000 kbps。
 *
 * @param motor    电机句柄
 * @param baudrate 波特率, 单位 kbps
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_set_baudrate(MiraMotor *motor, uint16_t baudrate);

/*--- 心跳周期 (0x1017: Producer Heartbeat Time) ---*/

/**
 * @brief 读取电机心跳周期 (0x1017)
 *
 * @param motor     电机句柄
 * @param period_ms 输出: 心跳周期, 单位 ms, 0 表示禁用心跳
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_get_heartbeat(MiraMotor *motor, uint16_t *period_ms);

/**
 * @brief 设置电机心跳周期 (0x1017)
 *
 * 设为非零值后电机固件会以该周期定时发送心跳帧,
 * 主机可通过 heartbeat 回调监控节点存活状态。
 * 设为 0 可禁用心跳。
 *
 * @param motor     电机句柄
 * @param period_ms 心跳周期, 单位 ms, 0=禁用
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_set_heartbeat(MiraMotor *motor, uint16_t period_ms);

/*--- 参数存储/恢复 ---*/

/**
 * @brief 将当前参数保存到非易失存储器 (0x1010)
 * @param motor 电机句柄
 * @return MRC_SUCCESS 或负值错误码
 *
 * 写入 CiA 301 标准签名 "save" (0x65766173) 到对象 0x1010 sub1,
 * 触发固件将当前运行参数写入 Flash/EEPROM。
 */
int miraculous_motor_save_config(MiraMotor *motor);

/**
 * @brief 恢复出厂默认参数 (0x1011)
 * @param motor 电机句柄
 * @return MRC_SUCCESS 或负值错误码
 *
 * 写入 CiA 301 标准签名 "load" (0x64616F6C) 到对象 0x1011 sub1,
 * 触发固件将所有参数恢复为出厂默认值。
 * 注意: 恢复后通常需要 NMT Reset 节点或重新上电才能生效。
 */
int miraculous_motor_restore_defaults(MiraMotor *motor);

/*--- 底层 CANopen 访问 (高级用法) ---*/

/**
 * @brief 注册电机节点的心跳回调
 * @param motor    电机句柄
 * @param callback 心跳回调函数
 * @param user_data 用户数据
 * @return MRC_SUCCESS 或负值错误码
 */
int miraculous_motor_set_heartbeat_callback(MiraMotor *motor,
                                             MiraHeartbeatCallback callback,
                                             void *user_data);

/**
 * @brief 为总线上所有可能的节点 (1~127) 注册统一的心跳回调
 * @param motor    电机句柄
 * @param callback 心跳回调函数
 * @param user_data 用户数据
 * @return MRC_SUCCESS 或负值错误码
 */
int miraculous_motor_set_heartbeat_scan(MiraMotor *motor,
                                         MiraHeartbeatCallback callback,
                                         void *user_data);

/**
 * @brief 注册 TPDO 接收回调
 *
 * 当该电机节点通过 TPDO 主动上报数据时触发回调。
 * 与内部 TPDO 缓存 (get_position/get_velocity) 共存。
 *
 * @param motor    电机句柄
 * @param callback TPDO 回调函数, 可为 NULL 注销
 * @param user_data 用户自定义数据, 传入回调
 * @return MRC_SUCCESS 或负值错误码
 */
int miraculous_motor_set_tpdo_callback(MiraMotor *motor,
                                        MiraTpdoCallback callback,
                                        void *user_data);

/**
 * @brief 注册 EMCY 紧急事件回调
 *
 * 当电机发生故障时触发回调, 获取错误码和诊断数据。
 *
 * @param motor    电机句柄
 * @param callback EMCY 回调函数, 可为 NULL 注销
 * @param user_data 用户自定义数据, 传入回调
 * @return MRC_SUCCESS 或负值错误码
 */
int miraculous_motor_set_emcy_callback(MiraMotor *motor,
                                        MiraEmcyCallback callback,
                                        void *user_data);

/**
 * @brief 配置从站 RPDO (接收 PDO) 通信与映射参数
 *
 * 通过 SDO 配置从站指定 RPDO 的 COB-ID、传输类型和映射对象。
 * 配置流程: 禁用 → 设置传输类型 → 清空映射 → 设置映射 → 写映射数 → 启用。
 *
 * @param motor        电机句柄
 * @param pdo_num      PDO 编号 (1~4)
 * @param cob_id       COB-ID (如 0x200+node_id)
 * @param trans_type   传输类型:
 *                       0   = 同步非周期 (远程帧触发)
 *                       1   = 同步周期 (每个 SYNC 触发一次)
 *                       2~240 = 每 N 个 SYNC 触发一次
 *                       254 = 异步 (事件/制造商特定)
 *                       255 = 异步 (事件/设备特定)
 * @param event_timer_ms 事件定时器 (sub5, 单位 ms, 0=禁用);
 *                        异步模式下定时触发 PDO 发送
 * @param mapped_count 映射对象个数 (0~8)
 * @param mappings     映射数组, 每个元素为 32 位映射值, 格式:
 *                      Bit 31..16: 对象索引 (如 0x6040)
 *                      Bit 15.. 8: 子索引 (通常 0x00)
 *                      Bit  7.. 0: 数据长度 (位)
 *
 *                      常用数据长度编码:
 *                      长度 (位) | 长度 (字节) | 编码值
 *                      ---------|------------|------
 *                       8       | 1 字节     | 0x08
 *                      16       | 2 字节     | 0x10
 *                      24       | 3 字节     | 0x18
 *                      32       | 4 字节     | 0x20
 *                      40       | 5 字节     | 0x28
 *                      48       | 6 字节     | 0x30
 *                      56       | 7 字节     | 0x38
 *                      64       | 8 字节     | 0x40
 *
 *                      常用映射值:
 *                      值                    | 对象          | 长度
 *                      ----------------------|---------------|-----
 *                      0x60400010            | 控制字        | 16位
 *                      0x60410010            | 状态字        | 16位
 *                      0x60600008            | 操作模式      | 8位
 *                      0x60610008            | 模式显示      | 8位
 *                      0x60640020            | 实际位置      | 32位
 *                      0x606C0020            | 实际速度      | 32位
 *                      0x60770010            | 实际转矩      | 16位
 *                      0x607A0020            | 目标位置      | 32位
 *                      0x60FF0020            | 目标速度      | 32位
 *                      0x60710010            | 目标转矩      | 16位
 *                      0x60810020            | 轮廓速度      | 32位
 *                      0x603F0010            | 错误码        | 16位
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * uint32_t maps[] = { 0x60400010, 0x607A0020 }; // controlword + target pos
 * miraculous_motor_rpdo_config(motor, 1, 0x200 + node_id, 255, 0, 2, maps);
 * @endcode
 */
int miraculous_motor_rpdo_config(MiraMotor *motor, uint8_t pdo_num,
                                  uint32_t cob_id, uint8_t trans_type,
                                  uint16_t event_timer_ms,
                                  uint8_t mapped_count,
                                  const uint32_t *mappings);

/**
 * @brief 发送 RPDO 数据帧
 *
 * 直接向总线发送一帧 RPDO 数据, 从站根据 PDO 映射解析。
 *
 * @param motor   电机句柄
 * @param pdo_num PDO 编号 (1~4)
 * @param data    数据缓冲区
 * @param len     数据长度 (字节)
 * @return 成功返回 0, 失败返回其他
 */
int miraculous_motor_rpdo_send(MiraMotor *motor, uint8_t pdo_num,
                                const uint8_t *data, uint8_t len);

/**
 * @brief 配置从站 TPDO (发送 PDO) 通信与映射参数
 *
 * 通过 SDO 配置从站指定 TPDO 的 COB-ID、传输类型、禁止时间和映射对象。
 * 配置流程: 禁用 → 设置传输类型 → 清空映射 → 设置映射 → 写映射数 → 设禁止时间 → 设事件定时器 → 启用。
 *
 * 注意: 若传输类型使用同步模式 (0~240), 从站需收到 SYNC 帧才会触发上报,
 * 主站应调用 miraculous_motor_sync_start() 启动 SYNC 定时器。
 *
 * @param motor        电机句柄
 * @param pdo_num      PDO 编号 (1~4)
 * @param cob_id       COB-ID (如 0x180+node_id)
 * @param trans_type   传输类型:
 *                       0   = 同步非周期 (远程帧触发)
 *                       1   = 同步周期 (每个 SYNC 触发一次)
 *                       2~240 = 每 N 个 SYNC 触发一次
 *                       254 = 异步 (事件/制造商特定)
 *                       255 = 异步 (事件/设备特定)
 * @param inhibit_time 禁止时间 (sub3, 单位 100μs, 0=禁用)
 * @param event_timer_ms 事件定时器 (sub5, 单位 ms, 0=禁用)
 * @param mapped_count 映射对象个数 (0~8)
 * @param mappings     映射数组, 格式同 rpdo_config
 * @return 成功返回 0, 失败返回其他
 *
 * 使用示例:
 * @code{.c}
 * uint32_t maps[] = { 0x60640020, 0x606C0020 }; // actual pos + vel
 * miraculous_motor_tpdo_config(motor, 1, 0x180 + node_id, 255, 0, 2, maps);
 * @endcode
 */
int miraculous_motor_tpdo_config(MiraMotor *motor, uint8_t pdo_num,
                                  uint32_t cob_id, uint8_t trans_type,
                                  uint8_t inhibit_time, uint16_t event_timer_ms,
                                  uint8_t mapped_count,
                                  const uint32_t *mappings);

/**
 * @brief 轮询电机所属总线的 CANopen 事件
 * @param motor      电机句柄
 * @param timeout_ms 超时毫秒
 * @return 处理的事件数
 */

/*--- 底层 SDO 读写 (高级用法) ---*/

/**
 * @brief SDO 读从站对象字典
 *
 * 直接通过 SDO 协议读取电机对象字典中指定索引/子索引的值。
 * 适用于 SDK 未封装的专有对象访问。
 *
 * @param motor   电机句柄
 * @param index   对象字典主索引 (如 0x6041, 0x6064)
 * @param subindex 子索引
 * @param data    输出缓冲区
 * @param len     输入: 缓冲区大小; 输出: 实际读取的字节数
 * @return 成功返回 0, 超时返回 MRC_ERROR_CO_SDO_UPLOAD_TIMEOUT
 */
int miraculous_motor_sdo_read(MiraMotor *motor,
                               uint16_t index, uint8_t subindex,
                               void *data, uint8_t *len);

/**
 * @brief SDO 写从站对象字典
 *
 * 直接通过 SDO 协议写入电机对象字典中指定索引/子索引的值。
 * 写入 0x1010 (save) / 0x1011 (restore) 等特殊对象时请使用专用接口。
 *
 * @param motor   电机句柄
 * @param index   对象字典主索引
 * @param subindex 子索引
 * @param data    待写入数据缓冲区
 * @param len     数据长度 (字节)
 * @return 成功返回 0, 超时返回 MRC_ERROR_CO_SDO_DOWNLOAD_TIMEOUT
 */
int miraculous_motor_sdo_write(MiraMotor *motor,
                                uint16_t index, uint8_t subindex,
                                const void *data, uint8_t len);

/**
 * @brief 获取电机 CANopen 节点 ID
 *
 * @param motor 电机句柄
 * @return 节点 ID (1~127)
 */
uint8_t miraculous_motor_get_node_id(MiraMotor *motor);

/**
 * @brief 获取电机关联的 CAN 上下文
 *
 * @param motor 电机句柄
 * @return CAN 上下文指针, 失败返回 NULL
 */
MiraCanCtx* miraculous_motor_get_can_ctx(MiraMotor *motor);

/*--- 异步 SDO 回调 / API ---*/

/** SDO 异步操作结果状态 */
typedef enum {
    MIRA_SDO_OK      = 0,  /**< SDO 成功 */
    MIRA_SDO_TIMEOUT = 1,  /**< SDO 超时 */
    MIRA_SDO_ABORT   = 2,  /**< SDO 被从站中止 */
} MiraSdoStatus_t;

/**
 * @brief SDO 异步回调函数
 * @param motor    电机句柄
 * @param tid      事务 ID
 * @param index    OD 索引
 * @param subindex OD 子索引
 * @param status   结果状态
 * @param data     响应数据
 * @param len      数据长度
 * @param user_data 用户注册参数
 */
typedef void (*MiraSdoCallback)(MiraMotor *motor, int tid,
                                 uint16_t index, uint8_t subindex,
                                 MiraSdoStatus_t status,
                                 const uint8_t *data, uint8_t len,
                                 void *user_data);

/**
 * @brief 异步 SDO 读 — 立即返回, 响应到达时触发回调
 */
int miraculous_motor_sdo_read_async(MiraMotor *motor, uint16_t idx,
                                     uint8_t subidx, MiraSdoCallback cb,
                                     void *user_data, int *tid_out);

/**
 * @brief 异步 SDO 写 — 立即返回, 响应到达时触发回调
 */
int miraculous_motor_sdo_write_async(MiraMotor *motor, uint16_t idx,
                                      uint8_t subidx, const void *data,
                                      uint8_t len, MiraSdoCallback cb,
                                      void *user_data, int *tid_out);

#ifdef __cplusplus
}
#endif

#endif /* MIRACULOUS_SDK_H */
