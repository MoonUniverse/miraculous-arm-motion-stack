# SDK API 概览

SDK 对外只暴露一个头文件 `miraculous_sdk.h`，包含以下三层的全部公开 API。

## 0. SDK 版本

| API / 宏 | 说明 |
| --- | --- |
| `MIRACULOUS_SDK_VERSION` | 版本号字符串，如 `"1.1.0"` |
| `MIRACULOUS_SDK_VERSION_MAJOR` | 主版本号 |
| `MIRACULOUS_SDK_VERSION_MINOR` | 次版本号 |
| `MIRACULOUS_SDK_VERSION_PATCH` | 修订号 |
| `miraculous_sdk_version()` | 运行时获取版本号 |
| `miraculous_sdk_build_time()` | 运行时获取编译时间 |

## 1. CAN 传输层

封装 Linux SocketCAN，提供 CAN 帧收发和事件循环。

| 函数 | 说明 |
| --- | --- |
| `miraculous_can_open()` | 打开 CAN 接口 |
| `miraculous_can_close()` | 关闭 CAN 接口 |
| `miraculous_can_fd()` | 获取底层 socket fd |
| `miraculous_can_send()` | 发送 CAN 标准帧 |
| `miraculous_can_send_frame()` | 发送原始 CAN 帧 |
| `miraculous_can_set_recv_callback()` | 注册接收回调 |
| `miraculous_can_recv_timeout()` | 阻塞等待匹配帧 |
| `miraculous_can_poll()` | 主循环轮询 |

## 2. CANopen 主站层

提供完整的 CANopen 主站功能：NMT、SDO、PDO、SYNC、EMCY、Heartbeat。

### NMT 网络管理

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_nmt_send()` | 通过电机发送 NMT 命令 |
| `miraculous_co_nmt_send()` | 发送 NMT 命令 |
| `miraculous_co_nmt_start()` | 启动节点 |
| `miraculous_co_nmt_stop()` | 停止节点 |
| `miraculous_co_nmt_preop()` | 进入预操作 |
| `miraculous_co_nmt_reset()` | 复位节点 |
| `miraculous_co_nmt_reset_comm()` | 复位通信 |

### Heartbeat 心跳监控

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_set_heartbeat_callback()` | 注册心跳回调 |
| `miraculous_motor_set_heartbeat_scan()` | 扫描总线心跳 |
| `miraculous_co_heartbeat_set_callback()` | 注册心跳回调 |
| `miraculous_co_wait_state()` | 等待节点达到指定状态 |

### SDO 服务数据对象

| 函数 | 说明 |
| --- | --- |
| `miraculous_co_sdo_read()` | SDO 读 |
| `miraculous_co_sdo_write()` | SDO 写 |
| `miraculous_motor_sdo_read()` | 通过电机句柄读 OD |
| `miraculous_motor_sdo_write()` | 通过电机句柄写 OD |
| `CO_SDO_READ` | 便捷宏，自动推断长度 |
| `CO_SDO_WRITE` | 便捷宏，自动推断长度 |

### PDO 过程数据对象

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_rpdo_config()` | 配置 RPDO (COB-ID/类型/映射) |
| `miraculous_motor_rpdo_send()` | 发送 RPDO 数据 |
| `miraculous_motor_tpdo_config()` | 配置 TPDO (COB-ID/类型/禁止时间/映射) |
| `miraculous_motor_set_tpdo_callback()` | 注册 TPDO 接收回调 |
| `miraculous_co_pdo_rpdo_config()` | 配置 RPDO (内部) |
| `miraculous_co_pdo_send()` | 发送 PDO 数据 |
| `miraculous_co_pdo_set_tpdo_callback()` | 注册 TPDO 接收回调 |

### SYNC 同步帧

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_sync_start()` | 启动周期 SYNC |
| `miraculous_motor_sync_stop()` | 停止 SYNC |
| `miraculous_motor_sync_send()` | 立即发送一帧 SYNC |
| `miraculous_co_sync_start()` | 启动周期 SYNC |
| `miraculous_co_sync_stop()` | 停止 SYNC |
| `miraculous_co_sync_send_once()` | 立即发送一帧 SYNC |
| `miraculous_co_sync_fd()` | 获取 SYNC timerfd |

### EMCY 紧急报文

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_set_emcy_callback()` | 注册 EMCY 回调 |
| `miraculous_co_emcy_set_callback()` | 注册 EMCY 回调 |

### 主循环和 Bootstrapping

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_poll()` | 电机轮询 |
| `miraculous_co_poll()` | 主站轮询（驱动异步操作） |
| `miraculous_motor_bootstrap()` | 一键初始化节点 |

## 3. CiA402 运动控制层

提供电机生命周期管理、状态机控制和多种运动控制模式。

### 枚举类型

| 枚举 | 说明 |
| --- | --- |
| `PosUnit_t` | 位置单位: `POS_UNIT_DEGREE`(度), `POS_UNIT_RADIAN`(弧度) |
| `VelSide_t` | 速度侧: `VEL_SIDE_MOTOR`(电机侧), `VEL_SIDE_LOAD`(负载侧) |
| `VelUnit_t` | 速度单位: `VEL_UNIT_RPM`(转/分钟), `VEL_UNIT_RAD_S`(弧度/秒) |
| `Cia402Mode_t` | 操作模式: PP/PV/PT/HM/IP/CSP/CSV/CST/MIT/None |
| `Cia402State_t` | PDS 状态机状态 |
| `CiaProfileType_t` | 轮廓曲线类型: LINEAR(梯形), SCURVE(S形) |

### 生命周期

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_open()` | 打开电机（一键创建 CAN + CANopen + 电机句柄） |
| `miraculous_motor_open_with_master()` | 使用已有 CANopen 主站打开电机（高级用法） |
| `miraculous_motor_close()` | 关闭电机并释放所有资源（含 CAN/CANopen） |
| `miraculous_motor_bootstrap()` | 一键初始化电机 |

### 状态机控制

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_shutdown()` | Shutdown |
| `miraculous_motor_switch_on()` | Switch On |
| `miraculous_motor_enable()` | Enable Operation |
| `miraculous_motor_disable()` | Disable Operation |
| `miraculous_motor_disable_voltage()` | Disable Voltage |
| `miraculous_motor_quick_stop()` | Quick Stop |
| `miraculous_motor_fault_reset()` | Fault Reset |
| `miraculous_motor_full_enable()` | 完整使能序列 |
| `miraculous_motor_get_state()` | 获取 PDS 状态 |
| `miraculous_motor_get_statusword()` | 获取状态字 |
| `miraculous_motor_wait_state()` | 等待指定状态 |

### 操作模式

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_set_mode()` | 设置操作模式 |
| `miraculous_motor_get_mode()` | 获取当前模式 |
| `miraculous_motor_get_supported_modes()` | 获取支持的模式 |

### 轮廓模式

| 函数 | 模式 | 说明 |
| --- | --- | --- |
| `miraculous_motor_pp_move()` | PP | 轮廓位置控制 |
| `miraculous_motor_pp_wait_target()` | PP | 等待目标到达 |
| `miraculous_motor_pv_move()` | PV | 轮廓速度控制 |
| `miraculous_motor_pt_move()` | PT | 轮廓转矩控制 |

### 周期同步模式

| 函数 | 模式 | 说明 |
| --- | --- | --- |
| `miraculous_motor_csp_init()` | CSP | 初始化 CSP |
| `miraculous_motor_csp_set_target()` | CSP | 设置目标位置 (脉冲) |
| `miraculous_motor_csp_set_target_ex()` | CSP | 设置目标位置 (度/弧度) |
| `miraculous_motor_csv_init()` | CSV | 初始化 CSV |
| `miraculous_motor_csv_set_target()` | CSV | 设置目标速度 |
| `miraculous_motor_cst_init()` | CST | 初始化 CST |
| `miraculous_motor_cst_set_target()` | CST | 设置目标转矩 |
| `miraculous_motor_mit_init()` | MIT | 初始化 MIT 阻抗控制 |
| `miraculous_motor_mit_set_stiffness()` | MIT | 设置刚度系数 Kp |
| `miraculous_motor_mit_set_damping()` | MIT | 设置阻尼系数 Kd |
| `miraculous_motor_mit_set_torque_limit()` | MIT | 设置力矩限制 |
| `miraculous_motor_mit_set_target()` | MIT | 设置目标位置 |

### 实际值读取 (TPDO 缓存)

位置、速度、转矩均通过 TPDO 缓存读取，不依赖 SDO。每个 SYNC 周期自动更新。

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_get_position()` | 读取实际位置 (脉冲) |
| `miraculous_motor_get_position_ex()` | 读取实际位置 (度/弧度) |
| `miraculous_motor_get_velocity()` | 读取实际速度 (RPM) |
| `miraculous_motor_get_velocity_ex()` | 读取实际速度 (电机侧/负载侧, RPM/rad/s) |
| `miraculous_motor_get_torque()` | 读取实际转矩 |

### 设置位置与零点

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_set_target_position_ex()` | 设置目标位置 (0x607A, 度/弧度) |
| `miraculous_motor_set_zero_position()` | 将当前位置设为零点 (0x607C) |

### 配置参数

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_set_encoder_bw()` | 设置编码器位宽 (默认 19) |
| `miraculous_motor_set_reduction_ratio()` | 设置减速比 (默认 100) |
| `miraculous_motor_save_config()` | 保存参数到持久存储 (0x1010) |
| `miraculous_motor_restore_defaults()` | 恢复出厂默认 (0x1011) |

### 厂商特定功能

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_get_baudrate()` | 读取 CAN 波特率 (0x2001 sub2) |
| `miraculous_motor_set_baudrate()` | 设置 CAN 波特率 |
| `miraculous_motor_get_heartbeat()` | 读取心跳周期 (0x1017) |
| `miraculous_motor_set_heartbeat()` | 设置心跳周期 |
| `miraculous_motor_get_temperature()` | 读取温度（电机 + MOS） |
| `miraculous_motor_set_current_pi()` | 设置电流环 PI (kp_reg/ki_reg + divisor) |
| `miraculous_motor_get_current_pi()` | 读取电流环 PI 参数 |
| `miraculous_motor_set_velocity_pi()` | 设置速度环 PI 参数 |
| `miraculous_motor_get_velocity_pi()` | 读取速度环 PI 参数 |
| `miraculous_motor_set_position_pi()` | 设置位置环 PI 参数 |
| `miraculous_motor_get_position_pi()` | 读取位置环 PI 参数 |

### 底层 SDO 访问

| 函数 | 说明 |
| --- | --- |
| `miraculous_motor_sdo_read()` | 通过电机句柄读 OD |
| `miraculous_motor_sdo_write()` | 通过电机句柄写 OD |
| `miraculous_motor_get_bus()` | 获取 CAN 总线句柄 |
| `miraculous_motor_get_node_id()` | 获取电机 Node-ID |

## 错误码

所有 API 返回 `int`，0 = 成功，负值 = 错误。使用 `mrc_strerror()` 获取描述。

| 范围 | 层 | 示例 |
| --- | --- | --- |
| -1 ~ -6 | 通用 | `MRC_ERROR_TIMEOUT`, `MRC_ERROR_INVALID_PARAM` |
| -10 ~ -16 | CAN 传输层 | `MRC_ERROR_CAN_OPEN`, `MRC_ERROR_CAN_SEND` |
| -20 ~ -28 | CANopen 协议层 | `MRC_ERROR_CO_SDO_ABORT`, `MRC_ERROR_CO_NODE_NOT_FOUND` |
| -30 ~ -35 | CiA402 运动控制层 | `MRC_ERROR_MOTION_FAULT`, `MRC_ERROR_MOTION_TARGET_TIMEOUT` |
