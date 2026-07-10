# 示例列表

所有示例使用 `./example_xxx [can_if] [node_id]` 格式运行。

## 前置条件

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

## 示例

| 示例 | 源文件 | 说明 |
|---|------|------|
| `example_read_params` | `test_read_params.c` | 读取电机设备信息（SDO 方式） |
| `example_pp_mode` | `test_pp_mode.c` | PP 轮廓位置模式 |
| `example_pv_mode` | `test_pv_mode.c` | PV 轮廓速度模式 |
| `example_pt_mode` | `test_pt_mode.c` | PT 轮廓转矩模式 |
| `example_csp_mode` | `test_csp_mode.c` | CSP 周期同步位置模式 |
| `example_csp_ex` | `test_csp_ex.c` | CSP 模式 (弧度单位) |
| `example_csv_mode` | `test_csv_mode.c` | CSV 周期同步速度模式 |
| `example_cst_mode` | `test_cst_mode.c` | CST 周期同步转矩模式 |
| `example_loop_pp_mode` | `test_loop_pp_mode.c` | 循环 PP 正反转 |
| `example_multi_motor` | `test_multi_motor.c` | 双电机同步控制 |
| `example_raw_can` | `test_raw_can.c` | 原始 CAN 数据收发 |
| `example_pdo_config` | `test_pdo_config.c` | PDO 配置 (RPDO + TPDO 映射) |
| `example_get_position` | `test_get_position.c` | 读取位置 (脉冲/度/弧度) |
| `example_set_zero` | `test_set_zero.c` | 设置零点 |
| `example_sync_read` | `test_sync_read.c` | SYNC 触发 TPDO 读取 (多节点) |

## 工具

| 工具 | 源文件 | 说明 |
|------|--------|------|
| `motor_diag` | `tools/motor_diag.c` | 电机诊断 — 读取并显示详细状态 |
| `motor_scan` | `tools/motor_scan.c` | CANopen 总线扫描 — 探测在线节点 |
| `motor_enc_calib` | `tools/motor_enc_calib.c` | 编码器校准参数读写 (0x2007/0x2008) |

## 运行

```bash
cd build/bin

# 读取设备参数
./example_read_params can0 1

# CSP 模式 (timer)
./example_csp_mode can0 1

# CSP 模式 (manual)
./example_csp_mode can0 1 manual

# 读取位置 (度/弧度)
./example_get_position can0 1

# 设置零点
./example_set_zero can0 1

# SYNC 触发 TPDO 多节点读取
./example_sync_read can0 1,2,3,4,5,7

# 编码器校准数据获取和设置
./motor_enc_calib can0 1 save calib.txt
./motor_enc_calib can0 1 load calib.txt
```
