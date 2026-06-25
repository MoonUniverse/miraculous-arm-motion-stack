# Git 提交记录 - SDK API 升级 + 轨迹跟踪测试框架

**提交哈希**: `cc8d4b7`  
**日期**: 2026-06-25 13:29:23  
**作者**: yuyingjin <jinyuyingsspu@gmail.com>

---

## 提交信息

```
feat: SDK API upgrade + trajectory tracking test framework

## SDK API Upgrade (2026-06-25)

- Added _ex functions for radian/degree CSP control
- MiraculousArm adapted to use get_position_ex/csp_set_target_ex
- Eliminated manual pulse conversion, SDK uses precise 2^bw calculation

## Trajectory Tracking Test Framework

- New test node for single-joint sinusoidal tracking
- Python plotting script with RMSE/MAE/correlation analysis
- Parameterized launch file and config template

## Documentation

- Updated IMPLEMENTATION_DETAILS.md with SDK upgrade notes
- Added SDK_API_UPGRADE_NOTES.md migration guide
- Added TRAJECTORY_TEST_DOCUMENTATION.md complete usage guide

Build verified on x86_64. Requires real CAN bus testing.
```

---

## 文件变更统计

| 类型 | 文件数 | 新增行 | 删除行 |
|------|--------|--------|--------|
| 修改 | 5 | 222 | 28 |
| 新增 | 7 | 1378 | 0 |
| **总计** | **12** | **1600** | **28** |

### 修改的文件

1. `miraculous_sdk_x86_64/example/CMakeLists.txt` (+7/-1)
   - 添加 test_csp_ex.c 编译配置

2. `miraculous_sdk_x86_64/include/miraculous_sdk.h` (+121/-0)
   - 新增 `_ex` API 函数声明
   - 新增 `PosUnit_t` 枚举
   - 新增编码器位宽配置函数

3. `ros2_ws/src/arm_motion_stack/miraculous_driver/CMakeLists.txt` (+10/-0)
   - 添加轨迹跟踪测试节点可执行文件
   - 安装 Python 绘图脚本

4. `ros2_ws/src/arm_motion_stack/miraculous_driver/docs/IMPLEMENTATION_DETAILS.md` (+41/-3)
   - 添加 SDK API 升级章节
   - 更新数据流图和后台线程说明

5. `ros2_ws/src/arm_motion_stack/miraculous_driver/src/miraculous_arm.cpp` (+41/-24)
   - 后台读取线程改用 `get_position_ex()`
   - CSP 下发改用 `csp_set_target_ex()`
   - 保留兼容性转换函数（标记为已弃用）

### 新增的文件

1. `miraculous_sdk_x86_64/example/test_csp_ex.c` (99 行)
   - SDK 示例代码，演示 `_ex` API 使用

2. `ros2_ws/src/arm_motion_stack/miraculous_driver/config/trajectory_test_params.yaml` (28 行)
   - 轨迹测试参数配置模板

3. `ros2_ws/src/arm_motion_stack/miraculous_driver/docs/SDK_API_UPGRADE_NOTES.md` (102 行)
   - SDK API 升级详细迁移指南
   - 包含前后对比代码示例

4. `ros2_ws/src/arm_motion_stack/miraculous_driver/docs/TRAJECTORY_TEST_DOCUMENTATION.md` (374 行)
   - 完整的轨迹测试框架设计文档
   - 包含架构设计、API 参考、使用示例

5. `ros2_ws/src/arm_motion_stack/miraculous_driver/launch/trajectory_test.launch.py` (88 行)
   - 参数化的启动文件

6. `ros2_ws/src/arm_motion_stack/miraculous_driver/scripts/plot_trajectory.py` (193 行)
   - Python 绘图和分析脚本
   - 三面板可视化 + FFT 频域分析

7. `ros2_ws/src/arm_motion_stack/miraculous_driver/src/trajectory_tracking_test_node.cpp` (524 行)
   - 单关节正弦轨迹跟踪测试节点
   - 同步记录命令/实际位置
   - 自动计算 RMSE/MAE/相关系数/相位滞后

---

## 核心改进

### 1. SDK API 升级

**之前（手动转换）:**
```cpp
// 读取
int32_t p = 0;
miraculous_motor_get_position(motor, &p);  // 脉冲
double rad = pulse / pulses_per_radian;     // 手动转换

// 下发
int32_t pulse = rad * pulses_per_radian;    // 手动转换
miraculous_motor_csp_set_target(motor, pulse);  // 脉冲
```

**之后（SDK 自动转换）:**
```cpp
// 读取
float rad = 0.0f;
miraculous_motor_get_position_ex(motor, &rad, POS_UNIT_RADIAN);  // 直接弧度

// 下发
miraculous_motor_csp_set_target_ex(motor, rad, POS_UNIT_RADIAN);  // 直接弧度
```

**优势:**
- ✅ 消除手动转换误差（SDK 使用 `2^bw` 精确计算）
- ✅ 代码更简洁（无需维护 `pulses_per_radian` 转换逻辑）
- ✅ 可配置精度（通过 `set_encoder_bw()` 调整编码器分辨率）

### 2. 轨迹跟踪测试框架

**功能:**
- 🎯 100Hz 正弦/余弦轨迹下发
- 📊 同步记录命令值和实际值（时间戳对齐）
- 📈 自动计算跟踪精度指标（RMSE、MAE、最大误差、相关系数、相位滞后）
- 🖼️ Python 脚本生成三面板图（轨迹对比、误差曲线、频谱分析）

**使用流程:**
```bash
# 1. 启动测试节点
ros2 run miraculous_driver trajectory_tracking_test_node

# 2. 开始测试（Service 调用）
ros2 service call /trajectory_test/start std_srvs/srv/Trigger

# 3. 停止测试并查看结果
ros2 service call /trajectory_test/stop std_srvs/srv/Trigger

# 4. 绘图分析
python3 install/miraculous_driver/lib/miraculous_driver/plot_trajectory.py \
  tracking_test_20260625_132923.csv
```

---

## 验证状态

| 项目 | 状态 | 说明 |
|------|------|------|
| x86_64 编译 | ✅ 通过 | 开发机构建成功 |
| 可执行文件安装 | ✅ 正确 | 所有节点和脚本正确安装 |
| 插件发现 | ✅ 正常 | ros2_control 插件注册成功 |
| API 兼容性 | ✅ 保持 | 外部接口不变，向后兼容 |
| ARM 实机测试 | ⚠️ 待测 | 需要在真实 CAN 总线上验证 |

---

## 下一步计划

1. **ARM 目标机部署**
   - 在真实机械臂上测试 `_ex` API
   - 验证编码器位宽配置是否正确
   - 确认 CSP 同步行为符合预期

2. **轨迹跟踪测试**
   - 运行单关节正弦跟踪测试
   - 收集 CSV 数据并绘图分析
   - 评估跟踪精度（RMSE、MAE 等指标）

3. **性能优化**（可选）
   - 根据测试结果调整控制频率
   - 优化 SYNC 发送策略
   - 考虑多关节同时测试

---

## 相关文档

- [IMPLEMENTATION_DETAILS.md](../ros2_ws/src/arm_motion_stack/miraculous_driver/docs/IMPLEMENTATION_DETAILS.md) - 完整实现文档
- [SDK_API_UPGRADE_NOTES.md](../ros2_ws/src/arm_motion_stack/miraculous_driver/docs/SDK_API_UPGRADE_NOTES.md) - SDK 升级迁移指南
- [TRAJECTORY_TEST_DOCUMENTATION.md](../ros2_ws/src/arm_motion_stack/miraculous_driver/docs/TRAJECTORY_TEST_DOCUMENTATION.md) - 轨迹测试框架使用文档

---

**备注**: 本次提交是 `miraculous_driver` 包的重要升级，引入了更精确的 SDK API 和完整的轨迹跟踪测试能力，为后续的电机控制精度验证奠定了基础。
