# 板上测试步骤 — 2026-07-07 驱动修复验证

本文档对应 2026-07-07 对 `miraculous_driver` 的代码审查与修复，按顺序在真实板子 +
真实机械臂上执行。**每一步通过后再进行下一步，任何一步失败先停下排查。**

## 本次改动清单（每项后面标注了对应的验证步骤）

| # | 改动 | 文件 | 验证步骤 |
|---|------|------|---------|
| 1 | seed 失败不再放行：configure/activate 读不到编码器直接报错拒绝；activate 时用编码器实时位置**重新** seed 命令值 | `miraculous_arm.cpp`, `miraculous_system.cpp` | 步骤 4 |
| 2 | timer SYNC 模式改为全臂共享**一个** SYNC 定时器（原来 6 电机各启一个 → 总线 6 倍 SYNC 风暴） | `miraculous_arm.cpp` | 步骤 7（可选） |
| 3 | 新增 `encoder_bw` / `reduction_ratio` 硬件参数（默认 19 / 100.0，即 SDK 默认值，本机型无需改） | `miraculous_arm.{hpp,cpp}`, `miraculous_system.cpp`, `miraculous_arm_params.yaml` | 步骤 3（位置读数正确即覆盖） |
| 4 | 补 `#include <algorithm>`；删死代码 `expand_single_value` | `miraculous_arm.cpp`, `miraculous_system.cpp` | 步骤 1（编译过即覆盖） |
| 5 | `state_poll_rate_hz=0` 时不再用 NOT_READY 覆盖状态缓存；shutdown 前注销 EMCY 回调并停 SYNC | `miraculous_arm.cpp` | 步骤 6 |
| 6 | **2026-07-08（git 版 SDK 适配，根因修复）**：EMCY 改用 SDK 专用 `set_emcy_callback`（raw recv 回调会顶掉 SDK 收包分发，正是上次 `get_position_ex failed` 的根因）；读线程在 CSP 未激活时主动发 SYNC（TPDO 为 SYNC 触发，否则使能前读不到位置）；CMake 指向源码仓 `miraculous_sdk/` | `miraculous_arm.{hpp,cpp}`, `CMakeLists.txt` | 步骤 2（无动力读数）、步骤 6（EMCY） |

背景知识：manual/timer SYNC 模式的区别见对话记录或 `docs/SDK_API_UPGRADE_NOTES.md`。
本驱动默认 **manual**（`sync_period_us: 0`），SYNC 节拍 = controller_manager 的
`update_rate`（当前 100 Hz = 10 ms，与电机工程师用 test_csp_ex 验证的周期一致）。

---

## 步骤 1：编译

```bash
# 前提：SDK 由电机同事负责构建/交付。driver 只需要拿到:
#   include/miraculous_sdk.h + libmiraculous_sdk（build/lib/ 或 lib/ 布局均可）
# 若板上只有源码仓，让电机同事构建一次:
#   cmake -S <sdk仓> -B <sdk仓>/build && cmake --build <sdk仓>/build

cd <ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select miraculous_driver
# SDK 不在默认位置 (<project_root>/miraculous_sdk) 时:
#   --cmake-args -DMIRACULOUS_SDK_DIR=/abs/path/to/sdk
source install/setup.bash
```
- **通过标准**：零错误。`miraculous_arm.cpp` 在 x86 上已过 `-Wall -Wextra` 零警告，
  板上如出现新警告请记录反馈。

## 步骤 2：总线与无动力读数（不使能，臂可手推）

```bash
sudo ip link set can0 up type can bitrate 1000000
candump can0 &   # 留一个终端一直开着，后面步骤都要看

ros2 launch miraculous_driver real_control.launch.py \
  node_ids:=1 joint_indices:=0        # 先只挂 J1 一个关节
```

launch 起来后 hardware 会走到 configured 状态（未激活 = 未使能），此时：

```bash
ros2 topic echo /joint_states --once
```

- **通过标准**：
  - 日志出现 `Seeded initial positions from encoders.`，没有 FATAL；
  - 手推 J1，`/joint_states` 的 position 跟着变，方向、量级（弧度）合理；
  - 读数与电机工程师侧 `./example_csp_ex` 打印的 rad 值一致（同一关节静止时数值吻合）。
- **失败排查**：如果日志报
  `No encoder feedback within 0.25 s; refusing to configure`，这是修复 1 的保护在起作用
  —— 说明 CAN 上读不到该节点，检查 `candump` 是否有 `7xx` 心跳帧、node_id 是否正确。

## 步骤 3：位置换算抽查（覆盖改动 3）

关节静止时记录 `/joint_states` 位置，手动把关节转约 90°（目测即可）：

- **通过标准**：位置变化 ≈ ±1.57 rad。若差了一个明显整数倍（如 100 倍），说明
  减速比/位宽配置和固件不匹配，先停下核对 `encoder_bw` / `reduction_ratio`。

## 步骤 4：seed 安全验证（覆盖改动 1，本次最重要的一步）

**目的**：证明激活瞬间不会跳变。这是原代码最危险的隐患。

1. 保持步骤 2 的单关节 launch 运行（configured、未激活）；
2. **用手把 J1 扳离当前位置至少 0.3 rad**；
3. 激活控制器：

```bash
ros2 control set_hardware_component_state <hardware_name> active   # 或 spawner 自动激活
```

- **通过标准**：激活瞬间关节**保持手扳后的位置不动**（电机使能、有保持力矩，但
  不回跳到扳动前的位置，更不能冲向 0 位）。
- 反向用例：拔掉 CAN 或断电机供电后尝试激活，**必须**看到
  `Cannot read joint positions after enable; refusing to activate.` 且激活失败。

## 步骤 5：单关节 CSP 小步跟踪

激活状态下，通过 JTC 发一条小幅轨迹（对齐 test_csp_ex 的 0.1 rad 量级）：

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [J1], points: [{positions: [<当前位置+0.3>], time_from_start: {sec: 3}}]}"
```

同时观察 `candump`：

- **通过标准**：
  - `080` SYNC 帧间隔稳定 ≈ 10 ms（= update_rate 100 Hz），且**只有一路**；
  - 每帧 SYNC 前有 RPDO 目标帧（`2xx` 段 ID），顺序为"先目标后 SYNC"；
  - 关节平滑走到目标，`/joint_states` 跟踪无振荡；
  - 到位精度参考 test_csp_ex 的容差 ±0.02 rad。
- 也可以用现成的跟踪测试节点（注意它默认 `sync_period_us=10000` 即 timer 模式，
  想保持 manual 需传 0）：

```bash
ros2 launch miraculous_driver trajectory_test.launch.py sync_period_us:=0
```

## 步骤 6：EMCY 与故障路径（覆盖改动 5）

按电机工程师在 `test_emcy_callback` 里用过的方式触发一次可控故障（如过载/限流）：

- **通过标准**：
  - ROS 日志出现 `EMCY from node X: code=0x.... reg=0x..`；
  - 之后每个 read 周期出现 `Motor fault detected` 报错；
  - `fault_reset` 后可重新激活。
- 再验证干净关闭：Ctrl+C 结束 launch，进程退出无 segfault / 无卡死
  （改动 5 的回调注销 + sync_stop 就是保这个）。

## 步骤 7（可选）：timer 模式回归（覆盖改动 2）

只有打算用 timer 模式才需要做：

```bash
ros2 launch miraculous_driver real_control.launch.py node_ids:=1 joint_indices:=0 \
  sync_period_us:=10000
```

- **通过标准**：`candump` 里 `080` 帧仍然是**一路** 10 ms（修复前挂 6 关节会出现
  ~6 倍密度的 SYNC）；挂满 6 关节后复测一次。

## 步骤 8：逐步扩到六轴

1. `node_ids:=1,2 joint_indices:=0,1` 重复步骤 2、4、5；
2. 确认两轴目标在同一帧 SYNC 前全部发出（candump 里两条 RPDO + 一条 SYNC 为一组）；
3. 挂满 6 轴，重复步骤 4、5；
4. 配置真实软限位（`position_min`/`position_max`，弧度）后再接 MoveIt 走
   `moveit_real.launch.py`。

## 异常与回退

- 任何异常动作：Ctrl+C（on_deactivate 会 disable 所有电机）或直接拍硬件急停；
- 激活/配置阶段的新增 FATAL 都是本次修复**故意加的拒绝逻辑**，不要绕过它们，
  按提示排查 CAN/节点；
- 测试全程建议先不接负载或用低限流参数。

## 已知的观察项（非阻塞，测试时顺带记录）

- csp_init 之前反馈走 SDO（100 Hz × 6 关节 ≈ 1200 SDO/s），1 Mbps 总线理论够用；
  六轴时留意 controller_manager 是否报 write/read 超时；
- 各关节零点/方向假设已在电机侧用 `set_zero_position` 对好（URDF 零位 = 电机零位），
  如有方向相反的关节，目前驱动层没有 per-joint 符号翻转，需要反馈后再加。
