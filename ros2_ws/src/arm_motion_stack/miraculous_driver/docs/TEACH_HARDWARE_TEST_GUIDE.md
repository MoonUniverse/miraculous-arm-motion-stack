# Miraculous Arm 真机示教录制与回放测试指南

本文档用于 `teach_record_node` 和 `playback_node` 的真机分阶段验证。文档基于
当前 Driver/SDK 源码的 fail-closed 行为编写，测试对象是：

1. 电机失能、机械臂可拖动时录制同步关节位置。
2. 检查 V2 CSV 的完整性和关节映射。
3. 回放时从当前位置低速插补到 CSV 首点。
4. 安全使能 CSP 后按 CSV 时间戳回放。
5. 从单轴逐步扩大到双轴、稀疏多轴和六轴。

本文档中的主动运动测试必须在单轴轨迹测试已经通过后执行。

## 1. 当前代码的真实工作流程

### 1.1 示教录制

启动 `teach.launch.py` 后，`teach_record_node` 会：

1. 打开并 bootstrap 所有配置的 CANopen 节点。
2. 对所有电机发送 `miraculous_motor_shutdown()`，控制字为 `0x0006`。
3. 确认所有电机处于 `Ready to Switch On`。
4. 不配置 CSP，不启动机械臂后台读线程。
5. 发送一帧广播 SYNC，并等待每个配置电机在该 SYNC 后返回新 TPDO2。
6. 收齐一组新反馈后才发布 `/arm_joint_states`。
7. 收到 `/teach_record/start` 后才开始写 CSV。

`Ready to Switch On` 不是 `Operation Enabled`，此时没有主动位置保持力矩，机械臂
可以被手动拖动。

每次录制采样都是一个完整同步采样：

```text
记录各节点 TPDO2 generation
-> 发送一帧广播 SYNC
-> 等待所有节点返回更新后的 TPDO2
-> 读取该组位置和速度
-> 发布 JointState
-> 录制状态下写入一行 CSV
```

任一关节没有返回本轮新反馈，该轮不会写入 CSV。连续失败达到
`max_consecutive_misses` 后，录制自动中止。

最新版 SDK 在每条 CAN 总线上维护独立接收线程。driver 只发送 SYNC，并通过 TPDO2
generation 和条件变量等待该线程完成回调；不再调用 `miraculous_motor_poll()`。

### 1.2 回放

启动 `playback.launch.py` 只会初始化驱动和 CSP 通信，不会立即进入
`Operation Enabled`。收到 `/playback/play` 后才执行：

launch 启动后，后台读线程会以当前固定的 50 Hz 发送只读反馈所需的手动 SYNC。
这时可能看到周期性 `0x080`，但不应看到周期性 CSP RPDO2 target，也不应有主动运动。

1. 重新加载并校验 CSV。
2. 检查 CSV 关节映射、时间戳、数值和软件限位。
3. 读取当前实际位置。
4. 安全执行 `enable_csp()`：
   - 阻止后台读线程插入 SYNC。
   - 所有轴先进入 `Switched On`，尚未进入 `Operation Enabled`。
   - 获取本次新鲜同步位置反馈。
   - 给所有轴预装对应的当前位置 target。
   - 所有 target 写完后发送一帧受控 SYNC。
   - 逐轴进入 `Operation Enabled`。
   - 再写一次相同当前位置 target，并发送一帧受控 SYNC。
   - 验证所有轴均为 `Operation Enabled` 后才报告成功。
5. 从当前位置按 minimum-jerk 曲线插补到 CSV 第一行。
6. 按 CSV 时间戳和 `speed_scale` 回放。
7. 正常完成、停止或异常退出后执行 `disable()`。

如果任一轴使能失败，驱动会 best-effort quick stop 并对所有配置电机执行
disable voltage，不允许把部分使能当作整体成功。

### 1.3 手动 SYNC 的多轴语义

示教和回放都固定使用 `sync_period_us=0`，即手动 SYNC：

- 一组多轴命令是先依次发送所有轴的 RPDO2 target。
- 然后只发送一帧广播 SYNC。
- 所有电机在同一个 SYNC 边沿锁存各自 target。
- 反馈采样同样是一帧广播 SYNC 后等待所有轴的新 TPDO2。
- 任一 RPDO target 写失败时立即停止后续写入，且本周期不发送 SYNC；全臂进入
  quick-stop 与 SDK I/O 隔离状态。
- 主动命令默认限制单周期变化不超过 `0.005 rad`；commanded-actual 连续 3 个新鲜
  反馈周期超过 `0.05 rad` 时触发全臂 quick-stop。这三个值仍需真机标定后冻结。

不是每个电机各发送一帧 SYNC。

## 2. 安全前提

以下条件有任一项不满足，不开始主动回放：

- 硬件急停可用，并且操作人员能够立即触达。
- 测试区域无人员、线缆、工具和其他碰撞物。
- 首轮测试不带负载，或负载已经获得可靠机械支撑。
- 对受重力影响的 J2、J3 等关节有机械支撑。示教启动时会失能，关节可能下坠。
- 电机侧电流、速度和跟随误差保护已经启用。
- `node_ids` 与物理电机节点一一核对。
- `joint_indices` 与 ROS 关节 `J1...J6` 一一核对。
- 每个主动关节都填写了经过确认的保守软件限位。
- 同一 CAN 接口上没有另一个 driver、SDK example 或 CANopen master。
- 首轮回放使用 `loop:=false`。
- 至少两人参与第一次多轴测试，一人观察机械臂，一人操作终端和急停。

以下动作不是急停：

- `/teach_record/stop` 只停止写 CSV。
- `/playback/stop` 是软件停止请求，回放线程随后执行电机 disable。
- `Ctrl+C` 依赖进程仍能正常执行退出清理。

出现非预期运动、异常声音、碰撞趋势或失控时，优先使用硬件急停。

## 3. 测试终端约定

建议准备四个终端：

| 终端 | 用途 |
|---|---|
| A | 启动 teach 或 playback launch，观察主日志 |
| B | 调用 start/stop/play 服务 |
| C | 观察 `/arm_joint_states` |
| D | `candump` 保存 CAN 证据 |

每个新终端先执行：

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

本文档沿用已经通过轨迹测试的接口配置：

```bash
export CAN_IF=can1
export SDK_BAUDRATE=0
```

`baudrate:=0` 表示 SDK 保持 SocketCAN 接口当前配置。若现场改用 `can0`，所有命令
中的接口必须一致替换。

## 4. 构建与静态检查

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select miraculous_driver arm_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

检查实际加载的包路径：

```bash
ros2 pkg prefix miraculous_driver
ros2 pkg executables miraculous_driver
```

通过标准：

- 构建无错误。
- `trajectory_tracking_test_node`、`teach_record_node` 和 `playback_node`
  均出现在可执行文件列表中。
- 包路径指向当前工作区的 `install/miraculous_driver`。

## 5. CAN 与进程独占检查

### 5.1 检查 CAN 接口

```bash
ip -details -statistics link show can1
```

如果接口尚未配置，由现场负责人按实际波特率配置。1 Mbps 示例：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip -details -statistics link show can1
```

必须确认：

- 接口状态为 `UP`。
- bitrate 与电机一致。
- `bus-off`、`error-passive` 和错误计数没有持续增加。

### 5.2 排除其他控制进程

```bash
pgrep -af 'trajectory_tracking_test_node|teach_record_node|playback_node|ros2_control_node|test_csp'
ros2 node list
```

开始前不能存在占用同一 CAN 接口的旧 driver 或 SDK example。`candump` 是只读监听，
可以保留。

### 5.3 保存 CAN 原始日志

在终端 D 启动：

```bash
mkdir -p /tmp/miraculous-teach-test
command -v candump || echo "candump 未安装，请先安装 can-utils"
candump -tz can1 | tee /tmp/miraculous-teach-test/can_trace.log
```

当前开发机没有安装 `candump`；真机如果同样提示命令不存在，需要先安装
`can-utils`，否则跳过 CAN trace 的项目不能作为完整验收。

关键 CAN ID：

| CAN ID | 含义 |
|---|---|
| `0x080`，DLC 0 | 广播 SYNC |
| `0x080 + node_id`，DLC 8 | EMCY |
| `0x280 + node_id` | TPDO2，实际位置和速度 |
| `0x300 + node_id` | RPDO2，CSP target position |
| `0x580 + node_id` | SDO response |
| `0x600 + node_id` | SDO request |

## 6. 第一阶段：J1 被动读取

第一轮只接入 J1。先不录制，只验证失能状态和反馈映射。

### 6.1 启动

终端 A：

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export TEACH_CSV=/tmp/miraculous-teach-test/teach_j1_$(date +%Y%m%d_%H%M%S).csv
echo "TEACH_CSV=$TEACH_CSV"

ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver teach.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1 \
  joint_indices:=0 \
  record_rate:=20.0 \
  feedback_timeout_ms:=15 \
  max_consecutive_misses:=10 \
  output_file:="$TEACH_CSV" \
  auto_record:=false \
  overwrite_existing:=false
```

记录终端中打印的 `TEACH_CSV` 完整路径，回放时需要使用同一个文件。

### 6.2 观察反馈

终端 C：

```bash
ros2 topic echo /arm_joint_states
```

另一次短时频率检查：

```bash
ros2 topic hz /arm_joint_states
```

缓慢手推 J1 小角度，确认：

- 启动日志出现 `Teach mode ready`。
- 日志确认所有配置电机处于 `Ready to Switch On`。
- J1 没有主动保持力矩，可以平稳手推。
- `/arm_joint_states` 只包含 `J1`。
- 手推 J1 时 position 连续变化，方向正确。
- 约转动 90 度时，位置变化量约为 `1.57 rad`，符号由机械安装方向决定。
- 发布频率接近 20 Hz。
- 没有 EMCY、fault 或连续 feedback timeout。
- CAN 中每轮是一帧 `0x080`，随后出现 node 1 的 `0x281` TPDO2。
- 被动读取阶段没有周期性 `0x301` CSP target。

任一项不通过，停止在本阶段，不进入录制或回放。

## 7. 第二阶段：J1 录制

### 7.1 开始录制

将 J1 放在一个远离机械限位的位置。终端 B：

```bash
ros2 service call /teach_record/start std_srvs/srv/Trigger "{}"
```

预期返回：

```text
success: true
message: recording V2 CSV to ...
```

如果返回文件已存在，不要直接打开覆盖。使用新的文件名重新启动，或在明确确认旧文件
不需要后才使用 `overwrite_existing:=true`。

### 7.2 执行示教动作

首轮只做 5 至 10 秒动作：

1. 开始后静止 1 秒。
2. 缓慢向一个方向移动约 0.05 至 0.10 rad。
3. 停留 1 秒。
4. 缓慢回到附近位置。
5. 再静止 1 秒。

不要在首轮录制中靠近机械限位，不要快速往复拖动。

### 7.3 正常停止录制

终端 B：

```bash
ros2 service call /teach_record/stop std_srvs/srv/Trigger "{}"
```

预期返回包含：

- `saved <file>`
- `rows=<录制行数>`
- `misses=<丢失采样次数>`
- `duration=<持续时间>`
- `reason=stop requested`

停止录制后，teach 节点仍然运行，仍处于被动读取状态。再次调用 start 会创建或检查
同一个配置文件路径，因此首轮测试建议停止 launch 后再为下一轮指定新文件。

### 7.4 退出 teach

在终端 A 按 `Ctrl+C`。正常退出时，被动模式会执行 disable voltage 并关闭电机句柄。

退出后检查：

```bash
pgrep -af 'teach_record_node|playback_node'
```

确认 teach 进程已经完全退出后才能启动 playback。

## 8. CSV 离线检查

先设置刚才实际生成的文件路径：

```bash
export TEACH_CSV=/tmp/miraculous-teach-test/teach_j1_YYYYMMDD_HHMMSS.csv
```

### 8.1 快速检查

```bash
ls -lh "$TEACH_CSV"
head -n 5 "$TEACH_CSV"
tail -n 5 "$TEACH_CSV"
wc -l "$TEACH_CSV"
```

J1 文件头必须为：

```text
timestamp,sample_index,J1
```

V2 CSV 语义：

- `timestamp` 是本次录制首个同步反馈后的相对时间，单位秒。
- `sample_index` 是采样尝试序号。
- `sample_index` 有跳号表示某次没有收齐所有轴的新反馈。
- 有跳号不等于文件损坏，但大量跳号说明反馈链路不稳定。
- 回放使用实际 `timestamp`，不会根据 `sample_index` 自动补点。

### 8.2 完整性和运动量检查

下面的检查不修改 CSV：

```bash
python3 - "$TEACH_CSV" <<'PY'
import csv
import math
import sys

path = sys.argv[1]
with open(path, newline="") as f:
    rows = list(csv.reader(f))

if len(rows) < 2:
    raise SystemExit("FAIL: CSV has no data rows")

header = rows[0]
if len(header) < 3 or header[:2] != ["timestamp", "sample_index"]:
    raise SystemExit(f"FAIL: unsupported header: {header}")

joints = header[2:]
times = []
indices = []
positions = {name: [] for name in joints}

for line_no, row in enumerate(rows[1:], start=2):
    if len(row) != len(header):
        raise SystemExit(f"FAIL: line {line_no} has {len(row)} columns")
    t = float(row[0])
    index = int(row[1])
    values = [float(v) for v in row[2:]]
    if not math.isfinite(t) or any(not math.isfinite(v) for v in values):
        raise SystemExit(f"FAIL: non-finite value at line {line_no}")
    if times and t <= times[-1]:
        raise SystemExit(f"FAIL: timestamp is not increasing at line {line_no}")
    if indices and index <= indices[-1]:
        raise SystemExit(f"FAIL: sample_index is not increasing at line {line_no}")
    times.append(t)
    indices.append(index)
    for name, value in zip(joints, values):
        positions[name].append(value)

gaps = sum(max(0, b - a - 1) for a, b in zip(indices, indices[1:]))
duration = times[-1] - times[0]
print(f"PASS: rows={len(times)} duration={duration:.6f}s index_gaps={gaps}")
for name in joints:
    values = positions[name]
    max_step = max((abs(b - a) for a, b in zip(values, values[1:])), default=0.0)
    max_speed = max(
        (abs(b - a) / (tb - ta)
         for a, b, ta, tb in zip(values, values[1:], times, times[1:])
         if tb > ta),
        default=0.0,
    )
    print(
        f"{name}: min={min(values):.6f} max={max(values):.6f} "
        f"range={max(values)-min(values):.6f} "
        f"max_step={max_step:.6f} max_est_speed={max_speed:.6f}rad/s"
    )
PY
```

首轮建议通过标准：

- 至少 50 个有效数据点。
- 时间戳严格递增。
- `sample_index` 严格递增。
- 所有位置为有限值。
- J1 的范围与手推量一致。
- 没有突变到 0、极大值或符号翻转。
- `index_gaps=0` 最理想；如果有少量跳号，先检查 CAN 日志再决定是否回放。
- 任何单帧位置突变都必须先解释，不能直接回放。

## 9. 第三阶段：J1 低速回放

### 9.1 填写软件限位

主动回放前必须使用经过现场确认的限位。以下 `-0.5` 到 `0.5 rad` 仅沿用之前
J1 轨迹测试通过的示例范围，必须确认该范围仍适合当前安装状态：

```bash
export J1_MIN=-0.5
export J1_MAX=0.5
```

确认：

- 当前 J1 实际位置在该范围内。
- CSV 中 J1 的所有点都在该范围内。
- 该范围没有跨越机械干涉区。

`position_min=0.0` 且 `position_max=0.0` 是占位值。主动真机回放会直接拒绝这种配置，
必须为每个活动关节填写有限且满足 `min < max` 的真实安全限位。

### 9.2 启动 playback

确保 teach launch 已退出。终端 A：

```bash
cd /home/alienware/Documents/PersonalProject/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export TEACH_CSV=/tmp/miraculous-teach-test/teach_j1_YYYYMMDD_HHMMSS.csv

ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver playback.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1 \
  joint_indices:=0 \
  position_min:=-0.5 \
  position_max:=0.5 \
  input_file:="$TEACH_CSV" \
  speed_scale:=0.25 \
  loop:=false \
  approach_velocity_rad_s:=0.03 \
  approach_rate_hz:=50.0 \
  approach_min_duration_s:=1.0 \
  start_tolerance_rad:=0.003 \
  feedback_timeout_ms:=15
```

`speed_scale:=0.25` 表示按原轨迹四倍时间慢速回放。代码使用：

```text
回放相对时间 = CSV 相对时间 / speed_scale
```

启动后只应出现 `Playback ready`，机械臂不应主动运动。

### 9.3 使当前姿态与首点保持小距离

首轮回放前，建议先在 teach 被动模式下将 J1 手动放到接近 CSV 第一行的位置，再退出
teach 并启动 playback。不要为了测试首点插补而故意制造大距离。

查看 CSV 首点：

```bash
sed -n '2p' "$TEACH_CSV"
```

### 9.4 开始回放

一人观察机械臂并握住急停，另一人在终端 B 执行：

```bash
ros2 service call /playback/play std_srvs/srv/Trigger "{}"
```

预期返回：

```text
success: true
message: starting safe playback (... points, approach to first point enabled)
```

预期运动分为两个明确阶段：

1. 安全 CSP seed 后保持当前实际位置，使能瞬间无跳变。
2. 按低速 minimum-jerk 曲线接近 CSV 首点。
3. 到达首点后按 `speed_scale=0.25` 回放录制动作。

回放完成后日志出现 `Playback finished.`，随后电机被 disable。

### 9.5 正常软件停止

需要提前停止时：

```bash
ros2 service call /playback/stop std_srvs/srv/Trigger "{}"
```

预期日志出现 `Playback stopped`，随后执行 disable。该服务是正常停止，不代替硬件急停。

### 9.6 J1 回放通过标准

- 调用 `/playback/play` 前没有主动运动。
- 使能瞬间 commanded position 与 actual position 无可见跳变。
- 首点接近方向正确、速度低且连续。
- 接近首点后才开始原轨迹。
- 回放时长约为原 CSV 时长的 4 倍，再加首点接近时间。
- 回放动作与手动示教方向一致。
- 结束后 J1 失去主动保持力矩。
- 没有 EMCY、fault、feedback failure 或软件限位拒绝。
- CAN 中每个控制周期是 `0x301` target 后跟一帧 `0x080` SYNC。

首轮通过后，可依次测试：

```text
speed_scale=0.25
-> speed_scale=0.50
-> speed_scale=1.00
```

每一级都重新检查最大速度、机械间隙和跟踪表现，不直接跳到 `1.00`。

## 10. 使能瞬间的 CAN 安全顺序检查

保存 `/playback/play` 前后 2 至 3 秒 CAN 日志。单轴应看到以下逻辑顺序：

```text
反馈 SYNC
-> 新鲜 TPDO2
-> 写当前位置 seed RPDO2
-> 受控 SYNC
-> Enable Operation
-> 再写相同当前位置 RPDO2
-> 受控 SYNC
-> 正常轨迹 RPDO2 + SYNC
```

多轴时必须是：

```text
J1 seed RPDO2
J2 seed RPDO2
...
JN seed RPDO2
一帧广播 SYNC
```

不能出现：

```text
J1 seed RPDO2
SYNC
J2 seed RPDO2
SYNC
```

当前 playback ROS 话题只发布 actual position，没有独立发布 commanded position。因此，
使能瞬间的精确 commanded-actual 差值应通过 CAN trace、驱动器诊断工具或电机工程师
提供的 0x607A/0x6064 解码工具测量。首轮建议通过门槛：

```text
abs(commanded_position - actual_position) <= 0.01 rad
```

如果无法解码 CAN 原始位置，至少必须满足无可见跳动，并保留 CAN 日志后再进入多轴。

## 11. 第四阶段：J1 + J2 录制与回放

只有 J1 完整通过后才加入 J2。J2 受重力影响时必须先加机械支撑。

### 11.1 双轴录制

终端 A：

```bash
export TEACH_CSV=/tmp/miraculous-teach-test/teach_j1_j2_$(date +%Y%m%d_%H%M%S).csv

ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver teach.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,2 \
  joint_indices:=0,1 \
  record_rate:=20.0 \
  feedback_timeout_ms:=15 \
  max_consecutive_misses:=10 \
  output_file:="$TEACH_CSV" \
  auto_record:=false \
  overwrite_existing:=false
```

确认两个关节都可安全拖动后：

```bash
ros2 service call /teach_record/start std_srvs/srv/Trigger "{}"
ros2 service call /teach_record/stop std_srvs/srv/Trigger "{}"
```

CSV 文件头必须为：

```text
timestamp,sample_index,J1,J2
```

每一行都要求 J1 和 J2 在同一轮 SYNC 后返回新反馈。任一轴反馈缺失，该轮不会写入。

### 11.2 双轴回放

先退出 teach。填写现场确认的双轴限位。以下数值只是命令格式示例：

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver playback.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,2 \
  joint_indices:=0,1 \
  position_min:="-0.5,-0.3" \
  position_max:="0.5,0.3" \
  input_file:="$TEACH_CSV" \
  speed_scale:=0.25 \
  loop:=false \
  approach_velocity_rad_s:=0.02 \
  approach_rate_hz:=50.0 \
  approach_min_duration_s:=1.5 \
  start_tolerance_rad:=0.003 \
  feedback_timeout_ms:=15
```

开始：

```bash
ros2 service call /playback/play std_srvs/srv/Trigger "{}"
```

双轴额外通过标准：

- 两个轴使能瞬间均无跳变。
- 首点接近期间两个轴同时平滑运动。
- 每个周期两条 target 均出现在同一帧 SYNC 之前。
- 没有出现只使能一个轴继续运行的情况。
- 回放结束后两个轴均退出 `Operation Enabled`。

## 12. 稀疏多轴示例：J1 + J3 + J5

稀疏映射支持如下配置：

```text
CAN node 1 -> ROS J1 -> joint index 0
CAN node 3 -> ROS J3 -> joint index 2
CAN node 5 -> ROS J5 -> joint index 4
```

录制：

```bash
export TEACH_CSV=/tmp/miraculous-teach-test/teach_j1_j3_j5_$(date +%Y%m%d_%H%M%S).csv

ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver teach.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,3,5 \
  joint_indices:=0,2,4 \
  record_rate:=20.0 \
  feedback_timeout_ms:=15 \
  max_consecutive_misses:=10 \
  output_file:="$TEACH_CSV" \
  auto_record:=false \
  overwrite_existing:=false
```

文件头应为：

```text
timestamp,sample_index,J1,J3,J5
```

回放时必须保持完全相同的 node 和 joint 映射：

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver playback.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,3,5 \
  joint_indices:=0,2,4 \
  position_min:="J1_MIN,J3_MIN,J5_MIN" \
  position_max:="J1_MAX,J3_MAX,J5_MAX" \
  input_file:="$TEACH_CSV" \
  speed_scale:=0.25 \
  loop:=false \
  approach_velocity_rad_s:=0.02 \
  approach_rate_hz:=50.0 \
  approach_min_duration_s:=1.5 \
  start_tolerance_rad:=0.003 \
  feedback_timeout_ms:=15
```

这里三项限位按活动关节顺序对应 J1、J3、J5。也可以传六项完整 J1 至 J6 限位，
驱动会按 `joint_indices` 选择对应项。

## 13. 第五阶段：六轴

六轴只在以下条件全部满足后开始：

- 每个关节单轴被动读取和方向检查通过。
- 每个关节单轴低速回放通过。
- 至少一组双轴回放通过。
- 一次三轴同步回放通过。
- seed 时 commanded-actual 差值有可审查证据。
- 第二电机使能失败回滚测试通过。
- seed 反馈缺失拒绝使能测试通过。
- 六轴均有真实软件限位。
- 重力轴有制动、支撑或已验证的受控失能方案。

六轴录制：

```bash
export TEACH_CSV=/tmp/miraculous-teach-test/teach_j1_j6_$(date +%Y%m%d_%H%M%S).csv

ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver teach.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,2,3,4,5,6 \
  joint_indices:=0,1,2,3,4,5 \
  record_rate:=50.0 \
  feedback_timeout_ms:=15 \
  max_consecutive_misses:=10 \
  output_file:="$TEACH_CSV" \
  auto_record:=false \
  overwrite_existing:=false
```

六轴回放命令模板：

```bash
ROS_LOG_DIR=/tmp/ros-log ros2 launch miraculous_driver playback.launch.py \
  can_interface:=can1 \
  baudrate:=0 \
  node_ids:=1,2,3,4,5,6 \
  joint_indices:=0,1,2,3,4,5 \
  position_min:="J1_MIN,J2_MIN,J3_MIN,J4_MIN,J5_MIN,J6_MIN" \
  position_max:="J1_MAX,J2_MAX,J3_MAX,J4_MAX,J5_MAX,J6_MAX" \
  input_file:="$TEACH_CSV" \
  speed_scale:=0.25 \
  loop:=false \
  approach_velocity_rad_s:=0.01 \
  approach_rate_hz:=50.0 \
  approach_min_duration_s:=2.0 \
  start_tolerance_rad:=0.003 \
  feedback_timeout_ms:=15
```

首轮六轴示教动作要短、小、慢，避免同时做大幅度关节运动。

## 14. 安全失败路径真机验证

本节只能在无负载、机械支撑完善、低电流限制、急停有效的专用台架上执行。故障制造
方式由电机工程师确认，不建议在通电运动过程中直接拔插 CAN 线。

### 14.1 第二个电机使能失败

目的：确认第一个电机即使已经进入 `Operation Enabled`，第二个电机失败后也会被
回滚。

准备：

1. 只配置 node 1 和 node 2。
2. 两个电机脱离负载或可靠固定。
3. 使用电机工程师认可的方法，让 node 2 在 `Enable Operation` 阶段返回失败，
   但仍允许通信和状态读取。
4. node 1 保持正常。

调用：

```bash
ros2 service call /playback/play std_srvs/srv/Trigger "{}"
```

通过标准：

- `/playback/play` 服务可先接受任务，但后台日志明确报告 `enable_csp failed`。
- 日志保留 node 2 的原始使能失败原因。
- node 1 收到 quick stop 和 disable voltage。
- node 2 以及所有其他配置电机也执行 best-effort disable voltage。
- 最终没有任何配置电机保持 `Operation Enabled`。
- 不继续发送正常 CSP target。
- 不自动 fault reset。

如果某个回滚命令失败，日志必须显示对应 joint 和 node ID，并继续回滚其他电机。

### 14.2 seed 反馈缺失

目的：确认无法获得本次新鲜反馈时，不进入 `Operation Enabled`。

准备：

1. playback 已启动并显示 `Playback ready`。
2. 使用电机工程师认可的方式，仅屏蔽一个节点的同步 TPDO2 反馈。
3. 保持 SDO 和其他节点通信可观察。

调用 `/playback/play` 后通过标准：

- 日志报告 `acquire fresh seed feedback` timeout。
- 所有轴均不进入正常主动回放。
- 所有配置电机执行 disable voltage 回滚。
- 失败后没有周期性 CSP RPDO2 target。回滚成功后，后台读取可能恢复只读反馈
  SYNC，不能仅根据仍有 `0x080` 判断回滚失败。
- 恢复 TPDO2 后，先重启节点，再进行下一次测试。

### 14.3 录制反馈连续丢失

在被动录制台架上，由电机工程师安全屏蔽一个节点 TPDO2：

- 单次失败产生 `Fresh feedback timeout`。
- 缺失反馈的采样不写入 CSV。
- 连续失败达到 `max_consecutive_misses` 后 CSV 标记为 `aborted`。
- 节点不进入 CSP。

### 14.4 EMCY

使用电机工程师已经验证过的可控方式触发 EMCY：

- 录制立即中止。
- 对所有配置电机执行 disable voltage。
- 日志记录 node ID、error code 和 error register。
- 当前进程内禁止再次录制。
- 不自动 fault reset，排查后重启节点。

## 15. 常见问题

### 15.1 启动 teach 时报 startup feedback unavailable

检查：

```bash
ip -details -statistics link show can1
candump -tz can1
```

重点确认节点心跳、`0x080` SYNC 和对应 `0x280 + node_id` TPDO2。teach 启动只给
500 ms 获取第一组完整新反馈。

### 15.2 teach 启动后机械臂仍有保持力

不要继续拖动。检查：

- 是否启动了错误的 launch。
- 是否还有另一个控制进程。
- 驱动状态是否确实为 `Ready to Switch On`。
- 电机侧是否有独立制动器或其他保持功能。

### 15.3 `/teach_record/start` 找不到

```bash
ros2 service list | grep teach_record
ros2 node list
```

当前 launch 中正确服务名是：

```text
/teach_record/start
/teach_record/stop
```

### 15.4 录制开始失败

常见原因：

- 已经在录制。
- 输出文件存在且 `overwrite_existing=false`。
- 任一驱动不再是 `Ready to Switch On`。
- 第一组完整新反馈超时。
- 已锁存 EMCY/fault，需要排查并重启节点。

### 15.5 playback 拒绝 CSV

检查：

- V2 文件头是否为 `timestamp,sample_index,J...`。
- 时间戳和 `sample_index` 是否严格递增。
- 是否存在 NaN、Inf、空字段或列数变化。
- V2 CSV 是否包含 playback 未配置的关节。
- CSV 位置和当前实际位置是否都在软件限位内。

### 15.6 playback 启动后不动

这是正常行为。启动 launch 只初始化，不主动回放。需要显式调用：

```bash
ros2 service call /playback/play std_srvs/srv/Trigger "{}"
```

### 15.7 首点接近时间很长

接近时间按下式计算：

```text
duration = max(
  approach_min_duration_s,
  1.875 * max_joint_delta / approach_velocity_rad_s
)
```

首点距离越大，接近时间越长。这是安全限制，不应通过大幅提高速度绕过。优先在被动
模式下把机械臂放到接近首点的位置。

### 15.8 回放结束后关节失去保持力

这是当前预期行为。正常完成、停止和大多数异常路径都会执行 disable。重力轴必须有
机械支撑或适当制动方案。

### 15.9 CAN 出现大量 feedback timeout

检查：

- CAN 错误计数和终端电阻。
- 电机 TPDO2 是否为同步触发。
- 是否有多个 master 同时发送 SYNC。
- `feedback_timeout_ms` 是否过小。
- 六轴总线负载是否允许当前 `record_rate`。

不要仅通过无限增大 `max_consecutive_misses` 掩盖通信问题。

## 16. 每轮测试记录模板

建议每轮保存以下信息：

```text
日期时间：
Git commit：
操作者：
CAN 接口：
node_ids：
joint_indices：
机械负载/支撑：
电机限流配置：
软件限位：
CSV 路径：
record_rate：
feedback_timeout_ms：
有效行数：
sample_index gaps：
CSV 关节范围：
speed_scale：
approach_velocity_rad_s：
首点最大距离：
使能瞬间 commanded-actual 最大差值：
是否出现 EMCY/fault：
CAN trace 路径：
ROS log 路径：
结果：PASS / FAIL
备注：
```

## 17. 六轴前最终通过条件

只有以下项目全部通过，才允许六轴主动回放：

1. 六个单轴分别完成被动方向和弧度量级检查。
2. 六个单轴分别完成 `speed_scale=0.25` 低速回放。
3. 双轴使能、首点接近、同步回放和停止均通过。
4. 至少一组三轴稀疏或连续映射通过。
5. CSV 离线检查无非有限值、时间倒退或无法解释的位置突变。
6. 每个主动关节有真实、保守的软件限位。
7. 使能瞬间 commanded-actual 差值不大于 `0.01 rad`。
8. 第二电机使能失败时，第一电机确认被回滚。
9. seed 反馈缺失时，确认没有电机进入正常主动回放。
10. EMCY 和正常 stop 都能使全部配置电机退出主动状态。
11. CAN 总线无持续 error-passive、bus-off 或大量反馈超时。
12. 重力轴失能后的机械安全方案已经验证。

任何一项失败，保存 ROS 日志、CAN trace、CSV 和当时参数，停止扩大测试规模。
