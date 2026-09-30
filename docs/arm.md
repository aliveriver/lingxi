# 机械臂

2026-09-29：固定 HAL 命令基线的 `+0.01/+0.02 rad` 跟随和回位仍未通过。
最新五次平滑 `+0.01 rad`、去程/回程各2秒的实机试验仍失败：实际增量0.0013423rad，URS回位残余0.0011506rad。本次补偿关闭，不能据此判断重力补偿效果。
重力补偿参考仓库已完成首轮审查，新增只生成报告的 `x2 gravity-report`，
模型条件、接口差异和离线使用见 [审查记录](upstream-ik-audit.md)。通用补偿执行器仍限mock，实时补偿仅接入受限单轴诊断。
后续已完成[首次单轴有界部分补偿实测](gravity-live-first-trial.md)，通用补偿API仍限mock；模型标定和精确运动验收仍未完成。
现已实现 [平滑轨迹、有界位置补偿与 mock 执行](compensated-joint-sessions.md)，可离线规划和核验回放；不改变现有实机控制路径。
后续 [官方模型与 PC2 对照](official-model-audit.md) 已发现 8 项位置限位字段差异，
并提供 `compare-arm-models`、只读 IMU/腰角采集与离线校验工具。

## 数据与顺序

双臂命令固定 14 项：左臂 7 项后接右臂 7 项，每侧顺序为 shoulder pitch/roll/yaw、elbow、wrist yaw/pitch/roll。位置单位 rad、速度 rad/s、力矩 N*m、刚度 N*m/rad、阻尼 N*m*s/rad。

```python
from lingxi_x2 import ArmCommand, X2Client

with X2Client("config/x2.yaml") as x2:
    current = x2.arm_state()
    target = tuple(j.position_rad for j in current.joints)
    x2.move_arm(ArmCommand(target, duration_s=2.0), confirm_hardware=True)
```

## 控制路径

当前优先路径（v1.1.4 已发现）：

```text
/mc/upper_body_command -> UpperBodyCommandArray -> PC1 MC -> EtherCAT HAL -> arms
```

它需要 `UPPERBODY_REMOTE_SPLIT`（官方缩写 `URS`），推荐 50 Hz，并保留 MC 管理。命令使用 `BEST_EFFORT + VOLATILE`；时间戳与 MC 接收时刻相差超过 200 ms 会被丢弃。v1.1.4 实机已完成模式进入、命令发布及恢复站立，但准确保持与运动跟随仍未通过。

底层回退路径（v0.9.7 的唯一可用路径）：

```text
/aima/hal/joint/arm/command -> JointCommandArray -> EtherCAT HAL -> arms
```

底层 Topic 没有超时保护。PC1 MC 也在发布同一 Topic；未停止 MC 时项目会拒绝控制。不能把 `allowed_competing_nodes` 配成 `mc_ros2_node*` 来绕过。

## 现场运动验收

2026-09-29 更新：旧 `test-arm-joint` / `test-arm-joint-session` 实机入口已停用（mock 保留），因为按测量反馈反复重设全臂目标会引入基线漂移。下方旧流程命令仅供历史对照；当前单关节诊断使用“固定命令基线”工具，见本页后文。

当前尚未完成。只读预检不会创建命令发布者或调用服务：

```bash
uv run x2 --config config/x2.yaml doctor --sample
uv run x2 --config config/x2.yaml preflight arm
ros2 topic info /mc/upper_body_command --verbose
```

`preflight` 在 `enabled: false` 下应返回 `ready: false`，并列出当前模式、动作、安全状态、反馈轴数和发布者冲突。`input_source` 仅代表连续速度/姿态控制权，官方明确说明 `SetMcAction` 不更新该字段；发布者排他性以 ROS 图为准。

`upper_body_mc` 的发布者排他检查不能通过 `allowed_competing_nodes` 绕过；该白名单仅保留给已单独验收的 HAL 回退路径。

首次现场验收必须逐步执行且每一步单独确认：操作员先通过官方 APP/控制器将机器人切到稳定站姿，操作员持急停，关闭旧遥控桥，确认 MC 运行且命令 Topic 无其他发布者；只读确认 `STAND_DEFAULT/RUNNING`、FSM 线值 `4`、body `STAND(1)`；记录原模式；再由项目请求 `UPPERBODY_REMOTE_SPLIT`；再次运行 preflight；发送当前姿态保持；只移动一个关节 `0.01-0.02 rad`；返回基线；停止发布并观察；最后恢复 `STAND_DEFAULT`。不得从 `PASSIVE_DEFAULT/SIT` 直接请求 URS，也不得把这些步骤合成无人值守脚本。v1.1.4 安装的 `.msg` 把 FSM 值 `4` 标为 `SAFE`，但官网 MC 状态页将线值 `4` 标为 `STABLE`，且实机 `STAND_DEFAULT` 稳定站立时实际发布 `4`；项目按线值和现场状态组合判断，不单独依赖枚举名称。

完成并记录上述现场验收前，保留：

```yaml
control:
  enabled: false
  authority: upper_body_mc
  publish_rate_hz: 50.0
```

以下命令是旧流程的历史对照，当前不执行。现行固定基线脚本须现场新确认，仅在进程内临时启用控制；磁盘配置始终保持 `enabled: false`：

```bash
uv run x2 --config config/x2.yaml mode UPPERBODY_REMOTE_SPLIT --confirm-hardware
uv run x2 --config config/x2.yaml preflight arm
uv run x2 --config config/x2.yaml hold --duration 1.0 --confirm-hardware > logs/hold.json
uv run x2 --config config/x2.yaml test-arm-joint 0 0.01 --confirm-hardware > logs/joint-0.json
uv run x2 --config config/x2.yaml mode STAND_DEFAULT --confirm-hardware
```

`test-arm-joint` 只接受 14 轴索引 `0..13` 和绝对幅度 `0.01..0.02 rad`，正常路径会命令回到测得的基线，再停止发布并记录反馈与 MC 状态。模式切换和三次 `+0.01 rad` 已实机执行，但编码器均未证明跟随。若任一步报警或异常，应停止后续命令并由现场人员按 MC/急停流程处置，不要自动尝试带故障恢复。完成验收后立即恢复 `enabled: false`。

进入单关节测试前必须先审查保持阶段的逐关节跟踪误差。会话代码对目标轴强制要求误差不超过计划幅度的一半；其他轴漂移仍需现场审查，不能把该资格条件等同于全上肢保持通过，也不能把自然漂移误判为命令响应。

## 当前诊断与下一次复测

三次左肩 pitch `+0.01 rad` 的实际变化依次为 `-0.001726`、`-0.000959`、`-0.000575 rad`，目标误差约 `-0.011 rad`。补齐 header、增加 1 秒驻留均未改善；关节响声不是运动验证。完整数据见 [验证记录](validation.md)。

2026-09-29 修正循环后的第四次复测仍失败：实际变化 `-0.001151 rad`、目标误差 `-0.011151 rad`。250 帧 sequence 连续，本地 DDS 最后命令回读匹配，段内实际 `46.6–47.5 Hz`；全程含跨段最大间隔 `34.5 ms`。会话已恢复站立，全部 fault 0。后续优先对齐 MC 下游 HAL command 与编码器数据，区分命令未被消费和消费后未跟随；不通过放大幅度代替诊断。

发布路径现在在轨迹前做一次完整预检，插值与驻留连续发布；逐帧执行缓存安全检查，后台检查 DDS 发布者冲突，不再逐帧查询 graph 或等待 hand state。另一肢体的保持目标在轨迹开始时固定。超时后不突发补帧；因此调度慢时轨迹可能延长，真实耗时以日志为准。

`move_arm()` / `move_hand()` 返回发布统计，验收会话 JSON 中的 `publication_stats` 包含每帧单调时刻、实际 Hz、最大间隔、发布数量、sequence 连续性。设置 `control.command_echo: true` 可以记录最后一条命令的本地 DDS 回读；它不表示 MC 已接收执行。发生异常时 CLI 会输出最近一段已成功发布帧的统计，API 可调用 `publication_stats()` 取得它。

现场授权的后续单关节诊断改用已观测的稳定 HAL 命令基线，其他轴目标保持不变：

```bash
source /opt/ros/humble/setup.bash
uv run python scripts/command_baseline_session.py --confirm-hardware \
  --trace logs/UNIQUE-fixed-baseline.json
```

新工具只在进程内临时启用控制和回读，不修改磁盘开关。每次选择未使用的日志路径；完整会话连续发布 250 帧。先核对时序、HAL 是否收到目标，再检查编码器跟随与恢复。该会话进入 URS 后在 `finally` 请求恢复 `STAND_DEFAULT`，但服务失败或进程被强制杀死时无法保证恢复，必须保留现场接管。

工具支持 `--delta-rad`（默认 `0.01`），绝对幅度仍硬限制为 `0.01–0.02 rad`，无效幅度在初始化 ROS 前拒绝。2026-09-29 操作员确认后执行一次 `+0.02 rad`：HAL 正确到达 `0.420 rad`，编码器相对基线仅增加 `0.003259659 rad`；恢复驻留后残留 `0.002109051 rad`。跟随与回位仍未通过，已停止加幅，没有执行 `0.2 rad`。结束后独立只读检查确认稳定站立及臂/手 fault 全零，这不等于编码器准确回位。

同步记录已证明 MC 将命令正确传到 HAL。固定基线 `0.400 → 0.410 → 0.400 rad` 的实测编码器增量仍仅 `+0.000575 rad`，所以基线修正尚未解决位置跟随。当前应继续检查驱动/MC 参数和机械条件；不要把本地回读、HAL 目标匹配或 effort 变化当作成功运动验收。

离线分析已有日志（不会初始化 ROS）：

```bash
uv run x2 analyze-log logs/20260928-arm-joint-0-001-dwell.stdout.log
```

使用 HAL 回退路径必须单独验收，并配置为 `hal_mc_stopped`。项目仍要求每次写调用显式 `confirm_hardware=True`。

## 限位

PC1 配置核查确认当前 URS 实际双臂增益为 `kp=40, kd=2`，由 MC 配置决定。`UpperBodyCommandArray` 只承载位置目标；`ArmCommand` 的默认增益 `20/2` 在该路径只是历史占位，不会改变 MC 增益。API 现在拒绝自定义刚度/阻尼或非零速度/力矩，避免静默忽略。HAL 回退路径不受这项字段限制影响，但仍必须单独验收。

完整证据、腕部索引排查及未应用的调参候选见 [PC1 控制核查](pc1-control-audit.md)。当前小幅位置跟随仍未通过，不能把驱动力矩随目标变化等同于位置控制成功。

项目按 AimDK 1.1.0 公布的 X2 Ultra 保证活动范围与已审查的官方 v1.3.0、PC2 SDK 模型限位取交集检查目标。2026-09-29 修复原先误将左臂限位重复用于右臂的问题：右肩 roll 为 [-2.993, 0.061] rad、右腕 roll 为 [-0.724, 1.5097] rad，左侧对应 [-0.061, 2.993] 与 [-1.5097, 0.724]。该表只对应当前审查的 X2 Ultra v1.3 模型，不能作为 -N 型号适配证明；它不是碰撞检测，也不替代自碰撞/环境规划。本平台当前不提供独立力矩控制入口，上肢位置控制也仍未完成实机运动验收。

固定命令基线的离线质量检查、运动判定及重复性标准见 [验收分析](acceptance-review.md)。

后续离线实现了 [重力偏置匹配对照及验收分析](gravity-comparison.md)，专用脚本显式选择on/off并在切模式前检查最终发送目标URDF限位。本机330项测试通过，未部署PC2、未新增实机结果。
