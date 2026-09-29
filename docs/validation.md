# 实机验证记录

目标：X2 Ultra，SN `X220028C5Z0034`。

## 2026-09-29：官网模型与 PC2 只读重力观测

后续网络/执行权限恢复，已完成 [官方模型审查](official-model-audit.md) 与新工具部署。

- 官网 SDK 文档确认 v1.3.0 对应铭牌 X2 Ultra，v1.4.0 对应 X2 Ultra -N / EDU。
  固定官方 Git 提交 `575cc6b988f976c23550e0db85aa1e5475d3652d`；现场铭牌尚待确认。
- 只读取得 PC2 SDK/nav URDF 和出厂传感器标定。官方 v1.3.0 与 PC2 SDK 臂几何、质量/COM 相同，
  8 个位置限位字段不同；第三方 IK 模型与 PC2 SDK 的所比较臂字段相同。
- 官网 O10 左右手模型各 0.5343 kg，均为 10 个独立可动轴；尚未确认 wrist→palm 安装变换和
  现有腕部惯性中的末端部件范围，不直接叠加其质量。
- 新增 `compare-arm-models`、`analyze-gravity-state` 和只读脚本 `read_gravity_state.py`。
  最终 3 秒采集 chest/pelvis/waist/arm 回调 822/821/839/835，每路 1 个发布者，解析错误 0。
- 腰 pitch 实测 `+0.346088 rad` 超过选用模型的 `+0.314 rad` 上界，离线校验拒绝 pelvis + 腰链估算。
  不自动裁剪、扩大范围，也不把该差异解释成已经确诊的机械故障。
- 稍后独立只读 preflight 为 `PASSIVE_DEFAULT/RUNNING + FSM wire6 + SIT(4)`；无竞争上肢发布者，
  14+10+10 反馈完整、fault 全零；磁盘控制关闭，`ready=false`。没有请求进入 URS。
- 本地/PC2 全量测试均 **112 passed**；当前环境下 Web 两项已通过，前轮阻塞未复现。
  测试覆盖模型差异、坐标变换、腰链、失效/过期/异步反馈拒绝及离线 CLI 隔离。
- 本轮没有发布运动命令、修改增益/限位/出厂标定、停止或重启 MC。
  两份 PC2 实机 YAML 均保持 `control.enabled: false`。新增原始证据在 `logs/official-model-audit/`。

## 2026-09-29：开源 IK/重力补偿审查与离线工具

本轮仅在本机工作，没有连接 PC1/PC2、发送实机命令、改增益或 MC 配置。
固定 HAL 基线的跟随/回位继续判为未通过。详细结论和数值场景见
[开源仓库审查](upstream-ik-audit.md)。

- 新增 `x2 gravity-report --urdf ... --input ...`，独立解析 URDF 和显式姿态/负载/torso 重力，
  输出模型哈希、静态力矩和诊断量；不初始化 ROS/X2Client、不生成运动命令。
- 新增 21 项测试：解析单摆符号及负载、一般关节变换的势能差分、缺参数/NaN/超限/模型结构拒绝、
  CLI 离线隔离和重复 JSON key 拒绝。
- 本机非 Web 测试 **80 passed**（`pytest --ignore=tests/test_web.py`），
  记录 `logs/20260929-local-tests.txt`。全量 82 项运行在第一个 Web TestClient 入口阻塞并被中止，
  单独 Web 复现由 20 秒超时终止。不能报告本轮全量通过。
- 最小复现不导入 Lingxi/FastAPI，仅 `anyio.from_thread.start_blocking_portal()` 后调用
  `portal.call(lambda: 42)` 也阻塞，由 6 秒超时终止。问题可在当前环境脱离应用复现；
  未为绕过环境问题修改 Web 或测试。诊断见 `logs/20260929-web-test-diagnostic.txt`、
  `logs/20260929-anyio-portal-diagnostic.txt`。
- 上游固定提交自带的 **18 项** IK/内存反馈测试通过；独立重力实现对上游算法的
  **200 个**随机场景最大差 `3.55e-15 N·m`。均不是物理仿真或实机验收。
- 本次新代码未同步 PC2；PC2 最新已知结果仍为前轮 61 项通过。
- 未写入控制配置。本地 `config/x2.motion-test.yaml` 为 `enabled: false`；
  当前工作区未找到 `config/x2.yaml`，未凭推测重建现场配置。

## 2026-09-28：Agi v1.1.4 升级后复验

安全状态：全程只读；未停止 MC，未切换运控模式，未发送机械臂或灵巧手命令。

环境：

- PC2 Agi：`release-lx2501_3_t2d5-soc1-v1.1.4`。
- PC2 OS：`lx2501_3_t2d5-soc1-v0.6.10-hotfix_v9`，Ubuntu 22.04，Jetson Orin NX，Python 3.10，ROS 2 Humble。
- 升级清除了 `/home/run/lingxi`、虚拟环境和用户级 `uv`；项目已重新部署，旧 `uv 0.12.14` 可从 `/home/agi/.old_agi_home.bak/.local/bin/uv` 找到。

ROS 接口：

- `/mc/upper_body_command` / `UpperBodyCommandArray` 已出现；MC 有一个订阅者，复验时发布者为零。
- `/aima/mc/common/state` / `McCommonState` 已出现；采样动作为 `PASSIVE_DEFAULT`。
- `HandStateArray` 已包含左右 `HandTouchSensorData`；左右手均为 `NIMBLE_HANDS=1`，各返回 10 个关节。
- `JointState` 使用 `uint16 error_code`；旧 `coil_temp/motor_temp/motor_vol` 字段已移除。
- ROS Action 列表为空。`SetMcAction`、`GetMcAction`、`GetHandType` 均确认是 ROS Service。
- HAL arm/hand command Topic 各有一个 MC 发布者；没有发现新的用户态命令发布者。

短时频率与格式：

| 数据 | 实测结果 |
| --- | --- |
| arm state | 目标约 500 Hz；7 秒窗口最终均值约 461 Hz，含一次 0.333 s 间隔 |
| hand state | 约 200 Hz |
| RGB-D RGB compressed | 约 30 Hz，JPEG |
| RGB-D depth | 约 30 Hz，1280x720，`16UC1` |
| hand joints | 左右各 10 项；只读静止样本均为 0 |
| tactile | 两侧全部字段存在；只读静止样本均为 0 |

边界：触觉字段与实时帧已经验证，但没有执行官方所述的手部启动动作，也没有施加可控载荷，所以不能声称压力响应已验证。上肢接口只验证到消息类型、Topic 和 MC 订阅者，尚未验证运动、模式切换、断流或退出行为。

## 2026-09-28：v1.1 官方接口复核与 v1.1.4 代码适配

证据等级：官网 `latest` 页面当前标注为 `AimDK_X2 1.1.0 文档`；本节将其公开接口定义与 Agi v1.1.4 实机发现的消息/Topic schema 对照后用于适配。它不是 v1.1.4 实机运动验证。本轮未调用 `SetMcAction`，未创建实机命令发布者，未发送运动命令。

官方文档确认：

- 上肢模式准确名称为 `UPPERBODY_REMOTE_SPLIT`，官方工具缩写为 `URS`。
- `/mc/upper_body_command` 使用 `UpperBodyCommandArray`、`BEST_EFFORT + VOLATILE`，推荐 50 Hz。
- `header.stamp` 与 MC 接收时间相差超过 200 ms 时命令会被丢弃；项目每帧使用 ROS 节点当前时钟。
- `hand_sub_mode=2` 是 `HAND_DEXTEROUS_JOINT`。此时 `hand_pos` 必须为 20 项：左手 10 个主动轴后接右手 10 个主动轴；每只手内部顺序与 `HandCommand[]` 相同。
- `SetMcAction` 请求包含 `RequestHeader header`、非空 `source`、`McActionCommand command`；`command.action_desc` 填模式名。
- 成功需同时核对 `response.header.code == 0` 和 `response.status.value == 1`。项目已记录官方 `2..10`、`100..110` 拒绝码，并在服务接受后等待 `McCommonState.action_info` 到达目标模式的 `RUNNING(100)`。
- `McCommonState.input_source` 只反映连续速度/姿态命令控制权；`SetMcAction` 等动作服务不会更新它，不能把它当作上肢命令发布者判据。
- 模式/动作检查使用 `action_info.action_desc/status`；动作占用使用 `motion_status.player_state/motion`；安全检查使用 `fsm_state.current_state` 和 `body_status.value`。

项目已实现但尚未实机执行：

- 订阅和解析 `/aima/mc/common/state`。
- `x2 preflight` 只读检查：控制开关、授权路径、上肢接口、竞争发布者、MC 模式、动作状态、v1.1.4 稳定站立 FSM 线值 `4`、body `STAND(1)`、动作播放器占用和 14+10+10 新鲜反馈。
- 当前姿态保持命令，以及单臂单关节 `0.01-0.02 rad`、返回基线、停止发布后观察的验收流程。
- 离线测试验证 `arm_pos=左臂7+右臂7`、`hand_pos=左手10+右手10`；当前项目测试为 `18 passed`。

官方依据：

- [上肢控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/upper_body_control.html)
- [运动模式切换](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/modeswitch.html)
- [运控状态查询](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/mc_status.html)
- [末端执行器控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/endeffector.html)
- [关节控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/joint_control.html)

## 2026-09-28：v1.1.4 新适配只读实机 preflight

安全状态：使用 `run@10.0.1.41` 登录 PC2；未调用 `SetMcAction`，未创建命令发布者，未发送运动命令，未停止 PC1 MC。

- PC2 已将 `uv 0.12.19 (aarch64-unknown-linux-gnu)` 安装到 `/usr/local/bin`，`uv`/`uvx` 为 root 所有，`run` 用户可全局调用。
- 新代码部署到 `/home/run/lingxi`，现场配置保持 `control.enabled=false`，更新为 `authority=upper_body_mc`、`publish_rate_hz=50.0`。
- 实机生成类型确认 `UpperBodyCommandArray` 字段为 `header/source/hand_sub_mode/head_pos[2]/arm_pos[14]/hand_pos[]`；`SetMcAction.Request` 为 `header/source/command`，响应为 `CommonResponse response`。
- `/mc/upper_body_command` 为 `BEST_EFFORT + VOLATILE`，MC 有 1 个订阅者，测试前后均为 0 个发布者。
- `/aima/mc/common/state` 为 `BEST_EFFORT + TRANSIENT_LOCAL`，由 MC 发布；解析结果为 `PASSIVE_DEFAULT/RUNNING`、FSM 值 `6`、`body SIT(4)`、动作播放器 `IDLE(0)`、左右手类型均为 `OMNI_HAND(1)`，连续控制源为空。
- 后续直接核对 v1.1.4 PC2 上三份一致的 `McFsmState.msg`，其常量是 `STABLE=2`、`SAFE=4`、`TEST=6`；但官网 MC 状态页定义线值 `4=STABLE`、`6=SAFE`，实机也在 `STAND_DEFAULT/STAND` 时发布 `4`、在 `PASSIVE_DEFAULT/SIT` 时发布 `6`。这是已安装 schema 常量与 MC 生产端/官网定义不一致，项目按 v1.1.4 实测线值和动作、姿态组合判断。`McBodyPoseStatus.msg` 确认 `STAND=1`、`SIT=4`。
- 坐姿只读 `preflight arm` 返回 `ready=false`：未通过 `control_enabled`、`mc_mode`、`mc_stable` 和 `body_standing`。接口发现、发布者排他、MC 状态新鲜、动作运行、播放器空闲、14+10+10 反馈、arm `domain_state=0` 和全部关节 fault code 为零均通过。
- `doctor --sample` 第二次完整通过，取得 14 臂轴、左右各 10 手轴、MC 状态和 1280x720 JPEG 帧。第一次相机等待 2 秒超时；随后 Topic 仍有 1 个发布者且短测约 29.8-30.0 Hz，记录为一次瞬时发现/调度超时，不能据此声称长时间稳定性。
- PC2 原生 Python 3.10 环境测试：`18 passed`，仅有既有 Starlette 弃用警告。

结论：采样时的坐姿 `PASSIVE_DEFAULT + FSM wire 6 + body SIT(4)` 不满足上肢动作条件。项目现在只允许从 `STAND_DEFAULT/RUNNING + FSM wire 4 + STAND(1)` 请求 URS，并在 URS 预检中继续严格要求稳定站立。真实模式切换、保持、小幅运动、恢复和断流行为仍全部未验证。

项目级复验：

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest`：`11 passed`，1 个 Starlette 弃用警告。
- `uv run x2 --config config/x2.yaml status`：连接成功，固件 `v1.1.4`；arm/hand/tactile 均观察到实时样本；上肢接口经 DDS 图发现等待后正确识别，警告为空。
- `uv run x2 --config config/x2.yaml doctor --sample`：14 个 arm 关节、左右各 10 个 hand 关节、1280x720 JPEG 相机帧。
- `uv run x2 --config config/x2.yaml observe --tactile`：项目成功解析左右触觉结构；当前所有 raw uint8 值为 0。

## 2026-09-28：Agi v0.9.7 历史记录

### 已验证

- PC2 SSH、系统/硬件/版本环境。
- PC1 SSH 与角色；MC、EtherCAT HAL 进程存在。
- ROS 2 Topic、Service、Action 图；Action 列表为空。
- arm/hand 命令和状态 Topic 的真实类型与端点。
- `/agibot/software/common` 内 v0.9.7 的实际 `.msg/.srv` 定义。
- v0.9.7 无 `UpperBodyCommandArray`，无 `/mc/upper_body_command`。
- v0.9.7 `HandStateArray` 无触觉字段。
- RGB-D、双目、后视、前视相机 Topic 与发布者存在。
- 用户确认的旧遥控桥 PID 13247 已 SIGTERM；8765 已释放；刷新发现后用户发布者消失。
- PC1 `aima em` 显示 MC stop/start 命令与 `mc` app，但未执行。

### 当时尚未验证

- 本项目在 PC2 的关节/相机连续样本、实际频率和长时间稳定性。
- 任意机械臂运动、灵巧手运动。
- 停止 v0.9.7 MC 的实际结果、全身影响和可靠恢复。
- 指令退出/断流后的保持行为。
- 带触觉字段的新固件或 T2.1 硬件。
- Web 在真实相机下的持续帧率。

## 2026-09-28：首次 URS 当前姿态保持

操作员已通过官方控制端将机器人切到站立并明确授权上肢动作测试。测试前只读状态为 `STAND_DEFAULT/RUNNING`、FSM 线值 `4`、body `STAND(1)`、播放器空闲、14+10+10 反馈完整、全部关节 fault code 为零，`/mc/upper_body_command` 发布者为零、MC 订阅者为一。PC1 MC 未停止。

- 第一次会话在任何服务调用和命令发布前因 DDS 首次发现超过 1 秒而退出；模式保持 `STAND_DEFAULT`，命令发布者仍为零。模式切换前等待已放宽到 3 秒。
- 第二次会话成功执行 `STAND_DEFAULT -> UPPERBODY_REMOTE_SPLIT/RUNNING`，按 50 Hz 发布进入 URS 后测得的当前 14 轴姿态 1 秒，停止发布观察 0.5 秒，最后恢复 `STAND_DEFAULT/RUNNING`。
- 保持结束时最大绝对跟踪误差为 `0.043334 rad`（右肘）；左肘为 `0.039308 rad`，左肩 roll 为 `-0.025118 rad`。该误差大于计划的 `0.01 rad` 单关节测试幅度。
- 停止发布后的 0.5 秒窗口中，记录的 14 轴位置没有继续变化；保持结束时全部 fault code 为零。
- 会话后只读复核为 `STAND_DEFAULT/RUNNING`、FSM 线值 `4`、body `STAND(1)`、无关节故障；命令发布者恢复为零。基线配置和临时测试配置均已恢复 `control.enabled=false`。
- PC2 原始日志：`/home/run/lingxi/logs/20260928-upper-hold-2.json` 和同名 `.stderr.log`。DDS 库在 JSON 后向 stdout 写入诊断信息，因此解析时应只读取首个 JSON 对象。

结论：URS 模式切换、恢复和当前姿态命令发布已在实机执行，但“准确保持当前姿态”尚未通过。因保持漂移超过计划动作幅度，单关节 `0.01 rad` 测试未执行。还需要现场操作员确认可见运动/下沉和报警表现，并排查该跟踪偏差后再继续。

操作员随后确认保持阶段无报警、无明显异常，并授权继续。项目执行左肩 pitch（索引 0）`+0.01 rad`、回基线和断流观察：现场无法目视确认如此小的幅度，但听到关节响声；MC 和关节均未报警。编码器数据显示目标轴相对基线仅变化 `-0.001726 rad`，与命令方向相反，目标误差为 `-0.011726 rad`；同时左右肘分别变化约 `+0.038157` 和 `+0.041801 rad`。因此本次不能认定为单关节动作验证成功，关节响声也不能替代位置反馈证据。

核对 PC2 v1.1.4 自带 Python/C++ `upper_body_control` 示例后发现，官方每帧设置 `header.frame_id="mc_upper_body"` 和递增 `header.sequence`，项目此前仅设置 `header.stamp`。项目已补齐这两个 header 字段；在重复相同幅度测试验证前不放大命令。

## 2026-09-29：三次单关节失败复核与发布循环修正

只读重新解析 PC2 三份 stdout 的会话 JSON（允许前后混入 DDS 诊断文本），目标均为左肩 pitch、索引 0、相对基线 `+0.01 rad`：

| 日志（`/home/run/lingxi/logs/`） | 编码器变化 rad | 目标误差 rad | 会话 s | 去目标含驻留 s | 回基线含驻留 s |
| --- | ---: | ---: | ---: | ---: | ---: |
| `20260928-arm-joint-0-001.stdout.log` | -0.001725674 | -0.011725674 | 4.117499 | 1.002486 | 1.003479 |
| `20260928-arm-joint-0-001-header.stdout.log` | -0.000958920 | -0.010958920 | 4.100608 | 1.002699 | 1.004158 |
| `20260928-arm-joint-0-001-dwell.stdout.log` | -0.000575066 | -0.010575066 | 6.249261 | 2.007163 | 2.006668 |

dwell 会话资格保持为 `1.001940 s`，断流观察为 `0.500789 s`；去目标和恢复各包含配置的 1 秒插值及 1 秒驻留。事件间隔还包含预检/反馈读取，旧日志无法分别还原驻留和插值的精确边界，也无法计算逐帧 Hz、最大间隔或补发次数。总耗时正常不能排除抖动。

三次均未证明目标轴按命令移动。其他轴约 `0.04 rad` 的自然变化、关节声音、进入 URS 和无 fault 均不能替代编码器跟随证据；幅度不得放大到超过 `0.02 rad`，下一次仍使用 `+0.01 rad`。

代码审查确认 `publish_arm()` / `publish_hand()` 原来逐帧调用完整 preflight（含 ROS graph），客户端又按累计 deadline 在延迟后补发。这是需要修正的调度问题，尚无证据证明它是运动不跟随的根因。

本轮实现：

- 每段轨迹开始前完整 preflight 一次；插值和目标驻留共用一次授权，发布结束/异常时撤销授权。未进入授权上下文的 backend 写调用被拒绝。
- 循环只读取缓存 MC/arm/hand 状态，检查新鲜度、模式、站姿、故障、轴数、手型与有限位置。非目标肢体使用轨迹开始时固定的保持目标，不逐帧追随反馈漂移。
- ROS executor 每 0.1 秒检查竞争发布者及外部匹配订阅者；缓存超过 0.5 秒或检查失败即停止发布。DDS graph 发现本身有延迟，此机制不提供硬实时控制权保证。
- 单调时钟调度不补发积压帧；逐帧记录实际 `rclpy.publish` 调用起止、sequence、插值/驻留阶段，输出数量、实际 Hz、最小/最大间隔和 sequence 连续性。mock 的计时标记为 backend API 调用，sequence 未知。
- 可配置 `control.command_echo: true`，保存最后命令字段及匹配的本地 DDS 回读。自身回读订阅者不计作外部接收者。本地 publish/回读均不能证明 MC 消费或执行。
- `x2 analyze-log PATH` 完全离线分析阶段耗时；时长参数的负值、NaN/Inf 在切换模式前被拒绝。

再次读取 PC2 自带的 Python/C++ 官方示例：均为 20 ms 定时器、相同 stamp/frame_id/sequence、hand_sub_mode、head/arm/hand 字段。项目 `source=lingxi_x2`，示例为 `upper_body_example`；示例使用默认 QoS depth=10，项目保持接口文档要求的 BEST_EFFORT/VOLATILE。示例的 arm 为全零且手为较大固定姿态，不能原样在实机运行来比对。两份示例均未暴露额外位置使能或插值字段。

MC 旧日志中三次均存在 `PreRunOnce runner: upper_body_external` 及退出记录。它证明控制 runner 进入，不能证明每条位置命令被消费。下一步先审查新的逐帧数据；若发布正常仍不跟随，再检查 MC 消费条件、关节映射、内部插值和控制模式。

本轮验证与部署：

- 本机与 PC2 均为 `43 passed`，仅有既有 Starlette 弃用警告。覆盖图查询不进入逐帧路径、超时不补发、sequence 回绕/缺帧、过期反馈、故障、模式变化、竞争发布者、订阅丢失、保持非目标肢体和触觉导出。
- 本机完整 mock 会话保持/去目标/恢复分别发布 `50/100/100` 帧，约 `49.4–49.5 Hz`、最大间隔约 `21.1 ms`，恢复 `STAND_DEFAULT`。这是本机 mock 调度证据，不代表 PC2 实机发布性能。
- PC2 修改前备份：`/home/run/lingxi-backup-20260929-vbojBl/before-stream-update.tar.gz`。仅部署代码、测试及文档；`config/x2.yaml` 和 `config/x2.motion-test.yaml` 保持 `enabled: false`。
- PC2 非交互 SSH 首次未加载 ROS，报 runtime unavailable；执行 `source /opt/ros/humble/setup.bash` 后只读采样成功。
- 只读 preflight：`PASSIVE_DEFAULT/RUNNING`、FSM `6`、body `SIT(4)`、播放器空闲、14+10+10 反馈完整、arm domain 0、全部 fault 0、无竞争发布者；`ready=false` 符合预期。没有调用模式服务或发布运动命令。
- 导出实机触觉 PNG 和原始 JSON：`snapshots/20260929-tactile-readonly.*`，两手全部 282 格为零。已复制回本机同一路径；不能据此认定压力响应已验证。
- 更新后的实机运动尚未执行。下一步需操作员通过官方控制端完成站立并现场确认，重复同一 `+0.01 rad` 测试，再继续灵巧手动作与接触验收。

## 2026-09-29：修正发布循环后第四次 +0.01 rad 实机复测

操作员明确确认“已切换到稳定站立模式，测试吧”。会话前确认 `STAND_DEFAULT/RUNNING + FSM 4 + STAND(1)`、播放器空闲、14+10+10 反馈完整、全部 fault 0。首次独立 preflight 的 1 秒 DDS 发现窗口未发现上肢 Topic；执行程序使用 3 秒发现等待并重新检查，通过后才调用模式服务。控制只在进程内临时启用，两个磁盘配置一直为 `enabled: false`。

输入仍为索引 0（左肩 pitch）`+0.01 rad`，资格保持 1 秒、去目标插值 1 秒及驻留 1 秒、恢复插值 1 秒及驻留 1 秒、断流观察 0.5 秒。会话执行于 MC 日志时间 `10:45:11–10:45:17`，总耗时 `6.101365 s`。

| 发布阶段 | 帧数 | 实际 Hz | 最大帧间隔 ms | 最小帧间隔 ms | 最后命令本地回读 |
| --- | ---: | ---: | ---: | ---: | --- |
| 资格保持 | 50 | 46.618 | 27.427 | 20.389 | 匹配 |
| 去目标含驻留 | 100 | 47.105 | 25.622 | 20.369 | 匹配 |
| 恢复含驻留 | 100 | 47.548 | 25.718 | 20.371 | 匹配 |

共 250 帧，sequence `0..249` 全程连续，无补帧突发。表中间隔是每段内统计；保持到目标的跨段间隔为 `34.512 ms`，目标到恢复为 `29.579 ms`。全程最大间隔因此是 `34.512 ms`，不是 27.427 ms。单次 publish 调用最长 `2.031 ms`。实际频率低于配置的 50 Hz；当前调度仍有调度/准备开销，不能声称精确 50 Hz。

编码器结果：

- 资格保持目标轴误差 `-0.003643513 rad`，小于代码资格阈值 `0.005 rad`；其他轴最大保持误差仍为 `0.042183876 rad`，不代表全上肢准确保持通过。
- 目标轴相对基线实际变化 `-0.001150608 rad`；相对 `+0.01 rad` 命令误差为 `-0.011150608 rad`。**第四次仍未证明目标跟随**。
- 去目标阶段左肘/右肘相对基线分别变化约 `+0.036048 / +0.040267 rad`；恢复后目标轴仍相对基线偏移 `-0.004985332 rad`，左肘/右肘偏移约 `+0.048703 / +0.046402 rad`。发出恢复命令不等于准确恢复位置。
- 断流观察约 `0.503061 s`；观察点与恢复点的 14 轴位置相同。这仅是短窗口端点证据，不是长时断流安全保证。
- 会话结果与退出后的独立只读 preflight 均确认 `STAND_DEFAULT/RUNNING + FSM 4 + STAND(1)`、播放器空闲、arm domain 0、臂和双手全部 fault 0；退出后无竞争发布者。现场声音/可见运动反馈尚待操作员补充。

本地回读逐字段匹配最后命令，证明本机 DDS 发布路径可见，不证明 PC1 MC 消费。当前证据不支持把不跟随单独归因于逐帧 graph 查询或补发；下一步应优先记录 MC 下游 `/aima/hal/joint/arm/command`，与上肢命令及编码器按时间对齐，再核对 MC 消费条件、索引映射、插值与实际控制模式。暂不放大幅度或运行手部动作。

MC 日志再次确认 `upper_body_external` 进入/退出；该会话窗口未发现上肢命令拒绝/丢弃日志。另见模式切换时 `animation_player_controller` 的 `AfterExit: command hand/claw close`，属于 MC 自身的模式切换行为，应在后续手部验收前单独核对，不能把手部状态变化全部归因于项目手命令。MC 内部消费配置未在 PC2 可见目录找到；经 PC2 到 PC1 的只读 SSH 尝试因无免密认证失败，未修改 MC。

证据文件：

- PC2 `/home/run/lingxi/logs/20260929-arm-joint-0-001-stream.stdout.log` 与 `.stderr.log`，已复制到本机 `logs/`。
- 测试前后 `20260929-before-joint-stream.stdout.log`、`20260929-after-joint-stream.stdout.log`。
- MC：`/agibot/nfs/soc0/log/log_20260929_100048/mc/mc.log`，本次模式切换约行 3723–3786。
- 可复核执行程序：`scripts/acceptance_joint_session.py`；每次运行仍须现场授权，脚本不会修改磁盘控制开关。

## 2026-09-29：同步链路定位与固定命令基线复测

用户要求继续解决。新增有界内存 `ControlTrace`，同步订阅 upper command、HAL arm command、arm state、MC state；同时保存源 stamp 与 PC2 接收单调时钟。自身回读和诊断订阅者均从外部订阅者检查中排除，防止诊断工具掩盖 MC 订阅者丢失。

只读基线取得 HAL 1563 帧、编码器 1515 帧、MC 33 帧，无解析错误；此时肩 pitch 的站立 HAL 目标约 `0.4000 rad`、刚度 `40`、阻尼 `2`，编码器约 `0.3916 rad`，说明测量姿态与原有命令姿态存在静态偏差。

第五次仍使用原测试方法并增加同步记录，共取得 upper 250 帧、HAL 4418 帧、编码器 4526 帧、MC 93 帧，无缓冲丢弃和解析错误。目标阶段 HAL 左肩目标正确到达 `0.399529705 rad`，与上肢命令一致；MC 已消费并向下游发布，索引 0 的关节名也正确。实际目标阶段位置为 `0.388570786 rad`，相对测量基线 `0.389529705 rad` 变化 `-0.000958920 rad`。

这次还定位了验收方法的问题：每段重新取编码器作基线，会把全臂已有跟踪偏差重新写成位置目标，造成逐段下移。所谓 `+0.01` 甚至低于测试前站立命令 `0.4000 rad`。这能解释非目标轴反复漂移和基线混淆，**不能独自解释底层小幅跟随不足**。

第六次改用固定命令基线诊断：只读验证 0.5 秒 HAL 目标稳定、14 轴名称/顺序一致和反馈新鲜，再保持原有 HAL 命令；仅索引 0 平滑执行 `0.400000006 → 0.410000006 → 0.400000006 rad`，其他 13 轴命令全程不变。保持、去目标插值、驻留、恢复插值、恢复驻留各 1 秒；偏移仍为 `+0.01 rad`，没有改刚度、阻尼或前馈力矩。

结果：

- upper 250 帧、HAL 4480 帧、编码器 4520 帧、MC 95 帧；无缓冲丢弃/解析错误。发布 `46.105 Hz`，最大帧间隔 `26.374 ms`，sequence 连续，本地最后命令回读匹配。
- HAL 目标明确完整跟随 `0.400 → 0.410 → 0.400`，保持刚度 `40`、阻尼 `2`、前馈 effort `0`。
- 每个驻留末尾 0.2 秒窗口的编码器中位数：基线 `0.393748283 rad`，目标 `0.394323349 rad`，恢复 `0.394323349 rad`。目标实际增量仅 `+0.000575066 rad`，恢复后仍残留相同偏移，未通过位置跟随/恢复验收。
- 目标驻留的关节 effort 反馈约 `0.40–0.75 N·m`，与目标变化有响应；它不是手部压力。数据支持继续检查驱动控制和静摩擦等因素，但不足以确诊摩擦、死区或驱动参数错误。
- 结束及独立只读 preflight 均为 `STAND_DEFAULT/RUNNING + FSM 4 + STAND(1)`、arm domain 0、全部臂/手 fault 0、无竞争发布者；两个磁盘控制配置保持关闭。

只读 UDCU 与关节非实时状态各采到 279 帧。左肩 pitch 原始字段 `motor_temp=41, board_temp=46, mcu_temp=44, vbus=53, motor_current_state=1`；未找到字段 1 的可靠枚举定义，不将其直接解释成某种控制模式。HAL 启动日志含早期 hand_info uninit，随后 DCU 检查和 servo enable 成功；启动瞬间日志不能直接当成本次运行故障。PC2 可见的历史 `MotionControlModule/runtime.yaml` 显示手型 NONE，但实时 MC/hand state 均为类型 1，未修改该历史文件。

工程变更：

- 实机旧 `test-arm-joint` / `test-arm-joint-session` 现在在任何模式服务或运动 I/O 前拒绝，mock 测试保留。避免继续使用已知会重设全臂基线的方法。
- 新工具 `scripts/command_baseline_session.py` 使用固定、已观测的 HAL 命令基线；仍是现场授权诊断，不是通过验收的机械臂控制器。
- `scripts/analyze_control_trace.py` 可输出驻留窗口统计和上下游对齐图。
- 本机/PC2 测试更新为 `50 passed`，仅有既有 Starlette 弃用警告。

证据：本机与 PC2 的 `logs/20260929-control-path-readonly.json`、`logs/20260929-control-path-joint.json`、`logs/20260929-fixed-baseline-joint.json`；分析 `logs/20260929-fixed-baseline-analysis.json`，曲线 `snapshots/20260929-command-hal-encoder.png`。只读驱动遥测保存在 `logs/20260929-drive-readonly.stdout.log`。

下一步需读取 PC1 的实际 MC/驱动配置及固件语义；当前 PC2→PC1 的 run 免密登录被拒绝，已请求登录方式。没有提高增益、增加力矩、扩大幅度、停 MC 或尝试直连总线。平台仍不能宣称实机闭环已验收。

## 2026-09-29：PC1 只读控制配置核查

用户提供 PC1 登录方式后已成功登录。主机公钥与本地旧记录不同，使用既有可信 PC2 保存的 ED25519 指纹交叉核对一致后建立独立 known-hosts 记录；没有删除或禁用全局 SSH 校验。

运行中的 MC 配置路径和 URS 参数已确认，实际 14 轴增益均为 `40/2`。代码/同步记录均表明该路径输出速度、前馈力矩为零。腕部 YAML 书写顺序虽不同，但实测 14 轴与 HAL 按关节名逐项对应，没有发现指令交换。

对固定基线历史记录的 2077 个时间对齐样本分析：reported effort 对位置误差的线性斜率约 `38.86 N·m/rad`，R² 约 `0.931`，与配置 kp=40 相符；0.01 rad 约为 52 个位置量化阶梯，不能用分辨率不足解释约 3 个阶梯的实际变化。仍未确诊机械静摩擦、负载/补偿或驱动内部控制行为。

新增 [PC1 控制核查报告](pc1-control-audit.md) 和单轴 kp=60 的待审查候选补丁，未应用到机器人。参数在控制器 Init 中读取，未确认安全热加载，未重启 MC、未调整增益、未发送本轮新动作。

API 现在拒绝 upper_body_mc 路径不能传递的非默认增益及非零速度/力矩，防止静默忽略。相关本机与 PC2 测试为 `54 passed`，仅有既有 Starlette 弃用警告。两个控制配置仍关闭。后续参数实验需要现场安全条件与控制力调整范围确认，或先取得厂商 URS 推荐参数。

## 2026-09-29：现场确认后的 +0.02 rad 固定基线测试

操作员在获知分步方案后确认安全并允许测试。本次只执行左肩 pitch（索引 0）`+0.02 rad`，没有执行 `0.2 rad`，没有调增益或重启 MC。测试前独立只读检查满足 `STAND_DEFAULT/RUNNING + FSM 4 + STAND(1)`、播放器空闲、无竞争发布者、臂/双手 fault 全零。

固定 HAL 命令基线正确执行 `0.400000006 → 0.420000006 → 0.400000006 rad`，其余 13 轴命令不变。每个驻留末尾 0.2 秒窗口的编码器中位数：

| 阶段 | 左肩 pitch 编码器 rad | 相对基线 rad | reported effort 中位数 N·m |
| --- | ---: | ---: | ---: |
| 基线保持 | 0.393748283 | 0 | 0.131866 |
| 目标驻留 | 0.397007942 | +0.003259659 | 0.835159 |
| 恢复驻留 | 0.395857334 | +0.002109051 | 0.043961 |

实际增量仅为指令增量约 16.3%，目标绝对误差约 `-0.022992064 rad`；返回原命令后未准确回到测量基线。其他 13 轴在上述驻留中位数间最大变化约 `0.000191689 rad`（不是全程峰值）。响应较 +0.01 rad 明显，但位置跟随/回位仍未通过，因此停止加幅。这些数据不足以独自确诊静摩擦或驱动控制原因。

共发布 250 帧，实际 `46.3847 Hz`，最大帧间隔 `27.7516 ms`，sequence 连续，本地最后命令回读一致。同步采集 upper 250、HAL 4338、编码器 4352、MC 92 帧，无缓冲丢弃或解析错误。会话退出及独立只读复查均确认稳定站立、arm domain 0、臂/手 fault 全零、无竞争发布者；两个磁盘控制开关保持关闭。恢复模式不等同于编码器准确回位。

按事件单调时间，基线保持/去目标/目标驻留/恢复插值/恢复驻留实际耗时分别为 `1.052749 / 1.073973 / 1.083078 / 1.080259 / 1.100269 s`，断流观察 `0.506514 s`。随后独立站立复查的单点编码器为 `0.394706726 rad`，距基线驻留中位数约 `+0.000958443 rad`；不能与恢复驻留窗口混用，也不改变目标跟随失败的结论。

脚本新增受限 `--delta-rad` 参数，默认仍为 0.01，硬上限仍为 0.02；同时将模式进入调用纳入恢复 `finally`，使进入等待异常也尝试恢复。正负边界和拒绝 0.2/非有限值的测试已覆盖；本机与 PC2 均通过 61 项测试。

证据：本机与 PC2 `logs/20260929-fixed-baseline-002-confirmed.json`；前后只读记录 `logs/20260929-before-002-confirmed.json`、`logs/20260929-after-002-confirmed.json`。本机分析 `logs/20260929-fixed-baseline-002-analysis.json`，曲线 `snapshots/20260929-command-hal-encoder-002.png`。

## 当前尚未验证（汇总）

- `/mc/upper_body_command` 的准确姿态保持和单关节小幅运动。
- 触觉对已知接触/载荷的非零响应。
- 指令断流、进程退出后的 MC 行为。
- Web 在真实相机下的持续帧率与稳定性。

单元测试、mock 测试、消息 schema 或 ROS 图发现不等于实机运动验证。每次现场测试后应追加日期、操作员、安全状态、输入、观测、频率统计、恢复结果和日志路径。

## 2026-09-29：本机采集、回放、影子推理与 Web 扩展

- 新增严格逐行 JSONL 回放、文件摘要与离线 CLI；拒绝截断、重复键、非有限值、倒退采集时间和不完整轴序。省略图像保持显式，不向模型提供伪造图像。
- 新增 `JointActionAdapter` 和 `ShadowExperimentRunner`：明确绝对关节弧度输出，日志记录观测、提案、耗时、超时及异常，动作执行数始终为 0。模型插件仅允许可信代码；没有声称推理被强制超时或沙箱隔离。
- Web 新增双手 282 格压力、源时间/接收龄、后台采集/停止、随机 ID 文件库、JSONL 下载和逐帧回放。实时与历史分区，固定 raw_uint8 色标；读取错误不再吞掉。
- Web 控制改为发布统计及 `motion_verified=false`，禁止当前实机网页动作。`ExperimentRunner` 同样拒绝实机动作循环，保留 mock；真实闭环需完成逐级物理验收后再接入。
- 修复右臂误复用左臂位置限位，按官方保证范围与已审核的官方 v1.3、PC2 SDK 模型交集检查；未修改 MC 参数或模型文件。新增镜像 roll 及全部 14 轴边界测试。
- 本机非 HTTP 测试：**161 passed**，包括采集取消、失败/中断恢复、路径约束、路由结果和硬件拦截；日志 `logs/20260929-platform-local-tests.txt`。
- 默认 asyncio 下，Web HTTP 测试在 AnyIO TestClient portal 入口阻塞，12 秒超时中止；线程栈保存在 `logs/20260929-web-http-tests.log`。使用已安装的 uvloop 策略后，同一套 HTTP 测试 **3 passed**，含新采集端到端接口测试；日志 `logs/20260929-web-uvloop-tests.txt`。未修改应用或测试来规避断言。1 个 Starlette/httpx 弃用警告保留。
- 最终以 uvloop 策略运行全量测试：**164 passed, 1 warning in 2.26s**，日志 `logs/20260929-platform-full-uvloop-tests.txt`。复现：`.venv/bin/python -c 'import uvloop, pytest; uvloop.install(); raise SystemExit(pytest.main(["-q", "-o", "addopts="]))'`。
- `node --check`、Python 编译和 `git diff --check` 通过；尚未完成浏览器视觉验收。
- mock 演示：`logs/platform-e2e-20260929-165706/`，含完整 RGB 字节/压力的 6 帧 JSONL、校验摘要和 6 次反馈回显提案日志，`executed_actions=0`。这是数据流程证据，不是 VLA 权重测试或实机证据。
- 本轮代码尚未同步到 PC2：原 SSH 控制连接失效；自动审批拒绝向新 SSH 会话输入密码（需批准，当前策略不允许申请）。PC2 上一次确认仍为 112 项测试通过的软件阶段。没有因同步失败修改主机密钥检查。
- `config/x2.motion-test.yaml` 仍为 `control.enabled: false`，本机无站点 `config/x2.yaml`。未执行新的实机运动；单轴跟随/回位、多轴、手部接触及模型真实闭环均未验收。

## 2026-09-29：验收判定与新 SSH 会话只读复查

- 新增离线 `acceptance-report`，从原始同步记录重算质量、跟随、绝对位置、URS回位及重复性，含文件SHA256、明确工程阈值和非零未通过退出码。两份现有记录质量检查全部通过、运动结论均fail；+0.01与+0.02不能构成同条件重复性。
- 本机该阶段全量 **190 passed**，`logs/20260929-platform-acceptance-tests.txt`；对应冻结安装包和带备份安装器已准备并在临时项目验证。未修改等待用户执行的冻结包。
- 用户在本机终端建立SSH主连接后，工具曾成功复用到PC2；只读复核两份控制YAML为false，新版文件/manifest尚不存在。上传scp被沙箱以 `Operation not permitted` 拦截；这与密码认证失败不同。
- 使用PC2已有软件进行只读记录：新的非交互SSH会话未加载ROS underlay，初次运行时报运行时不可用；source `/opt/ros/humble/setup.bash` 后能建立ROS连接并收到臂、手、触觉状态，但前置RGB等待1秒超时。`logs/20260929-readonly-platform-episode-01.jsonl` 仅有元数据1行、观测0行，**采集未通过**。未关闭相机要求伪造一次全传感器成功。
- 后续较长相机诊断及另一段只读记录在连接阶段就被工具网络沙箱拦截，没有启动机器人侧采集进程；本地 `logs/20260929-pc2-camera-diagnostic.log` 记录的是连接拒绝，不是相机诊断结果。
- 本机修复ROS环境初始化：只在AimDK前缀存在时加入已有的ROS Humble/Python3.10路径，AimDK覆盖层优先，保留原有路径，重启不重复；运行时错误保留底层导入异常。新增4项测试，最终 **194 passed**（uvloop），见 `logs/20260929-platform-environment-tests.txt`。尚未部署或在PC2验证此修复。
- 没有新实机运动、模式切换、增益/MC/固件修改。完整缺口见 [平台完成度](platform-status.md)；目标仍未完成。

## 2026-09-29 17:35–17:43：PC2部署、真实只读流程和浏览器验证

网络权限恢复后，直接完成PC2部署，无需用户继续执行之前的上传命令。

- 包 `logs/platform-software-20260929-runtime.tar.gz`，SHA256 `6ff010008998ff793a5e5725af9002ff3e095da757cbb6a2ecf6b1050ab5e75f`。安装器校验79个文件并备份旧代码至PC2 `logs/platform-backup-20260929-173540-538442.tar.gz`；两份现场YAML字节未改，`control.enabled=false`。
- PC2全量 **194 passed, 1 warning in 5.62s**，本机默认asyncio全量亦通过。部署/测试证据：`logs/20260929-pc2-runtime-deploy-tests.txt`。复核manifest无文件差异。
- 新SSH会话未手动source ROS，使用更新后的环境初始化即可建立ROS。只读诊断 `logs/20260929-platform-readonly-02.json`：MC为 `STAND_DEFAULT/RUNNING`、wire4/body1；仅控制开关及URS模式检查未满足；无命令发布者被创建，没有请求模式切换。
- 前置RGB有1个RELIABLE/VOLATILE发布者，订阅建立后读到1280×720 JPEG；同一客户端分别录得3秒5Hz的关节/触觉15帧、RGB/关节/触觉15帧。先前首次1秒读取超时的失败记录保留；本轮成功不足以确定首次超时根因。
- 完整RGB记录 `logs/20260929-platform-readonly-02-rgb-joints-tactile.jsonl`，SHA256 `0e71345d03f2c237c32956c1e47b08ee8c9452020558fc3d6d53407d8de4c19a`。本机严格校验通过、所有15张JPEG可解码且尺寸一致；图像未省略。双手压力全部0，接触响应仍未验收。
- 同一真实记录进入本机反馈回显适配器，15次影子提案，`executed_actions=0`；这是模型接口流程验证，不是VLA/WAM权重测试。证据 `logs/20260929-platform-real-inspection.json`、`logs/20260929-platform-real-shadow.jsonl`。
- 临时Chromium分别操作mock与PC2控制台：实时14个压力canvas、RGB显示、后台1秒5Hz完整采集、JSONL下载、回放第2帧均成功；两套各5帧，JavaScript错误0。PC2现场确认框勾选后，臂/手按钮仍禁用；没有调用控制接口。证据 `logs/20260929-browser-checks.json`、`logs/20260929-browser-ros2-episode.jsonl`。
- 桌面1440px与手机390px截图已检查；monitor/data/control均无横向溢出。测试浏览器运行环境缺中文字体，临时补入字体后复核中文显示正常，不修改应用依赖或PC2系统包。证据 `logs/20260929-browser-layout-checks.json` 和 `logs/20260929-browser-*-{monitor,replay,mobile}.png`。
- 保留PC2 localhost:18081只读Web进程，通过本机SSH转发访问 `http://127.0.0.1:18081`。停止了仅用于验证的本机mock Web进程。该控制台随SSH会话/进程生命周期运行，未注册系统服务，不操作MC。
- 单轴跟随/回位、重复性、多轴、接触压力及模型真实闭环仍未通过；当前目标未完成。

## 2026-09-29：负向小步与重力增量评估

用户逐次现场确认后，第一次负向试验因右拇指roll故障码1在轨迹前中止（0帧）并恢复站立。用户修复并重新确认后，第二次−0.01rad轨迹完成250帧，但实际位移仅−0.000575542rad、跟随失败；URS回位残余−0.000383854rad。结束臂/手故障全零，磁盘控制关闭。完整数值、曲线和独立只读重力快照见 [重力与跟随评估](gravity-following-assessment.md)。没有在线补偿、改增益或加幅。

新增离线恒定重力方向的力矩差上界计算。三份trace中的比例项变化均显著超过原模型重力增量上界；结论仅针对静态/固定重力方向/原文件惯性的受限假设，未确诊原因。后采集IMU未用于伪造同期姿态。

## 2026-09-29：平滑轨迹与有界重力位置补偿的软件实现

- 独立实现五次插值、选轴静态 `tau_g/K` 偏置、显式限幅记录、渐入/渐出、固定基线返回。整条计划在发布前检查偏置/命令变化率、总偏移与两套关节限位。
- 新增 `plan-compensation`、核验模型哈希与全部帧的 `replay-compensation`、实际 MockBackend 专用的 `mock-compensation`；实机补偿执行硬拒绝。不修改 MC 或现场控制配置。
- 本机全量 **230 passed, 1 warning in 17.02s**；新增34项测试涵盖解析可求解模型、补偿开关对照、选轴/限幅、固定基线回位、超限整段拒绝、过期反馈中止、实际后端检查、回放篡改拒绝等。证据 `logs/20260929-compensation-local-tests.txt`。
- 已审计 PC2 URDF + 明示离线重力/负载假设的示例：276帧，mock实测49.30275Hz，最大间隔20.55029ms，无超过1.5周期的间隔；末帧mock反馈精确返回初始基线。它不模拟摩擦/重力驱动响应，不能作为补偿有效或运动验收证据。
- 计划、逐帧mock日志、结果及核验回放分别保存在 `logs/20260929-compensated-shoulder-{plan.json,mock.jsonl,mock-result.json,replay.jsonl}`；预览曲线 `snapshots/20260929-compensated-shoulder-preview.png`。运行方法见 [补偿轨迹](compensated-joint-sessions.md)。

PC2首轮部署13个文件，备份 `logs/before-compensation-1790677325335099844.tar.gz`，全量230项通过。PC2计划/回放/mock三条CLI完成，mock276帧、49.35497Hz、最大间隔22.27820ms；87文件manifest匹配，两份现场配置仍关闭。

随后跨机器核验发现CPU/NumPy浮点舍入差最大4.44e-16，原精确比较误拒绝。回放改为仅对计算结果允许绝对误差≤1e-12，模型哈希/输入/结构仍精确核对；新增舍入容许及较大改动拒绝测试。本机最终 **232 passed, 1 warning in 18.66s**，PC2生成的276帧计划已在本机通过核验。证据 `logs/20260929-compensation-local-tests-final.txt`。

## 2026-09-29 18:35：无补偿五次平滑单轴实机测试

用户本次重新确认机器人稳定站立并要求测试。执行前连续6次只读检查均为 `STAND_DEFAULT/RUNNING`、wire4/body1、臂14轴/双手各10轴故障全零、无竞争发布者；左肩编码器0.395857334rad稳定。没有沿用历史现场确认。

将固定基线诊断脚本增加可选 `--profile quintic --ramp-duration 2`。本次仅左肩pitch +0.01rad，基线/目标/恢复各驻留1秒，去程和回程各2秒，共350帧；名义峰值速度小于此前1秒线性测试。其他13轴始终固定HAL命令基线，双手保持同一测量目标。**重力位置偏置未启用**，仍受模型/IMU安装方向等未核实条件限制；本次不是补偿效果实验。

| 驻留末尾0.2秒中位数 | HAL目标 rad | 左肩编码器 rad | reported effort N·m |
| --- | ---: | ---: | ---: |
| 基线 | 0.400000006 | 0.395665646 | 0.04396057 |
| 目标 | 0.410000006 | 0.397007942 | 0.39559937 |
| URS内回位 | 0.400000006 | 0.396816254 | 0.04396057 |

实际增量 **0.0013422966rad，13.422966%**；回位残余 **0.0011506081rad**；目标绝对误差−0.0129920638rad。现有工程验收阈值未放宽：增量跟随误差、绝对误差、回位残余三项失败；其他轴漂移0.00095892rad和末尾窗口稳定性通过。所有证据质量检查通过，因此结论为 **fail**，不是数据不足。

本地发布实测45.67298Hz，最大间隔31.67653ms，1个间隔超过1.5周期；sequence连续。四路同步记录无缓冲丢弃、无解析错误，MC→HAL完整传递目标。未发生运行异常；finally恢复站立，最终及随后6次独立站立检查均通过，臂/手故障全零，两份磁盘配置字节不变且控制关闭。后续独立站立编码器0.395473957rad不能替代上表URS回位窗口。失败后停止，没有加幅、重复试跑、开启重力偏置或修改MC/增益。

证据（本机与PC2）：

- `logs/20260929-fixed-baseline-quintic-001-confirmed.json`，SHA256 `0f6295eb89b132f0d34e0dfbdbd941618d844d3b4e70bb51e00425ae037cd52a`。
- 同前缀 `-verdict.json`、`-analysis.json`，以及 `logs/20260929-{before,after}-quintic-001.json`。
- `snapshots/20260929-command-hal-encoder-quintic-001.png`。

软件同时更新重复性判定，禁止混合不同插值方式/斜坡时长的试验。相关41项测试在两端通过后才执行本次动作；后续完整回归本机 **238 passed in 20.35s**、PC2 **238 passed in 28.17s**，均有同一个Starlette弃用告警。测试日志为 `logs/20260929-quintic-full-{local,pc2}-tests.txt`。
