# Lingxi X2

Lingxi X2 是面向 AgiBot X2 / X2 Ultra 的具身智能实验平台。它把 AimDK / ROS 2 接口封装为稳定的 Python API、CLI、采集格式和 Web 控制台，使 VLA、WAM、WLA 与自定义策略不必直接处理 ROS 消息。

当前代码覆盖双臂、OmniHand、RGB/RGB-D/双目相机、触觉、同步观测、连续采集、策略循环、模拟后端和 Web 前端。能力是否可用由连接机器的固件决定，不会用模拟数据替代实机能力。

## 当前实机结论

2026-09-29 最近一次无补偿跟随测试：左肩 pitch `+0.01 rad` 五次平滑轨迹，去程/回程各2秒，
实际位移仅 `+0.001342 rad`（13.4%），URS 回位驻留残余 `+0.001151 rad`，跟随/回位未通过。
该次没有启用重力位置补偿。MC→HAL 目标传递正确，结束后恢复稳定站立，臂/手故障全零。
随后已完成 [首次实机有界部分重力补偿试验](docs/gravity-live-first-trial.md)：
仅左肩 +0.002rad 偏置渐入/保持/渐出，350帧，HAL目标正确，编码器仅变化约0.000192rad。
补偿通路已实测，完整位置跟随、回位与重复性验收仍未完成；本次没有叠加+0.01rad动作。
磁盘控制配置保持关闭。新增 [开源 IK/重力补偿审查与离线工具](docs/upstream-ik-audit.md)：
可运行 `x2 gravity-report` 做模型计算；实机补偿仅限专用单轴诊断，VLA/WAM 闭环仍待验收。
已新增 [五次平滑轨迹与有界重力位置补偿](docs/compensated-joint-sessions.md)：
`plan-compensation` 离线规划、`replay-compensation` 核验回放、`mock-compensation` 模拟执行。
支持补偿开关、选轴、偏置限幅/变化率检查、固定基线回位及逐帧记录；通用执行器仍限mock，专用实机试验需逐次现场确认。
后续已找到 [官方模型、手部 URDF 并完成 PC2 只读对照](docs/official-model-audit.md)，
新增模型比较和 IMU/腰角采集校验；该阶段两端全量测试 112 项通过。机型/负载与安装外参仍需核清。

后续本机新增 [采集/回放/模型提案实验流程](docs/experiments.md)、网页双手压力网格、后台采集与历史逐帧查看，
并修复右臂 roll 限位方向。当前网页和模型动作循环仅开放 mock；实机继续使用专用现场验收脚本。
此前新增离线实机验收判定和新终端 ROS 环境修复；该阶段本机全量测试在 uvloop 下 194 项通过。
该阶段版本已部署到 PC2，两端194项测试通过；真实RGB/关节/触觉采集、离线回放和浏览器只读流程已验证，仍未通过实机运动验收。详细证据见 [平台完成度](docs/platform-status.md)；测试范围与限制见 [验证记录](docs/validation.md)。

2026-09-29 后续实现 [650帧重力补偿开关匹配对照与离线比较](docs/gravity-comparison.md)，补强发送目标URDF限位检查；本地330项测试通过，尚未部署或执行新实机试验。最新接续见 [开发交接](docs/session-20260929-gravity-comparison.md)；前一阶段 [离线收尾](docs/session-20260929-offline-handoff.md) 保留。

用户确认硬件为X2 Ultra + O10后，已重新从官网下载URDF并完成 [模型兼容性核查](docs/x2-ultra-o10-compatibility.md)。机器人版本与主动轴数量相符，但腕到掌变换、手角度映射和惯性分账尚未对齐，不能直接拼接用于补偿。

以下为 2026-09-28 的只读接口发现记录，不代表最新运动验收状态。

2026-09-28 对升级后的 `X2 Ultra / Agi v1.1.4`（PC1 `10.0.1.40`、PC2 `10.0.1.41`）进行了只读复验：

| 能力 | 实机接口 | 当前结论 |
| --- | --- | --- |
| 双臂状态 | `/aima/hal/joint/arm/state` / `aimdk_msgs/msg/JointStateArray` | 实时发布，约 500 Hz；短测出现一次 0.333 s 调度间隔 |
| 双臂控制 | `/mc/upper_body_command` / `UpperBodyCommandArray` | 已存在且 MC 有订阅者；未发送运动命令 |
| 灵巧手状态 | `/aima/hal/joint/hand/state` / `HandStateArray` | 左右各 10 轴，约 200 Hz；静止样本为零 |
| 灵巧手控制 | `/mc/upper_body_command` / `UpperBodyCommandArray` | 与上肢统一控制；未发送运动命令 |
| 相机 | `/aima/hal/sensor/...` / `sensor_msgs` | RGB-D 彩色/深度约 30 Hz；深度为 1280x720 `16UC1` |
| 手部触觉 | `HandStateArray.left/right_touch_sensors` | 字段和实时帧已出现；静止样本全零，尚未做受力验证 |
| 运控状态 | `/aima/mc/common/state` / `McCommonState` | 已读取实时状态；复验时为 `PASSIVE_DEFAULT` |

升级清除了 PC2 的 `/home/run` 用户数据；项目已重新部署。HAL 手臂和手部命令仍由 PC1 原生 MC 发布，新的上肢接口当前无发布者、MC 有一个订阅者。没有停止或修改 PC1 服务。

详细证据见 [X2 接口清单](docs/x2-interfaces.md) 和 [验证记录](docs/validation.md)。

## 架构

```text
X2 Hardware / PC1 MC / PC2 ROS 2 / AimDK
                  |
       ROS2Backend | MockBackend
                  |
       typed models + safety interlocks
                  |
              X2Client
          /        |        \
   Experiment   CLI/JSONL   FastAPI/Web
```

底层适配、统一 API、实验层和交互层相互独立。`mock` 后端用于离线开发；`ros2` 后端只动态加载 PC2 上与固件匹配的 `aimdk_msgs`，不在 PyPI 猜测或复制消息定义。

## 环境与安装

- 通用开发：Linux/macOS，Python 3.10-3.12，`uv`（仓库通过 `.python-version` 固定使用 3.10，与 PC2 一致）
- X2 实机：PC2 Ubuntu 22.04、ROS 2 Humble、与固件匹配的 `/agibot/software/common`
- 不在 PC1 部署本项目；官方明确禁止把运控计算单元作为二开运行环境

```bash
uv sync
uv run pytest
```

不需要手工创建虚拟环境。完整说明见 [安装](docs/installation.md)。

## 启动

离线模拟：

```bash
uv run x2 --config config/mock.yaml status
uv run x2 --config config/mock.yaml web --host 127.0.0.1 --port 8080
```

PC2 只读连接：

```bash
cp config/x2.example.yaml config/x2.yaml
uv sync
uv run x2 --config config/x2.yaml doctor --sample
uv run x2 --config config/x2.yaml web --host 0.0.0.0 --port 8080
```

打开 `http://10.0.1.41:8080`。统一入口与所有 demo 见 [启动说明](docs/startup.md)。

## Python API

获取图像、关节和触觉：

```python
from lingxi_x2 import X2Client
from lingxi_x2.models import CameraName

with X2Client("config/x2.yaml") as robot:
    frame = robot.camera_frame(CameraName.RGBD_FRONT_RGB)
    arm = robot.arm_state()
    hands = robot.hand_states()
    tactile = robot.tactile_frames()  # v1.1.4 schema 已支持；是否有非零压力取决于手部硬件/状态
```

控制机械臂：

```python
from lingxi_x2 import ArmCommand, X2Client

target = (-0.1, 0.1, 0.0, -0.2, 0.0, 0.0, 0.0) * 2
with X2Client("config/x2.yaml") as robot:
    robot.move_arm(ArmCommand.from_positions(target, duration_s=3.0), confirm_hardware=True)
```

控制灵巧手：

```python
from lingxi_x2 import HandCommand, X2Client

with X2Client("config/x2.yaml") as robot:
    command = HandCommand.from_positions("right", [0.0] * 10, duration_s=1.0)
    robot.move_hand(command, confirm_hardware=True)
```

实机写入默认关闭。以上调用还必须满足配置授权、反馈新鲜、向量长度、关节限位、MC 模式/安全状态、动作空闲和命令 Topic 无竞争发布者等检查。v1.1.4 优先使用 MC 上肢接口，但它仍未完成运动验收，不能直接照抄示例运行；HAL 回退路径仍要求先停止 MC。

## CLI 与采集

```bash
uv run x2 --config config/x2.yaml status
uv run x2 --config config/x2.yaml preflight arm
uv run x2 --config config/x2.yaml observe --camera rgbd_front_rgb
uv run x2 --config config/x2.yaml capture rgbd_front_rgb snapshots/front.jpg
uv run x2 --config config/x2.yaml record recordings/run-001.jsonl --duration 60 --rate 10 --camera rgbd_front_rgb
```

JSONL 首行记录能力/版本元数据，之后每行一帧结构化观测并立即 flush。详细格式与策略接入见 [实验](docs/experiments.md)。

## 常见问题

- 升级后项目或 `uv` 消失：v1.1.4 升级已观察到 `/home/run` 被重置；重新部署并执行 `uv sync --frozen`。
- 找不到 `aimdk_msgs`：CLI 会补齐 `/agibot/software/common` 环境并重启自身；自定义脚本应通过 `uv run x2` 或先使用 PC2 官方环境。
- 控制被 `competing publishers` 拦截：MC 仍在发布，这是预期保护。不要提高发布频率抢占。
- 触觉全零：v1.1.4 schema 已有字段，但静止全零不等于压力链路已验证；按厂商说明确认手部硬件和触发条件。
- 相机带宽高：原始 RGB 仅在 PC2 本机订阅；跨机/Web 使用压缩话题。

更多问题、恢复方式和安全边界见 [故障排查](docs/troubleshooting.md)。
