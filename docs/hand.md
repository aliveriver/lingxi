# 灵巧手

## 接口链路

```text
AimDK /mc/upper_body_command
 -> aimdk_msgs/msg/UpperBodyCommandArray
 -> PC1 MC
 -> Ros2Backend.publish_hand
 -> X2Client.move_hand
```

v1.1.4 实机已发现该 Topic 和 MC 订阅者。`/aima/hal/joint/hand/command` 仍作为需要停止 MC 的底层回退路径。

每只 OmniHand 使用 10 个主动轴，按索引识别，不依赖可能变化或为空的 `name`：thumb roll、thumb abad、thumb mcp、index abad、index pip、middle pip、ring abad、ring pip、pinky abad、pinky pip。位置单位 rad。

v1.1 官方 `UpperBodyCommandArray` 定义已经确认：灵巧手关节控制必须设置 `hand_sub_mode=2 (HAND_DEXTEROUS_JOINT)`，`hand_pos` 固定 20 项，排列为左手索引 `0..9` 后接右手索引 `0..9`。项目 `_publish_upper()` 正是这个排列，并有离线测试固定该契约；这属于消息适配验证，不是实机手部运动验证。

```python
from lingxi_x2 import HandCommand, X2Client
from lingxi_x2.models import Side

with X2Client("config/x2.yaml") as x2:
    state = x2.hand_states()
    current_right = [joint.position_rad for joint in state[Side.RIGHT].joints]
    x2.move_hand(
        HandCommand.from_positions("right", current_right, duration_s=1.0),
        confirm_hardware=True,
    )
```

AimDK 1.1 文档说明 OmniHand 底层仅启用 `position`；`velocity/effort/acceleration/deceleration` 暂未启用。项目不把这些字段标成可控能力。官方示例给出的左右手符号直接写入数组；本项目不做未经当前固件验证的隐式左手取反。

## 控制权

2026-09-29 更新：机械臂三次 `+0.01 rad` 测试仍未证明跟随，所以首次手部实机动作和有载触觉验收继续等待。`move_hand()` 已共用新的轨迹调度和发布统计；同一轨迹内双臂及另一只手固定在开始时的保持目标。此变更已纳入离线测试，不是手部运动验证。

官方要求底层控制前在 PC1 停止 MC；v1.1.4 的上肢统一控制路径则应保留 MC。当前默认互锁仍禁止写入，需先按 [机械臂](arm.md) 的现场清单验收上肢控制模式。

v1.1.4 流式状态已直接确认左右均为 `NIMBLE_HANDS=1`，每侧返回 10 个关节。关节 `name` 当前为空，项目按官方固定索引解析。底层回退命令只填目标侧数组，另一侧为空，避免无意覆盖另一只手；上肢统一命令会同时携带双手当前目标。

首次手部动作必须在站姿上肢保持和单臂小幅测试通过后进行。先用只读 `preflight hand` 确认 `UPPERBODY_REMOTE_SPLIT/RUNNING`、FSM 线值 `4`、body `STAND(1)`、无竞争发布者、无动作播放且双手反馈各 10 轴；再以当前双臂和另一只手反馈作为保持目标，只给目标手发送极小、可恢复的变化。项目不会把 `effort` 当触觉：OmniHand 的 `velocity/effort` 是官方标注的原始反馈/待适配字段。

触觉仍标记为 `raw_uint8`。官方消息定义只确认掌面前 25 字节、手背 36 字节和五个指尖各 16 字节的数组布局，没有给出压力或牛顿换算。静止全零只证明消息可解析；必须在厂商推荐启动条件下接触已知软物体，观察可重复非零变化后，才能记录为触觉响应已验证。

OmniHand 官方独立 SDK 支持设备直连能力，但 X2 内部总线、设备 ID、HAL 仲裁和恢复流程尚未由实机接口确认，因此本项目没有在 HAL 运行时另开 CAN/串口抢占。

现可用 `x2 tactile-map snapshots/tactile.png` 只读导出双手原始触觉网格和同名 JSON；具体配置与当前全零实机结果见 [压力与触觉](tactile.md)。
