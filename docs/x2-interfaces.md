# X2 接口清单

## 版本与证据

- 官方在线文档：AimDK X2 1.1.0，2026-09-28 访问。
- 当前实机：X2 Ultra，SN `X220028C5Z0034`，Agi `release-lx2501_3_t2d5-*-v1.1.4`。
- PC2 OS 镜像：`lx2501_3_t2d5-soc1-v0.6.10-hotfix_v9`。
- PC1/SoC0：`10.0.1.40`，实时内核，运行 MC 与 EtherCAT HAL。
- PC2/SoC1：`10.0.1.41`，Jetson Orin NX，运行传感器与二开程序。
- 实机消息源：`/agibot/software/common/share/aimdk_msgs`；冲突时以此为准。

## 官方接口到项目 API

| 官方能力 | Topic/Service | 消息类型 | 项目封装 | Python API |
| --- | --- | --- | --- | --- |
| 双臂状态 | `/aima/hal/joint/arm/state` | `aimdk_msgs/msg/JointStateArray` | `Ros2Backend._on_arm` -> `ArmState` | `X2Client.arm_state()` |
| 双臂底层控制 | `/aima/hal/joint/arm/command` | `JointCommandArray` | `publish_arm` | `move_arm(ArmCommand)` |
| 上肢统一控制 | `/mc/upper_body_command` | `UpperBodyCommandArray` | 动态存在性检查 | `move_arm` / `move_hand` |
| 灵巧手状态/触觉 | `/aima/hal/joint/hand/state` | `HandStateArray` | `HandState` / `TactileFrame` | `hand_states()` / `tactile_frames()` |
| 灵巧手控制 | `/aima/hal/joint/hand/command` | `HandCommandArray` | `publish_hand` | `move_hand(HandCommand)` |
| RGB-D RGB | `/aima/hal/sensor/rgbd_head_front/rgb_image/compressed` | `sensor_msgs/msg/CompressedImage` | `CameraFrame` | `camera_frame(RGBD_FRONT_RGB)` |
| RGB-D depth | `/aima/hal/sensor/rgbd_head_front/depth_image` | `sensor_msgs/msg/Image` | `CameraFrame` | `camera_frame(RGBD_FRONT_DEPTH)` |
| 双目左右 | `/aima/hal/sensor/stereo_head_front_{left,right}/rgb_image/compressed` | `CompressedImage` | `CameraFrame` | `camera_frame(STEREO_FRONT_*)` |
| 后视 RGB | `/aima/hal/sensor/rgb_head_rear/rgb_image/compressed` | `CompressedImage` | `CameraFrame` | `camera_frame(HEAD_REAR)` |
| 模式设置 | `/aimdk_5Fmsgs/srv/SetMcAction` | `aimdk_msgs/srv/SetMcAction` | `set_motion_mode` | `X2Client.set_motion_mode()` |

ROS action 实机列表为空；`SetMcAction` 是 Service，不是 ROS Action。

## v1.1.4 实机结果

- `UpperBodyCommandArray` 和 `/mc/upper_body_command` 均存在；复验时 MC 有一个订阅者，Topic 没有发布者。只验证了接口发现，未发布命令。
- `/aima/mc/common/state` 存在并持续发布 `McCommonState`；采样时动作是 `PASSIVE_DEFAULT`。
- `HandStateArray` 包含左右 `HandTouchSensorData`；两侧均为 `NIMBLE_HANDS=1`，每侧返回 10 个关节。
- 静止状态下观测到的触觉数组全部为零。字段存在已验证，但传感器受力响应尚未验证。
- `JointState` 已使用 `uint16 error_code`，不再包含旧温度/电压字段。
- ROS Action 列表仍为空；`SetMcAction`、`GetMcAction`、`GetHandType` 均为 ROS Service。
- HAL arm/hand command Topic 仍由 `mc_ros2_node*` 发布。使用上肢统一接口时不应停止 MC；仅 HAL 回退路径需要接管底层控制权。

## v0.9.7 历史差异

- v0.9.7 `JointState` 字段末尾是 `coil_temp/motor_temp/motor_vol`；1.1 文档写 `error_code`。项目用动态字段读取。
- v0.9.7 `HandStateArray` 只有左右类型和左右 `HandState[]`，没有 `left_touch_sensors/right_touch_sensors`。
- v0.9.7 没有 `UpperBodyCommandArray`，ROS 图也没有 `/mc/upper_body_command`。
- v0.9.7 曾出现 `aima topic list` 因 `127.0.0.1:50587` 监控 RPC 未监听而失败；该结论不自动沿用到升级后的环境。
- v0.9.7 默认 shell 没有 source `aimdk_msgs` ament 前缀；v1.1.4 复验仍需显式加入 `/agibot/software/common` 才能使用 `ros2 interface show aimdk_msgs/...`。

## 官方程序关系

PC1 MC 发布 HAL 手臂/手部命令并负责腿、腰和全身稳定。v1.1.4 应优先使用 `/mc/upper_body_command`，由 MC 完成仲裁和下发；只有直接控制底层 HAL 时，官方才要求先在 PC1 停止 MC。停止 MC 会影响全身原生运控，不能在无人值守、站立未支撑时执行。

本轮没有调用 PC1 `stop-app` / `start-app`，也没有发送运动命令。上肢统一控制的运动、模式切换和退出保持行为仍需现场验收。

## 官方来源

- https://x2-aimdk.agibot.com/zh-cn/latest/about_agibot_X2/index.html
- https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/joint_control.html
- https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/endeffector.html
- https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/upper_body_control.html
- https://x2-aimdk.agibot.com/zh-cn/latest/Interface/hal/sensor.html
- https://www.agibot.com.cn/filepage/291.html
- https://www.agibot.com.cn/DOCS/OS/Omnihand-O10
