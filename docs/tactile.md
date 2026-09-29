# 压力与触觉

AimDK 1.1.x 的触觉链路为：

```text
OmniHand tactile firmware
 -> /aima/hal/joint/hand/state
 -> aimdk_msgs/msg/HandStateArray
 -> left/right_touch_sensors: HandTouchSensorData
 -> Ros2Backend -> TactileFrame
```

每侧结构：掌面前 25 字节（5x5）、手背 36 字节（6x6）、五个指尖各 16 字节（4x4）。数据类型是 `uint8`。官方没有给出 N、Pa 等物理单位或标定曲线，因此项目单位明确为 `raw_uint8`，不能称为力值。

```python
frames = x2.tactile_frames()
left_palm = frames[Side.LEFT].palm.values
```

官方示例标注“T2.1 支持”，并说明至少给手发送一次运动指令后才会上报压力。项目不会为了触发传感器而自动运动手指；启动动作必须由实验人员显式发出。

## v1.1.4 实机结果

只读导出双手原始触觉图及同名 JSON：

```bash
source /opt/ros/humble/setup.bash  # PC2 非交互 SSH 环境需要先加载 ROS
uv run x2 --config config/x2.yaml tactile-map snapshots/tactile.png
```

PNG 使用固定 `0..255` 色标，按消息数组顺序排列；网格在物理表面上的朝向尚未验证。JSON 保留时间戳、原始数组和 source。反馈超过 `ros.state_timeout_s` 时拒绝导出，避免把旧帧当实时触觉。命令不启动手部动作。

2026-09-29 只读实机导出成功：`snapshots/20260929-tactile-readonly.png` 及 `.json`，左右合计 282 个格点全部为零。图像表示原始数据，不是已标定的压力图，也未验证受压响应。

`HandStateArray` 已包含 `left_touch_sensors/right_touch_sensors`，并以约 200 Hz 发布。两侧手型均为 `NIMBLE_HANDS=1`，每侧返回 10 个关节。只读静止采样中，两侧掌面、手背和五个指尖数组全部为零。

因此目前可以确认消息 schema、实时发布和项目解析入口具备，但不能据此确认压力传感器的物理响应。官方示例说明需要至少发送一次手部运动指令后才会上报压力；本轮为只读复验，没有触发该条件，也没有施加并记录可控载荷。

## v0.9.7 历史结果

当时安装的 `HandStateArray.msg` 不含触觉字段，只有左右手类型和关节状态，因此 `tactile_frames()` 会抛 `CapabilityUnavailableError`。升级到 v1.1.4 后该限制已解除。

本项目仍不会将 `HandState.effort` 冒充压力，也不会把全零数组表述为已验证的触觉测量。
