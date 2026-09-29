# 故障排查

## `aimdk_msgs` unknown package

PC2 默认 shell 的 `AMENT_PREFIX_PATH` 只有 `/opt/ros/humble`。使用 `uv run x2 ...`，CLI 会加入 `/agibot/software/common` 并重启自身。不要从别的固件复制生成的 Python 消息包。

## `aima topic list` 连接 50587 失败

v0.9.7 实机曾复现健康监控 HTTP RPC 未监听；升级到 v1.1.4 后需重新判断，不能把旧结论直接沿用。只读替代：

```bash
ros2 topic list -t
ros2 service list -t
ros2 action list -t
```

不要为了让 CLI 好看而重启未知监控服务。

## 控制提示 competing publishers

用 `ros2 topic info -v /aima/hal/joint/arm/command` 查看。PC1 `mc_ros2_node*` 在发布时，HAL 接管不成立。停止其他用户程序后 ROS 图可能缓存旧端点，可只重启当前用户的 CLI daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

这不停止机器人节点。不要把 MC 加入允许名单来绕过。

## 触觉 unavailable

检查：

```bash
ros2 interface show aimdk_msgs/msg/HandStateArray
```

若没有 `left_touch_sensors/right_touch_sensors`，当前固件 schema 不提供集成触觉。v1.1.4 实机已有这些字段；若读数全零，还需确认手部硬件、是否执行过官方要求的启动动作以及是否施加了可控接触，不能只根据 schema 判定传感器有效。

## 相机超时

确认相机已写入 `ros.camera_streams`，然后检查对应 Topic 和发布者。压缩 Topic QoS 在设备间不同，项目订阅使用 BEST_EFFORT/VOLATILE 以兼容现有发布者。原始流不要跨机订阅。

## 控制退出后怎么办

底层关节 Topic 没有失效保护，停止进程不等于安全停止。v1.1.4 应优先通过 MC 上肢接口测试，并预先定义模式恢复和断流处理；底层 HAL 现场验证仍必须预先定义保持/阻尼/恢复 MC 流程并有人持急停。
