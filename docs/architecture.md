# 架构

项目按硬件适配、领域 API、实验、入口四层划分。

```text
PC1: MC + EtherCAT HAL  <--- DDS --->  PC2: sensors + this project
                                     |
                    backends/ros2.py | backends/mock.py
                                     |
                    models.py + safety.py + X2Client
                                     |
               recording.py / experiments.py / CLI / Web
```

`Ros2Backend` 只负责 ROS 类型转换、订阅、发布和图发现；它不向上层泄漏 ROS 消息。`MockBackend` 实现同一协议。`X2Client` 负责轨迹插值、显式操作员确认和稳定的 Python 方法。`ExperimentRunner` 接收 `Observation` 并输出 `PolicyAction`，适配 VLA/WAM/WLA。

时间戳同时保存消息源 `sec/nanosec` 和本进程 `monotonic_ns`。前者用于跨传感器数据语义，后者用于新鲜度与本地调度。RGB 与 depth 在官方接口中独立发布，项目不会宣称它们硬同步。

安全边界：

- 默认配置是 `observe_only` 且 `control.enabled=false`。
- `upper_body_mc` 只在实际发现 `/mc/upper_body_command` 和对应消息类型时开放。
- `hal_mc_stopped` 会检查命令 Topic 的其他发布者；发现 MC 就拒绝写入。
- Web/CLI/Python 三个入口最终调用同一组互锁。
- 采集器使用新文件模式 `x`，拒绝覆盖已有 episode。

