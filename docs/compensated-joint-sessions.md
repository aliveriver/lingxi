# 平滑关节轨迹与有界重力位置补偿

已实现离线规划、可核验回放和 mock 执行。参考上游公开 MC/URS 位置控制思路，独立实现标准五次插值和 `q_command = q_desired + alpha * clip(tau_g(q_desired) / K, ±bias_limit)`。源码见 `src/lingxi_x2/compensation.py`、`compensation_runner.py`；没有复制上游 Python 代码或打包其模型。

`K` 是显式假定的等效刚度。示例使用 40，与审计到的 MC kp 数值一致，但不是已辨识的传动/摩擦模型。计算这个偏置不需要力矩消息字段，也不修改 MC 增益。当前 API **仅允许实际 MockBackend 执行**；ROS 后端即使配置为可控制也会被拒绝。该模块尚未接入实时 IMU、实机命令或 Web 运动按钮。

另有独立、固定上限的专用实机诊断脚本，已在逐次现场确认后完成[首次左肩0.002rad部分补偿试验](gravity-live-first-trial.md)。它逐帧验证实时IMU/腰/臂数据，不能从本页离线计划直接执行，也不解除本页通用API的mock限制。

## 直接运行

在项目根目录执行，模型采用之前从 PC2 只读复制的 URDF。输入中的姿态、重力方向、负载均为声明的离线假设，不是新的现场观测。

```bash
# 输出完整逐帧计划；不读取 ROS、不建立机器人连接。
uv run x2 plan-compensation \
  --urdf logs/official-model-audit/pc2-sdk/x2_ultra.urdf \
  --input examples/compensated-shoulder-mock.json \
  --output logs/compensated-shoulder-plan.json

# 核对 URDF 哈希、重新计算每帧，再向 stdout 回放；没有执行动作。
uv run x2 replay-compensation logs/compensated-shoulder-plan.json --speed 1

# 独立模拟器按声明的 baseline 初始化，整段只打开一次命令流。
uv run x2 --config config/mock.yaml mock-compensation \
  --urdf logs/official-model-audit/pc2-sdk/x2_ultra.urdf \
  --input examples/compensated-shoulder-mock.json \
  --output logs/compensated-shoulder-mock.jsonl
```

输出文件必须不存在，避免覆盖证据。回放省略 `--speed` 时立即输出；搬运到另一台电脑后，可用 `--urdf 新路径` 指定相同哈希的模型。已有客户端可调用 `client.run_mock_compensated_session(urdf, request, journal=...)`；它要求模拟器当前姿态与固定基线一致，不会自动重设姿态。CLI 的初始姿态只用于构造独立模拟器，不发送定位命令。

回放对模型哈希、输入及结构精确核验，对重新计算的浮点结果允许最多 `1e-12` 的绝对舍入误差，以兼容不同 CPU/NumPy 实现；超过容差则拒绝。回放保留保存文件里的原始数值，仅向 stdout 输出，不将它们发送到机器人。

将输入文件的 `enabled` 改为 `false` 可比较同一条期望轨迹的无补偿结果，其他阶段和时间保持一致。`motion_joints` 指定允许运动的轴，`compensation_joints` 单独指定允许补偿的轴；所有未指定轴保留固定目标。示例只选择左肩 pitch，期望增量 +0.01 rad，偏置上限 0.01 rad，总命令偏离基线不得超过 0.02 rad。

## 轨迹与边界

依次执行：基线驻留 → 补偿渐入 → 平滑移动 → 目标驻留 → 返回固定期望基线 → 补偿渐出 → 原始命令基线驻留。移动和渐入/渐出使用 `10s³−15s⁴+6s⁵`，端点速度和加速度为零。每段时长向上取整为发布周期，第一帧与最后一帧都精确使用同一基线。额外保留最后一帧一个周期。

偏置来自期望姿态的静态重力计算，不使用编码器跟踪误差，不会积分积累。幅度裁剪会逐帧记录；偏置变化率、最终命令变化率、相对固定基线的总偏移、URDF 与项目关节限位任一超限，整条计划在发送前拒绝，不自动扩大限额或截断最终关节目标。当前诊断工具硬限期望增量 ≤0.02 rad、偏置 ≤0.02 rad、总偏移 ≤0.04 rad；这些是软件边界，**不是厂商认可的实机安全值**。

重力、负载、刚度、轴掩码及限额都必须显式提供。静态模型忽略摩擦、动态惯性、接触、平衡响应；零附加载荷表示不额外加入手/工具质量，并不证明模型已经包含完整灵巧手。不要重复加入末端惯性。

## 记录与验收含义

计划记录模型 SHA256、完整输入和所有帧：`desired_rad`、`gravity_torque_nm`、`unbounded_bias_rad`、`applied_bias_rad`、`saturated_joints`、`command_rad`、阶段及相对时间。重力来源明确标为 `declared_fixed_offline_scenario_not_live_feedback`，不会把旧 IMU 快照称为实时反馈。

mock JSONL 另记每帧发布前反馈及其接收单调时间、调用时间、发布统计、最终反馈。每帧检查反馈完整性、有效性与 ≤200 ms 新鲜度；失效时保留已发送记录和中止原因，不突跳回基线。这里的 mock 故障码允许 None，与实机故障检查不是同一套保护。模拟器直接跟随发送位置，不模拟真实重力、摩擦或平衡，因此 `mock_returned_to_baseline=true` 只验证软件流程；不证明重力补偿提高了实机精度。`hardware_validated` 始终为 false。

未来实机版本还需要：验证机型/URDF、关节零位、手部惯性组成、IMU 安装外参和方向；接入同步新鲜的重力/腰部反馈并拒绝过期数据；取得稳定 HAL 命令基线和固定手目标；复用模式/故障/图监控保护；按现场新确认逐次完成单轴跟随、URS 内回位和重复性验收。不能把离线计划直接喂给 ROS，也不提供跳过该门槛的开关。已失败的实机位置验收结论保持不变。
