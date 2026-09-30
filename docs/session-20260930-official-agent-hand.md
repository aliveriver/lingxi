# 2026-09-30：PC1 官方 agent 握手控制路径

已只读检查 PC1 官方程序、MC 配置、消息定义和本次握手日志。确认用户此次“握手”通过 **MC 的 SetMcPresetMotion 预置动作服务**执行，MC 使用已有手部发布器输出目标；不需要独立向 HAL 争抢命令。没有在本轮调用运动服务、重放握手、修改配置或重启 MC。

## 本次操作的直接证据

PC1 `/agibot/log/latest/mc/mc.log:4111` 附近记录：

```text
2026-09-30 15:52:13.813643 SetMcPresetMotionCoServiceImpl
input_source: {name: "agent", priority: 0, timeout: 0}
area: {value: 2}
motion: {value: 1003}
interrupt: true
ani_path: ""
play_timestamp: 0
```

15:52:13.814433，`motion_handler.cc:255` 将动作解析为：

```text
cur_motion: 1003
cur_motion_type: ANIMATION
res_path: /agibot/software/mc_param/robot/lx2501_3_t2d5/./classic/animation/SM_shake_right_hand.csv
area: RIGHT_HAND
input_source: agent
```

15:52:13.815371，播放器记录 `Interrupt signal accepted.`。服务启动注册日志明确映射 ROS 名称 `/aimdk_5Fmsgs/srv/SetMcPresetMotion`。这比从话题存在或版本号推断接口可用更直接，但不是本次重新采集的完整编码器响应。

官方 app_proxy 同时存在该服务的客户端注册，以及通用 ROS RPC 转发实现。其 `Agent is playing` 日志在 15:52:13.834318 出现，不过对应代码只是订阅语音/表情状态；**不能把这条状态日志当作 app_proxy 发出了这次运动 RPC 的证据**。已经确认的运动链路从 MC 收到 `input_source=agent` 的请求开始；agent 前端到请求发送进程的完整来源链尚未定位。

## 为什么能张手

`classic/animation_player.yaml` 启用 `enable_hand: true`，将动作 1003 的 area=1/2 映射到左右握手 CSV。已复制实际右侧 CSV，只在本地解析：

- 2721 行、40 列，timeMS 从 0 到 5440，步长 2 ms。这是文件时间轴，不是本轮实测执行时长。
- 全部 20 个手指目标保持常量。左手 `[-0.4, 0.2, -0.4, 0, 0.2, 0.2, 0, 0.2, 0, 0.2]`；右手 `[0.4, -0.2, 0.4, 0, 0.2, 0.2, 0, 0.2, 0, 0.2]`。
- 变化的是右肩、右肘、右腕共 5 个通道；例如右肩 pitch 从 0.4 到最低 −1.0 rad，最后回到 0.4。因此 **RIGHT_HAND/area=2 不是“只动手指”的保证**，不能用这个预设充当小幅单指测试。
- 腰、头、左臂及其他通道也存在于 CSV 中；文件范围本身不能证明播放器在所有模式下的输出隔离。

这条路径使用文件里的既定手姿目标，不要求把当前异常手反馈锁存为目标，因而能在手反馈异常时依然下达张手姿态。它不是在线反馈恢复或重新标定接口；右食指侧摆在后续只读采样中仍约为 −3.1964 rad。

MC 配置 `mc.yaml` 的 `hand_joint_command` 发布器启用，话题 `/aima/hal/joint/hand/command`、频率 50 Hz。此前实际订阅也观察到约 50 Hz 的 MC 双手目标。故 agent 动画使用现有 MC 输出链，不会像旧程序直接发布 HAL 目标那样增加第二路发布者。

## 对后续控制的意义

已找到可复现核对的官方上层入口，不再笼统地说“新版本手部控制方式未找到”。但这个入口首先是**含手部目标的预置上肢动作**，尚未验证任意单手/单指位置控制，更未证明臂独立控制或异常反馈恢复。

请求中的 `agent` 名称和 priority=0 是日志事实，不能推导为任意程序冒用此名称即可安全抢占。MC 全局 `app_proxy` 优先级和预置动作请求来源不是同一条证据；本轮没有充分核清预置动作服务的完整仲裁规则。后续若接入，应沿 MC 支持的动作接口核验范围、抢占/停止语义和编码器响应，而不是把此日志直接改成可执行命令。

原肩部 +0.01 rad 关闭/开启组仍受手反馈阻断，本轮没有新增实体跟随结果。另在读取现有日志时发现 16:08:19 出现 STAND_DEFAULT→LOCOMOTION_DEFAULT 转换；这不是本轮触发，说明旧现场/模式快照不能用作下一次运动的入场状态。

## 证据

目录 `logs/official-agent-hand-20260930/`：

- `confirmed-handshake.txt` / `mc-handshake-excerpt.txt`：原日志摘录与 MC/HAL 启动时间。
- `preset-interface.txt`：现场 srv/msg 定义、MC 手发布器配置及 CSV 远端哈希。
- `motion-config.txt` / `rpc-and-animation.txt`：动作映射、播放器和 app_proxy 转发源码。
- `SM_shake_right_hand.csv` / `animation-analysis.json`：实际动作文件及全列范围分析。
- `service-origin-check.txt` / `agent-callback.txt`：区分客户端注册、语音播放通知和真正的运动请求。

CSV SHA256：`655a40f86724859b8cb9ac031ab9d2317a2bea4828f8a6500050cbed10e0df50`，本地与 PC1 一致。先检查 git diff，保留已有修改；新增内容仅为本地证据和报告。本轮没有实体测试、软件控制变更或新增测试通过数。
