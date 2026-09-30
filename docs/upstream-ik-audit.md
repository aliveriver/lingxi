# X2 IK 仓库审查与离线重力诊断（2026-09-29）

结论：可参考其模型内 IK 和重力计算，不能直接把公开 MoveJ/MDI 接到本机。
公开入口仍经 MC/URS，所谓补偿是模型力矩折算的位置偏置；内部另有直发 HAL 的力矩路径。
本轮没有连接 PC1/PC2、切模式、发布实机命令、修改增益或 MC 配置。
当前固定 HAL 基线的单轴跟随/回位仍未通过，不能进入实机 VLA/WAM 闭环。

后续同日已完成官网和 PC2 模型只读核对、IMU/腰角采集，见 [官方模型审查](official-model-audit.md)。
本页保留首轮审查记录。后续确认官方 Git 与 PC2 SDK 有位置限位差异；七段臂模型的 distal link
惯性是否含末端零件仍未知，因此本页“裸臂”算例应理解为“按原文件惯性、不额外加 payload”。

## 审查版本与证据

- 仓库：[maine-cat/Agibot-X2-IK-upper-body-control](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control)。
- 固定提交：`2f6302ebf6f1b8a2c82c98b5c11ba248858944ea`（2026-09-28，模块 2.1.1）。
- 本地只读参考副本：`logs/reference-x2-ik-2f6302e/`，未安装 wheel、运行 AppImage 或在线入口。
- 模型 `source/x2ik/x2_ultra.urdf` SHA256：`1163b3c76b31c4ea0afd284b67b28948003ce46f7ea4b2d9826f1309e9af7f11`。
- 模型 `source/x2ik/x2_ultra.xml` SHA256：`2b755b7affecb4e3df9a0379b2e0ab4a1b2ccda53dd3531ad324dfe5c77cd904`。

下面链接全部锁定该提交；上游日后更新不自动改变本次结论。

## 控制路径与补偿

公开 `Robot.moveJ` → `X2Arm` → `_MotionClient` → `X2ArmClient._send_upper` →
`/mc/upper_body_command` → MC。MDI 同样使用 upper_body；公开入口要求已处于 URS，
不代替用户切换模式。参见 [MoveJ](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_movej.py#L16)、
[发送函数](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_sim_ros.py#L427)。

`ArmDynamics` 使用 URDF 质量、质心和串联关节变换，按虚功计算静态重力补偿力矩：

```text
U(q) = -Σ m_j gᵀ c_j(q)
τ_g,i = ∂U/∂q_i = -Σ下游 m_j gᵀ[a_i × (c_j - o_i)]
bias = clip(τ_g(q_desired) / k_effective, ±bias_limit)
q_sent = clamp_to_URDF_limits(q_desired + bias)
```

公开接口 [fixed_compensation](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_compensation.py)
固定为 `40 N·m/rad / 12° / pelvis`，忽略 SN 标定文件；不代表各机器的等效刚度已辨识。
12° 为 0.20944 rad，约为本项目当前 0.02 rad 试验上限的 10.47 倍。
这不是误差积分：每帧从当前期望姿态重新计算有限偏置，不累加编码器误差。
但第一次发送、姿态估计变化和退出补偿都会改变目标，且两臂同时计算；
只保持另侧的原始目标，并不保证另侧最终发送目标恒定。
上游先裁剪目标，再加偏置、再裁剪，限位处会改变补偿效果。
本项目现已独立实现 [有界位置偏置与五次轨迹](compensated-joint-sessions.md)，用于离线规划、回放和 mock 执行。
尚未实现实机在线位置偏置，不使用跟踪误差不断推高位置目标。

内部 `_send_joint` 则向 `/aima/hal/joint/arm/command` 发送 `JointCommandArray`，包含
`position/velocity/effort/stiffness/damping`；effort 为缩放并限幅的模型重力力矩。
这是绕过上肢 MC 的另一条接口。公开 API 拒绝该模式，但源码保留该路径和历史模式管理代码。
不能据此推断当前固件支持安全的力矩透传、MC 仲裁或命令超时恢复；本项目不接入它。

## 与当前 v1.1.4 的差异

| 项目 | 上游源码 | 当前平台要求 / 判断 |
| --- | --- | --- |
| 接口 | 公开入口 URS 位置；内部 HAL 力矩 | 保留 URS；已确认 MC 将速度与 effort 清零 |
| MC 增益 | 补偿假定等效刚度 40 | 实际 URS kp40/kd2；不等于含摩擦与传动后的已辨识模型 |
| 节拍/QoS | upper_body 50 Hz，RELIABLE；`goto_joint/_pace` 使用 wall clock | 已验证 50 Hz BEST_EFFORT/VOLATILE、monotonic、禁止突发补帧；没有重改 QoS 的证据 |
| header/轴序 | mc_upper_body、递增 sequence，左7右7，腕 yaw/pitch/roll | 与当前已验证映射一致；不交换腕轴 |
| 双手 | 默认 hand_mode=1；mode2 时给 20 个零 | 零目标不是保持手姿；当前 OmniHand 必须保留固定手部基线 |
| 初始基线 | 原始保持值初次来自编码器；MDI rebase 也重取反馈 | 必须固定经验证的 HAL 命令基线，避免已知基线漂移 |
| 状态检查 | 检查 URS action；公开 API 有关节新鲜度、完整性、有限数、图独占与发送间隔检查 | 还需持续核对 RUNNING、FSM wire4、body STAND、播放器、fault、手部反馈和安全缓存有效期 |
| 图检查 | 发布者数量=1、订阅者≥1 | 不足以识别真实 MC 订阅者；本项目继续排除自身回读订阅 |
| 模型限制 | URDF 限位；动力学取 min(URDF effort, MJCF ctrlrange) | 这些是模型元数据，不是 v1.1.4 驱动获准参数 |
| 运动限制 | IK 单帧 0.05 rad、有限局部迭代、连续五帧失败停止推进 | 不是时间相关的速度/加速度限制，不覆盖碰撞与整机平衡 |
| 退出 | 停止发送；桌面退出无自动 HOME/模式恢复，终端普通 q 可双臂 HOME | 停发不是急停；不能照搬退出动作或假定异常已安全恢复 |

源码未提供与当前项目同等的 fault、站立 wire 状态、手部与缓存安全检查。
这是所审查入口的范围差异，不是声称 MC 自身没有保护。
公开入口保留 MC 下肢控制，并不能证明叠加的全臂运动仍满足平衡和碰撞条件。

## 重力坐标系、负载和模型

[模型文件说明](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_arm_model.py#L12)
标注 **X2_URDF-v1.3.0**，不是 Agi v1.1.4 固件版本。
其注释称与旧内部 T2.5 的臂几何相同、肘限位和质量不同；当前机器的实际模型尚未完成哈希和参数对照。
左右臂几何并非严格镜像，必须分别计算。模型有七个臂 link 的惯性参数，没有手部 link。
裸臂质量左 3.940932 kg、右 3.930909 kg；这不是称量结果。

[GravityEstimator](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_frames.py#L310)
根据模型挂点把 `/imu/chest/state` 视为 torso IMU、`/imu/torso/state` 视为 pelvis IMU。
公开入口选择 pelvis，计算 `R_world_torso = R_world_pelvis * R_pelvis_torso(q_waist)`，
再用 `g_torso = R_world_torsoᵀ * [0,0,-9.81]`。
不能只凭 Topic 名称认定实机挂点、安装旋转或四元数方向一致。

当前源码检查 IMU 曾接收和 pelvis 是否有姿态，但没有接收时间过期门限；
腰角初始化为零、缺轴默认零，未强制完整新鲜腰反馈。
`refresh_gravity()` 可对同一缓存重复更新滤波，更新次数不等于新消息数量。
因此仅“有 IMU”不足以批准补偿。应先只读验证挂点、安装方向、四元数有效性、
腰角完整性、单调接收时间和源时间戳对齐，再计算倾角；缺失或过期应报告不可用。

上游 `ArmDynamics` 的 payload COM 从 TCP 原点起算并在 wrist 坐标表达，
公开 MoveJ 默认 payload=0；手/夹爪 TCP 预置只改变位姿解释，不补充实际质量。
本项目新工具统一要求 COM **相对 wrist_roll_link 原点、在该 link 坐标表达**，避免混淆。
手质量、工具、负载质心须分别核实；0 明确表示排除这些质量。

## IK 可复用范围

[求解器](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_srs_ik.py)
在 torso_link 系求解 7 轴 SRS，利用 SEW 臂角和解析分支搜索，再对非理想肩腕几何做数值精修。
`solve` 优先保留已满足目标的姿态，允许局部求解及全局备用种子；
运动 `track` 使用局部跟踪、分支检查、完整 FK 残差和限位验证，拒绝超过 0.05 rad 的单步，
不使用全局恢复。TCP 平移和旋转均参与变换。
这些适合后续独立离线 IK 评估，但模型内 FK 精度不是实体 TCP 精度，且不含碰撞规划。

本轮复现上游自带 18 项测试全部通过，记录在 `logs/20260929-upstream-ik-tests.txt`。
测试是 NumPy 求解与内存关节反馈，没有运行 MuJoCo/PhysX，也没有机器人连接。
没有将上游 Python 文件或模型复制进本项目安装包；其第三方说明只明确模型许可，
未授予项目自身代码新的许可证，后续若要分发代码需核清许可。

## 本项目新增离线工具

`x2 gravity-report` 仅解析指定 URDF 数据，不导入上游 Python 或初始化 X2Client/ROS。
要求完整命名的 14 轴姿态、显式 torso 重力、两侧 payload 和假定刚度。
只支持从 torso_link 开始的七轴串联裸臂；额外手部/固定工具分支会被拒绝，防止漏算或重复计入质量。
输出模型 SHA256、原始重力力矩、势能、模型限位裕量及 `τ/k` 诊断量；
没有 q_sent、自动限幅或发布功能，`hardware_validated=false`。

```bash
uv run x2 gravity-report \
  --urdf logs/reference-x2-ik-2f6302e/source/x2ik/x2_ultra.urdf \
  --input examples/offline-gravity-home.json
```

示例只是水平 torso、裸臂、HOME `[0.4,0,0,-1.2,0,0,0]` 的假设场景，**不是实测姿态/负载**。
在受限环境可使用 `UV_CACHE_DIR=/tmp/lingxi-uv-cache uv run --offline ...`。
参考副本位于被 git 忽略的 logs；新检出需单独取得固定提交的 URDF，不会自动下载或选模型。

独立实现与上游 `ArmDynamics` 在两臂共 200 个固定随机种子的姿态/重力/负载场景下交叉比较，
最大差 `3.55e-15 N·m`。另用合成单摆解析解验证符号、负载力臂，用势能中心差分验证一般轴和关节变换。
二者模型一致只验证计算；不能验证模型描述的机器是否正确。

| 假设场景 | 左肩 pitch q=0.400 的 τ_g | q=0.420 的 τ_g | q=0.420 的 τ_g/40 |
| --- | --- | --- | --- |
| 裸臂、水平 torso | +0.007644 N·m | +0.139600 N·m | +0.003490 rad |
| 假设腕系 z=+0.05 m 处增加 0.5 kg | -0.159731 N·m | +0.001962 N·m | +0.000049 rad |

0.5 kg 是敏感性假设，不是 OmniHand 的测量质量。HOME 裸臂其他轴也有非零补偿：
左肩 roll 的 τ/k≈+0.03357 rad、左肘≈-0.04102 rad，已超过当前单轴试验幅度。
所以不能把全臂补偿当成“只改善左肩一点点”的无影响改动。
左肩结果对姿态和负载敏感，现有数据不能证明重力是跟随不足的唯一根因。
本报告不将 τ_g 与 reported effort 直接等同，也不把 τ/k 解释成实测所需偏置。

结果文件：`logs/20260929-offline-gravity-home.json`、`logs/20260929-gravity-crosscheck.json`。

## 下一阶段验收条件

1. 先只读收集本机 URDF/MJCF、安装手型的质量/质心、关节零位与轴向、IMU/腰角带时间戳样本；
   对照模型哈希、限位和实际 MC 参数。未知项保持未知，禁止套用上游固定 profile。
2. 向厂商确认在不停止平衡 MC 的前提下是否有受支持的 URS 重力前馈接口、参数加载、
   仲裁与超时恢复方式。当前消息不能透传 effort；不直发 HAL，不修改/重启 MC。
3. 若另行考虑模型位置偏置，需先审查每轴最大额外目标、引入/撤销斜坡、补偿变化率、
   饱和处理、双手/另臂固定基线和反馈失效行为；不得积分累积误差。没有这些条件不接入在线路径。
4. 获得可审查实现和现场新确认后，仍先做固定 HAL 基线的单轴小步跟随、URS 内回位、重复性。
   同步采集原始目标、最终目标、HAL 命令、编码器、effort、模式；以驻留窗口编码器验收。
   失败停止，不放大到 0.2 rad，不用切回站立后的反馈替代 URS 回位数据。
5. 单轴通过后才逐步多轴、灵巧手接触与压力响应，最后实机 VLA/WAM。
   目前继续允许离线计算、mock、只读采集和回放。


## 后续同日：匹配对照与发送目标限位补强

再次只读检查本地参考副本，HEAD仍为 `2f6302ebf6f1b8a2c82c98b5c11ba248858944ea`，副本工作区干净；没有拉取新的上游版本或运行机器人入口。

- [`fixed_compensation`](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_compensation.py#L8) 明确是固定运行参数而非逐机标定：40、12°、pelvis。本机不采用12°上限。
- [`_send_upper`](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_sim_ros.py#L427) 明确提到零偏置作为纯位置基线；该思路用于本地650帧off/on匹配对照。上游同处对期望及最终目标执行clamp，本地选择在切模式前整段拒绝越界，避免裁剪改变实验条件。
- [`goto_joint`](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_sim_ros.py#L778) 从编码器构造起点、采用五次插值并比较期望误差；本地保留五次曲线及期望/发送目标分离，但继续固定HAL命令基线，不复用逐段编码器起点。
- [`refresh_gravity`](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control/blob/2f6302ebf6f1b8a2c82c98b5c11ba248858944ea/source/x2ik/x2_sim_ros.py#L339) 使用最新缓存更新估计；本地继续逐帧检查源/接收时钟、腰角完整性、六假设与站立故障，不因off组未施加偏置而跳过保护。

本轮独立实现匹配对照及离线原始trace分析，没有复制上游代码、套用其实机默认参数、修改增益或启用HAL直发。新增发送目标URDF限位整段检查，修复专用脚本此前仅验证期望姿态模型限位的缺口。操作和判定见 [重力匹配对照](gravity-comparison.md)。本地全量330项通过；PC2未连接、未部署、未复测，完整实机目标仍未完成。
