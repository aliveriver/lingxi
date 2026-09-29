# PC1 上肢位置控制核查（2026-09-29）

本轮只读登录 PC1，没有改固件/控制器配置、重启 MC 或发送新动作。文件副本保存在 `logs/pc1-config-audit/`。

## 已确认

运行中的 MC 命令行指向 `/agibot/software/mc_param/robot/lx2501_3_t2d5/mc.yaml`。

- 系统发行版本：`release-lx2501_3_t2d5-soc0-v1.1.4`。
- MC 包：`v1.1.0-r9-rc9`，构建 `e3c738dc`；包版本字符串不等于系统发行版本。
- MC 参数包：`v1.1.0-r9-rc3`，构建 `be6171a9`。
- HAL EtherCAT：`release-v0.3.19`，构建 `620a4515`。
- URS 的下肢 runner 为 `rl_cpgtelecon`，上肢为 `upper_body_external`，两者配置周期均 10 ms。
- `/agibot/software/mc_param/robot/lx2501_3_t2d5/classic/upper_body_external.yaml` 的 14 个臂轴均为 `kp=40, kd=2`。
- `WriteArmsFromCommand` 的只读反汇编显示通过关节名查配置，并将输出速度与前馈 effort 清零；原始同步数据也确认 `velocity=0, effort=0, stiffness=40, damping=2`。
- 没有在该 URS 配置中找到可用的重力/摩擦补偿设置。另一条遥操作路径的配置有 `compense_switch=1, compense_ratio=0.8`，但不能据此认定 URS 使用该配置，也不能盲目复制。

`UpperBodyCommandArray` 没有 stiffness/damping/effort/velocity 字段，因此 Python `ArmCommand` 中这些参数无法调整 URS 控制器。代码现在拒绝非默认增益和非零速度/力矩参数，避免静默忽略；默认 `20/2` 是历史 API 占位值，不代表实际发送了该增益。

## 索引与限位

URS YAML 的腕部配置书写顺序为 yaw/roll/pitch，而 HAL 顺序为 yaw/pitch/roll。使用第五次测试中不相等的腕部目标逐轴比对后，全部 14 个上肢目标与 HAL 按关节名对应值一致，最大差为 0。这是配置条目书写顺序差异，不是已发生的腕部指令交换，项目不改变索引。

`x2_T2.5_softlimit_brake_cfg.yaml` 中左肩节点的刹车区间是靠近软限位的速度保护参数；文件不提供“微小位置命令被抱闸阻挡”的证据。左肩 0.400–0.410 rad 未接近所读位置范围边界。未修改任何限位或制动参数。

HAL 上肢 MIT 编码参数为位置 ±6.283、速度 ±50、effort ±180、kp 0–1024、kd 0–40.96。以 16 位位置量化估计约 `0.000191745 rad/LSB`，与观察到的反馈阶梯一致；`0.01 rad` 约为 52 个阶梯，不能用“低于编码器分辨率”解释实际只移动约 3 个阶梯。

## 现有实测支持什么

固定命令基线测试中，上肢与 HAL 目标一致执行 `0.400 → 0.410 → 0.400 rad`，实际目标位移仅 `+0.000575066 rad`。

按源 ROS stamp 对齐（只用最近 20 ms 内 HAL 样本），在去目标至恢复驻留窗口的 2077 个样本上回归 `reported effort ~ (HAL target - encoder)`：斜率约 `38.86 N·m/rad`，截距约 `-0.1084 N·m`，R² 约 `0.931`。它是未经校准的遥测相关分析，不是力矩标定，也不应直接把截距当作摩擦/重力值。

该结果与 kp=40 的响应相符。`+0.01 rad` 最多带来约 `0.4 N·m` 的额外比例项，小位移下的静摩擦/负载与缺少补偿值得检查；目前仍不足以区分这些因素和驱动内部的控制行为。不能宣称根因已完全解决。

## 下一步调参实验的边界

2026-09-29 后续优先级更新：先审查模型与重力补偿路径，见 [开源仓库审查](upstream-ik-audit.md)。
下述 kp60 补丁继续仅为未应用的历史候选，不是下一次实机试验的默认选择。

可审查的候选是仅将 `left_shoulder_pitch_joint` 的 kp 从 40 临时改为 60，kd=2、其余 13 轴、前馈力矩均不变，仍只测试固定命令基线上的 `+0.01 rad`。这是一项待批准的单变量诊断，不是已确认修复，也不是厂商给出的 URS 推荐值。

候选补丁保存在 `docs/proposals/left-shoulder-kp60.patch`，没有应用到 PC1。应用前需要现场确认安全支撑/姿态和允许改变控制力；增益变化将额外改变维持已有位置误差所需的力矩，并非只影响测试偏移。

控制器在 `Init()` 中读 YAML，尚未确认安全热加载入口。不能假定修改文件立即生效，或把进出 URS 当作可靠重载；若需重启 MC，必须在现场安全承托条件下单独执行，不能在当前自由站立状态直接重启维持下肢平衡的 MC。

获得参数加载和现场条件确认后，必须先只读检查 HAL 实际增益是否为目标值，再考虑运动；失败时撤回候选，恢复原文件并核对实际增益。先用当前证据向厂商确认 URS 的推荐增益/补偿路径也可避免盲目调参。本轮未向任何人发送消息。

原文件 SHA-256：

- upper_body_external.yaml：`f58022e55febb54fdbb6953100811bcfcc756e87918951e1d86b5d97a4cfec4b`
- hal_ethercat_x2.yaml：`e36271f7d2271018b40071e9bb41d2a03ea17c2930bdb742df1496de9468a315`
