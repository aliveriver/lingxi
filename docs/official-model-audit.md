# 官方模型来源、实机副本与重力观测（2026-09-29）

已从官网找到 X2 官方模型仓库和 OmniHand 左右手 URDF 下载，并取得 PC2 安装模型及出厂标定的只读副本。
当前没有接入在线重力补偿。官网模型、安装模型、真实硬件三者需区分，模型文件存在不等于控制验收通过。
本轮只读预检为 `PASSIVE_DEFAULT/RUNNING + FSM wire 6 + body SIT(4)`，控制开关关闭。
没有切模式、移动机器人、修改增益、停止/重启 MC 或写入出厂标定。

## 官方下载与机型选择

官方 [获取 SDK / URDF](https://x2-aimdk.agibot.com/zh-cn/latest/get_sdk/index.html)
明确链接到 [AgibotTech/agibot_x2_urdf](https://github.com/AgibotTech/agibot_x2_urdf)。
本次审查固定提交 **`575cc6b988f976c23550e0db85aa1e5475d3652d`**，提交时间 2026-09-21。
官网亦提供 [X2_URDF.zip](https://x2-aimdk.agibot.com/zh-cn/latest/_downloads/3f0ab2c08bf92924fd76963bbea7a23a/X2_URDF.zip)，
本次实际取得的是固定 Git 提交中的模型文件，不把未经下载的 ZIP 视为相同内容。

| 后颈铭牌第一行结尾 | 官方模型 | 说明 |
| --- | --- | --- |
| X2 Ultra | `X2_URDF-v1.3.0/x2_ultra.urdf` | 旗舰版 |
| X2 Ultra -N | `X2_URDF-v1.4.0/X2-Ultra.urdf` | 旗舰焕新版；电机与结构有变更 |
| X2 EDU | `X2_URDF-v1.4.0/X2-EDU.urdf` | 人人造版 |

`X2-Ultra_omnihand.urdf` 位于 **v1.4.0**，官方明确将它归为旗舰焕新版的灵巧手变体。
不能因为当前机器人装有 OmniHand 就选它。Agi 固件 `v1.1.4` 也不是 URDF 版本选择依据。
本机配置写 X2 Ultra、安装包提供 v1.3.0，但铭牌末尾仍待现场确认；当前没有推断为 -N。

用户提供的 [filepage/291](https://www.agibot.com.cn/filepage/291.html) 当前页面主要提供飞书资料入口，
没有从该页取得 URDF 直链；具体模型依据是上面的官方 SDK 文档与 Git 仓库。

## 文件哈希与对照范围

所有参考数据在 `logs/official-model-audit/`；完整文件哈希见 `manifest.json`。
官网 HTML、下载 ZIP、模型文件、测试和只读日志均保留；这些文件没有打包进 Python 库。

| 文件 | SHA256 |
| --- | --- |
| 官方 Git v1.3.0 `x2_ultra.urdf` | `f628a969aaf3d5785b8c8901611a85f358d7eb62a8b06b1216dc42291a72944e` |
| PC2 SDK v1.3.0 `x2_ultra.urdf` | `f84fcd22187d7f3d9730f36b06b39e49fb6a59cd8b1d26bcb5a817dfd300b40a` |
| PC2 nav `x2_31dof_hand.urdf` | `60399346c15d0855ef6a4756eb88eb74c5d9ba2e973f05a83db8cebb483eb17c` |
| 第三方 IK 仓库 `x2_ultra.urdf` | `1163b3c76b31c4ea0afd284b67b28948003ce46f7ea4b2d9826f1309e9af7f11` |

PC2 SDK 路径：`/agibot/software/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0/`。
导航副本来自 `/agibot/software/nav/share/urdf/x2_31dof_hand.urdf`。
这些文件是该机已安装资源；本轮没有证明 MC 当前加载其中哪份动力学模型。

`compare-arm-models` 对照 14 个臂轴的 child link、关节 origin/旋转/归一化 axis、质量、质心及位置/effort 限位。
第三方 IK ↔ PC2 SDK、PC2 SDK ↔ nav 的上述字段全部数值相同，虽然文件哈希不同。
官方 Git ↔ PC2 SDK 的臂几何、质量/质心及 effort 限位相同，但下列 **8 个位置限位字段不同**：

| 关节字段 | 官方 Git rad | PC2 SDK rad |
| --- | ---: | ---: |
| left_shoulder_roll upper | 3.0456 | 2.993 |
| right_shoulder_roll lower | -3.0456 | -2.993 |
| left_wrist_pitch lower | -0.5236 | -0.558 |
| left_wrist_pitch upper | 0.5236 | 0.558 |
| right_wrist_pitch lower | -0.5236 | -0.558 |
| right_wrist_pitch upper | 0.5236 | 0.558 |
| left_wrist_roll lower | -1.5097 | -1.571 |
| right_wrist_roll upper | 1.5097 | 1.571 |

报告没有比较惯性张量、速度限制、腰/基座/IMU 链、驱动参数、实际零位或固件行为。
不自动合并模型限位，不覆盖当前安全配置。PC2 的 `x2_hand.urdf` 只有每臂五个活动关节，
缺少 wrist_pitch/roll，不能因文件名包含 hand 就替代当前七轴臂模型。

```bash
uv run x2 compare-arm-models \
  logs/official-model-audit/official-files/X2_URDF-v1.3.0/x2_ultra.urdf \
  logs/official-model-audit/pc2-sdk/x2_ultra.urdf
```

## OmniHand 官网模型

[OmniHand O10 官方资料页](https://www.agibot.com.cn/DOCS/OS/Omnihand-O10)
标注 URDF 上传日期 **2026-08-27**：

- [右手 URDF ZIP](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813525152953.zip)：
  SHA256 `f4b48bbc28085370300496eac59a32f3bac497cdb9dda2447156c727c41fef34`，模型 `OmniHandright4.urdf`。
- [左手 URDF ZIP](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813527171066.zip)：
  SHA256 `8764a84d911dd5d46ece309bb3712e71c23fedffd366e353fc737f74f7e7efdb`，模型 `OmniHandleft3.urdf`。
- [2025 灵动款规格书](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260623/1782205918957651.pdf)。

已检查 ZIP 成员，只提取 URDF 和元数据，没有运行包内脚本。
左右均有 18 links、17 joints：10 个独立可动关节、6 个 mimic 关节、1 个锁定中指 abad 关节。
左侧锁定关节名原文为 `l_middle_abad_jonit`，右侧前缀为大写 `R_`、左侧为小写 `l_`。
左右若干 mimic 符号不同，应保留原始定义，不用简单字符串大小写或镜像取反生成控制映射。
当前 HAL 已确认的主动关节顺序不因这些模型名称而改变。

模型惯性质量合计均为 **0.5343 kg**。规格书写明重量（**不含手腕板护盖及螺钉**）：
普通灵动款 ≤510 g、灵动触觉款 ≤520 g。二者的部件范围、具体手型/版本、实际安装转接件尚待核对，
不能把模型合计值当作本机称量值。
独立模型根为 `l_palm` / `R_palm`，不含 X2 wrist_roll_link 到掌根的安装变换。
质心随手指姿态改变，掌心/TCP 坐标也不等于全手质心。

另需注意：七段臂模型没有独立 hand link，**并不证明 distal link 的惯性已排除所有末端零件**。
v1.3.0 `left_wrist_roll_link` 质量 0.30304 kg、COM z=-0.083026 m；右侧 0.303847 kg、COM z=-0.082799 m。
尚未确认这些聚合参数具体包含哪些部件。不能机械地在它们之上再加 0.5343 kg，以免重复计重。
前轮“裸臂/零 payload”算例的准确含义是“按原文件惯性、没有额外添加 payload”，不是已确认的物理裸臂。

## IMU 官方语义与实机采集

官方 [坐标系与出厂标定](https://x2-aimdk.agibot.com/zh-cn/latest/about_agibot_X2/coordinate_system.html)
说明模型是标称设计值；每台机器的传感器标定在 PC2：
`/agibot/data/param/protected/factory_calib/factory_calibration_params.yml`。
已只读复制到 `pc2-factory-calibration.yml`，原文件 SHA256
`ce405931791f04ffdc14b6ac511e43e74912f0a86370ab327d20070d6c8dc0ef`。
胸部/腰部段都存在，标定时间为 2026-05-13。未修改原文件，未对外发送。

官方 [传感器接口](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/hal/sensor.html) 还明确：

- `/aima/hal/imu/chest/state` 是胸部，`/aima/hal/imu/torso/state` 是胯部；都在 PC1，标称 500 Hz。
- 两者 `frame_id` 都是 `base_link`，不能仅靠 frame_id 区分安装体。
- covariance 全零表示未提供，不是无噪声；`header.stamp` 是机器人接收时间，不是传感器采集时间。
- QoS 为 BEST_EFFORT/TRANSIENT_LOCAL；本只读采集器使用 BEST_EFFORT/VOLATILE 订阅新帧，可匹配该发布者，
  不使用历史缓存帧冒充实时帧。

已运行两次只读采集。最终 3 秒采集得到 chest/pelvis/waist/arm 回调数 **822/821/839/835**，
每路 1 个发布者，错误数 0；回调数含发现和调度影响，不据此认定稳定 500 Hz。
四路原始采样及收发时间保存在 `pc2-gravity-final.json`；DDS 日志单独保存。
独立稍后 preflight 确认为坐姿 PASSIVE_DEFAULT，fault 均为 0、无竞争上肢发布者。
这个模式记录不是与每个 IMU 帧同步的模式证据。

当前样本揭示了一个阻断条件：`waist_pitch_joint ≈ +0.346088 rad`，
而 PC2 SDK/官方 v1.3.0 腰 pitch 模型范围都是 `[-0.314, +0.314] rad`。
新校验工具因此**拒绝胯部 IMU + 腰链估算**，不裁剪腰角、不扩大限位。
这是反馈与模型范围不一致，不是本轮确认的驱动超限故障或根因；需要进一步核对坐姿反馈语义、模型与零位。

对胸部 IMU，使用“安装旋转为单位阵”的显式假设，算法可输出 torso 重力约
`[-2.51505, +1.03196, -9.42580] m/s²`。该结果只验证数据格式和计算链，**不代表安装假设成立**。
出厂字段名/文档的 `R_baselink_sensor` 方向，以及 HAL 是否已经应用该外参，尚未核清，
本轮没有把该矩阵或其转置自动应用。不能因为矩阵存在就重复补偿安装旋转。

## 新工具及验证

- `compare-arm-models`：比较两份支持的七段臂 URDF；字段相同也始终 `hardware_validated=false`。
- `scripts/read_gravity_state.py`：独立 ROS 节点，仅创建 IMU/腰/臂订阅；无运动发布器、模式服务和 X2Client。
  使用单调时间限时、每路只保留最新一帧、计数及最多 20 个错误，避免无界采集。
  重复/倒退时间戳或解析失败会使记录无效，不沿用旧帧隐藏异常。
- `analyze-gravity-state`：离线检查完整关节、故障、有限数、四元数范数、SO(3) 安装矩阵、
  三轴腰链和范围；接收/源时间年龄 ≤200 ms，跨流偏差 ≤50 ms。两个时钟域分别计算。
  不自动退回静态重力或加速度计，不把 timestamp 一致当作传感器真实同步。

只读采集（PC2 已具备 ROS/AimDK 环境）：

```bash
uv run python scripts/read_gravity_state.py --duration 3 --output logs/gravity-snapshot.json
```

`--output` 拒绝覆盖旧文件，并将 JSON 与厂家 DDS stdout 分开。
离线 mounting 文件需显式声明 `imu_key`、`body`、`expected_frame_id`、`rotation_body_from_imu`；
后者定义为**将 IMU 系向量转换到所声明 body 系**的 3×3 旋转矩阵。
`imu_key` 为 `chest_imu/pelvis_imu`，body 为 `torso_link/pelvis`，不能从 frame_id 猜测。

```bash
uv run x2 analyze-gravity-state logs/gravity-snapshot.json \
  --mounting /path/to/reviewed-mounting.json --urdf /path/to/reviewed-x2.urdf
```

失败退出码 2，报告无重力向量；成功只表示录制时刻的离线数据可计算，不准许运动。
`logs/official-model-audit/*-identity-mount-hypothesis.json` 仅是本次假设检查输入，不能作为已确认标定配置。

本地和 PC2 全量测试均 **112 passed**（含此前环境阻塞的 Web 两项，现在通过）。
新增工具已经部署 PC2；磁盘两份实机配置仍为 `control.enabled: false`。
下一步先核实铭牌、末端惯性组成/安装变换，以及 IMU 外参应用位置和腰角范围差异；
这些条件明确后再制定单轴跟随/回位验收，保持原有站立检查和现场新确认要求。
