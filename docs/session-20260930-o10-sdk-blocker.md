# 2026-09-30：O10 SDK 与当前臂运动阻断复核

本轮取得新的只读状态及本机 HAL 转换证据，**仍未恢复可信手反馈，没有新增实体轨迹或编码器跟随验收**。用户已确认自由站立、有人监管、可立即急停；在反馈阻断仍存在、暂无厂商步骤时，用户要求先整理证据及待核实问题。此前所有未提交软件修改保留，本轮未修改控制代码或部署候选。

## 新鲜实机证据

PC2 约 15:33 的 5 秒订阅采集保存在 `logs/following-resume-20260930-1533/readonly.json`。末 3 秒 MC 为 STAND_DEFAULT/RUNNING，FSM 4/body 1，臂、腰、手无非零故障；臂及腰位置跨度为 0，手最大跨度 0.000927734375 rad。胸部 IMU 模长 9.795756–9.835236、骨盆 9.797419–9.836149 m/s²。未创建运动发布者，独立 upper 记录为 0。静止和故障零不能证明绝对手位置正确。

| 手轴 | 本轮 HAL/客户端位置 rad | 按已安装磁盘 HAL 转换逆算的输入整数 |
|---|---:|---:|
| 左食指 PIP | −23.549616699218753 | 65535 = 0xFFFF |
| 右食指侧摆 | −3.1963867187500004 | 65462 = 0xFFB6 |

输入整数是**离线推算，不是总线抓包**。同一转换还能逐项重现本轮末帧全部 20 个手位置，浮点残差为 0。左食指侧摆对应 4189，也超过 SDK 名义归一化范围；不能因为其角度在 ±π 内就认定已标定正确。

PC1 `hal_ethercat_app_main` 磁盘 SHA256：`375aa24fddee90f0bae2dcd12c1cf6116945da25bd6be68408f22aa6b1955f10`；`.version` 为 `hal_ethercat 2328 release-v0.3.19 620a4515`。本轮未另行执行、替换或注入该程序；只读提取了符号、指令与常量。`/proc/2361/exe` 哈希读取被拒绝，因此不能声称核验了运行中映像与磁盘映像相同。

证据链：

- 本地 `_joint()` 直接使用 `float(item.position)`，未换算单位、改符号或归一化。
- 磁盘 HAL `ProcessHandState` → `ParsePayload` → `OmnihandCtrl::ActuatorInput2ActiveJointPos`。`ParsePayload` 在 `0x12c0c4–0x12c124` 将前 20 字节按 10 个小端无符号 16 位数拼装，再进入位置转换。
- 转换函数在 `0x15e7f0` 调用 `CheckActuatorInput`，随后没有使用返回值来拒绝输入；`0x15e878–0x15e8bc` 作线性外推。构造函数及只读常量给出相应端点。对两处主要异常可简化为：左 PIP `1.57*(4096−65535)/4096`；右侧摆 `−0.2*65462/4096`。
- 保存了 `process-hand-state.asm`、`parse-payload.asm`、`actuator-to-joint.asm`、`omnihand-ctor.asm`、`hand-rodata.txt`。复算：`python3 logs/following-resume-20260930-1533/analyze_hand_conversion.py`。

这证明存在与异常完全吻合的转换路径，未证明输入异常由硬件越界、标定、通信、哨兵值或固件行为中的哪一种造成。没有把 65535 改为 −1、把 65462 改为 −74，更没有把推算值用作控制目标。

## 官方 SDK 提供的新线索

2026-09-30 查询仓库 HEAD 为 `c23801f03e396d8d667a307548a05b5d0aecf9aa`，`linux/x64/VERSION` 为 **1.1.9**。这是 SDK 版本，不是机器人 Agi v1.1.4 固件版本。保存的 17 个文件及固定 URL/哈希见证据目录 `sdk/sources.json`；只阅读文本，没有安装 SDK、运行示例或连接手总线。

### 1. 普通协议和私有协议不能混用类型

[普通位置 API](https://github.com/AgibotTech/agillink_omnihand_sdk/blob/c23801f03e396d8d667a307548a05b5d0aecf9aa/linux/x64/cpp/include/omnihand/omnihand.h#L178) 返回 `int16_t`，文档名义范围 0–4096；基类单轴查询默认返回 −1，设置接口也将 −1 描述为失败/无应答。**这不是当前私有回包中 0xFFFF 的定义。**

[私有 `SetAllAxisPosResponse`](https://github.com/AgibotTech/agillink_omnihand_sdk/blob/c23801f03e396d8d667a307548a05b5d0aecf9aa/linux/x64/cpp/include/omnihand/private_omnihand.h#L80) 明确使用 `vector<uint16_t> positions`，前 20 字节是位置，之后为速度、电流、逐轴错误。这与本机 HAL `ParsePayload` 的字段偏移吻合。不能仅凭普通 API 的 `int16_t` 将当前问题确诊为 HAL 符号错误。

未在本次审阅的 O10 文档、头文件及示例中找到对 0xFFFF/0xFFB6 的适用固件含义、有效性规则或在线恢复步骤。需要厂商提供当前回包来源和原始字节；同为 60 字节的其他回包不可直接套同一布局。

### 2. 有可以明确向厂商索取的查询字段

[PrivateOmniHand](https://github.com/AgibotTech/agillink_omnihand_sdk/blob/c23801f03e396d8d667a307548a05b5d0aecf9aa/linux/x64/cpp/include/omnihand/private_omnihand.h) 继承 [IOmniHandCalibrator](https://github.com/AgibotTech/agillink_omnihand_sdk/blob/c23801f03e396d8d667a307548a05b5d0aecf9aa/linux/x64/cpp/include/omnihand/i_omnihand_calibrator.h)，O10 又继承 PrivateOmniHand，因此 O10 确有这组接口声明。

| 查询 | 用途与单位边界 |
|---|---|
| `GetAllAxisPos()` | 私有归一化位置，文档 0–4096 |
| `GetAllActualAxisPos()` | 另一组实际/标定位置，不能与归一化值混为一谈 |
| `GetAxisLimitPos()` | 已保存的 min/max 标定边界 |
| `GetAllAxisPosRange()` | 角度行程，文档单位 0.1°，不是上一项的原始计数 |
| `GetPowerState()`、`GetControlSource()` | 手使能/标定模式及机器人/HMI控制源 |
| `GetFwVersion()` | 手本体型号及固件/硬件版本 |
| `GetErrorCode()`、`GetAllAxisCurrent()` | 整手错误、电流（后者文档 mA）；不能直接等同 HAL 单轴 faultcode 或机械臂 effort |

[O10 头文件](https://github.com/AgibotTech/agillink_omnihand_sdk/blob/c23801f03e396d8d667a307548a05b5d0aecf9aa/linux/x64/cpp/include/omnihand/omnihand_2025.h#L232) 还区分归一化 0–4096 与实际位置：普通轴 0–4095，侧摆轴（1-based 4/7/9）0–1023。此差异使“读取位置属于哪个域”成为必要核查项，不能把 1023 直接写成本机 HAL 的限位。

这些 API 是直连设备能力，不是已经在 X2 上确认可与 HAL 并存的入口。`demo_monitor_error.py` 也会通过 factory 创建 CAN/串口连接；不能因名字叫 monitor 就当作被动订阅。需要厂商提供既有 HAL 的只读导出或明确总线交接方法。

### 3. 清错、归零、Reset 含义不同

- `ClearError()` 仅声明清设备错误；没有保证恢复绝对位置或标定。
- `SetAxisHoming`、`SetAxisMinPos/MaxPos`、`ClearAllLimitPos`、`SaveParam` 会改参考、限位或持久化参数，不是本轮可直接尝试的恢复流程。
- O10 protected `Reset()` 只是设置 SDK 对象字段并建立求解器；不是重启设备或编码器复位。整数手势中的 RESET 则是动作目标，也不是只读恢复。
- `demo_set_max_min_calibration.py` 针对 `OmniHandDexUmi`，不能把该示例当作 X2 上 O10 的恢复步骤。O10 虽有自身标定接口声明，仍缺本机适用流程。

### 4. SDK 不能给出臂独立控制或机械臂 effort 定义

SDK 控制的是手本体。O10 手电流/保护阈值及控制模式不适用于 X2 肩关节驱动。PC2 安装的 `UpperBodyCommandArray.msg`、官方 upper 示例仍只列 HAND 1/2/3，没有验证过的忽略手入口。参考 IK 仓库也不能提供本机臂独立授权或证明。

## 验证与下一步

本轮离线检查用新鲜手位置调用原 v11 `validate_preparation_hands`，仍拒绝左食指 PIP。没有重跑实体或 shadow 会话；已有 `physical-off-v10.json` 用 `evaluate_file` 复算仍为 `inconclusive`，当次真实轨迹与独立 upper 均 0。缺少合法 off/on 对照，未调用 `compare_files` 制造配对结果。

前轮准备证据清单 80 个文件、原重力运动证据清单 25 个文件全部哈希匹配；本次采集在 PC2/本地哈希一致。两份控制配置哈希不变；MC PID 3199、HAL PID 2361 启动时间不变，未改增益、未重启服务。只读分析不需要重跑全量软件测试；历史 523/145 项通过不算本轮新测试或硬件通过。

厂商问题草稿位于 `logs/following-resume-20260930-1533/vendor-support-request.md`，**未发送**。先取得当前 O10 反馈字节/位置域/恢复或官方臂独立接口的确认，解除阻断后才继续固定 HAL 基线的关闭/开启组；肩部低跟随、限流、静态负载、摩擦和 reported effort 仍待验证。
