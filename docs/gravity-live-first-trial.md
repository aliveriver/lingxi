# 首次实机有界重力位置偏置：方法、结果与局限

2026-09-29 已完成一次左肩 pitch **bias-only 通路诊断**：固定期望位置，渐入、保持并撤销 `+0.002 rad` 位置偏置。350 帧目标经 MC 完整传到 HAL，结束后恢复稳定站立。编码器仅变化 `+0.000191689 rad`，约一个量化阶梯，**不足以证明补偿改善位置跟随**。机械臂位置跟随、回位和重复性尚未验收，灵巧手接触/压力与 VLA/WAM 实机闭环仍未开放。

## 实际执行的方法

本次由用户明确授权并确认稳定站立、活动区清空及现场可急停。确认只覆盖已完成的这一次试验。采用专用脚本 `scripts/gravity_baseline_session.py` 的 `--bias-only` 分支；原候选中的期望 `+0.01 rad` 运动被取消，650 帧方案没有执行。

控制路径保持 `/mc/upper_body_command → PC1 MC upper_body_external → /aima/hal/joint/arm/command`。稳定站立条件为 `STAND_DEFAULT/RUNNING`、FSM wire 4、body 1；流内为 `UPPERBODY_REMOTE_SPLIT`。MC 保持运行，未修改 `kp=40、kd=2`，未直发 HAL、注入力矩或直连总线。

固定经验证的 HAL 命令基线，左肩索引 0 原始期望始终约 `0.400 rad`，其余 13 轴固定不变，双手保持进入流时的位置。没有以各段编码器反馈重新定义全臂基线，没有积分累加目标。

```text
q_sent = q_desired + alpha × clip(tau_g(q_desired) / 40, ±0.002 rad)
0.400 → 0.402 → 0.400 rad
1 s 基线 → 2 s 五次曲线加偏置 → 1 s 保持 → 2 s 撤偏置 → 1 s 恢复
```

名义 50 Hz、350 帧、7 秒；实际运行会受调度延迟影响，不突发补帧。位置偏置通过位置误差产生附加 PD 力矩，上层消息没有力矩/速度/增益字段；这不是直接重力前馈力矩控制。

模型与标定固定如下：

| 输入 | SHA256 |
| --- | --- |
| `logs/official-model-audit/pc2-sdk/x2_ultra.urdf` | `f84fcd22187d7f3d9730f36b06b39e49fb6a59cd8b1d26bcb5a817dfd300b40a` |
| `logs/official-model-audit/pc2-factory-calibration.yml` | `ce405931791f04ffdc14b6ac511e43e74912f0a86370ab327d20070d6c8dc0ef` |

参考仓库 [Agibot-X2-IK-upper-body-control](https://github.com/maine-cat/Agibot-X2-IK-upper-body-control) 锁定提交 `2f6302ebf6f1b8a2c82c98b5c11ba248858944ea`，仅参考模型补偿思路，未采用其约 12° 默认上限。未额外叠加手质量。胸部/骨盆 IMU 分别结合单位旋转、出厂矩阵及其转置，共六种假设，逐帧要求限幅后全部为 `+0.002 rad`。六种估算均饱和到相同值不能证明外参、HAL 标定应用方式或手部惯性已核实。

## 同步数据与结果

以下均为 **URS 内各驻留末尾 0.2 秒的编码器中位数**；切回站立后的读数不参与回位判定。

| 阶段 | HAL 目标 rad | 左肩编码器 rad | 编码器样本数 |
| --- | ---: | ---: | ---: |
| 原始基线 | 0.400000006 | 0.395473957 | 40 |
| 加偏置保持 | 0.402000006 | 0.395665646 | 32 |
| 撤偏置后恢复 | 0.400000006 | 0.395665646 | 37 |

- 加偏置实际变化和恢复残余均为 `+0.00019168853759765625 rad`。
- 相对原始期望的绝对误差由 `−0.004526049` 变为 `−0.004334360 rad`，撤偏置后仍为后者。单阶梯且未回落的变化不能作为补偿有效的证据。
- 发布 350 帧，实测 `43.123422748 Hz`，最大间隔 `44.018124 ms`，4 个间隔超过名义周期的 1.5 倍；sequence 连续。
- 同步 trace 无丢弃、解析错误；重力采集无错误，运行无反馈保护错误；HAL 完整接收目标变化。
- finally 恢复站立，双臂/双手故障码为 0；随后独立 6 次站立检查通过。现场两份磁盘配置仍为 `control.enabled: false`。

分析结论为 `bias_only_diagnostic_complete`，且 `hardware_acceptance_complete=false`、`model_and_mounting_verified=false`、`execution_authorized=false`。最后一项表示分析报告不授权新动作。报告内恢复残余小于 `0.001 rad` 的单项检查通过，仅适用于此 bias-only 诊断；它不覆盖 `+0.01 rad` 期望运动的跟随、回位与重复性验收。

![命令、HAL、编码器、偏置与 reported effort](../snapshots/20260929-gravity-live-bias-001.png)

reported effort 是反馈字段，不能替代关节位移或解释为已验证的外部负载力矩。旧无补偿五次曲线试验的实际增量 `+0.001342297 rad`、回位残余 `+0.001150608 rad` 仍判失败；两次期望轨迹不同，不能直接比较为补偿改善比例。

## 时序保护与优化

50 ms 是项目诊断脚本对相邻发送间隔的保护，尚未确认为厂商硬限制；名义 50 Hz 对应 20 ms。它不同于重力流间时间偏差 50 ms 检查，也不同于逐帧源/接收数据年龄 100 ms 检查。本次保留全部保护。

优化仅提前计算固定期望姿态的模型系数、避免循环内重复解析和高开销模型计算，并在有界运行窗口暂停 Python 循环 GC、退出恢复原状态。IMU/腰/臂反馈仍逐帧更新和核验；没有缓存旧反馈冒充实时观测。先前 shadow 多次触发时序/新鲜度保护，最终 shadow 与实际试验通过；不能据此认定 GC 是历史延迟的唯一原因。

本地 shadow 统计已采用 `computed_count`，真实发布仍使用 `published_count`。旧 shadow 日志保留原字段，是否实际发送以 `physical_publish_count=0` 为准；不得重写历史日志来统一命名。

## 原始证据与复算

原始 trace：`logs/20260929-gravity-live-bias-001-confirmed.json`，SHA256 `638a088ceee0b311bae28824dc3440b17104070d406fade36161e4164b8a1631`。

同前缀 `.commands.jsonl` 与 `logs/20260929-gravity-live-bias-001.stdout.log` 为命令流水和运行输出。独立检查为 `logs/20260929-after-gravity-live-bias-001.json`；前置证据为 `logs/20260929-before-gravity-live-001.json` 和 `logs/20260929-gravity-live-001-readonly.json`。本地分析与曲线分别为 `logs/20260929-gravity-live-bias-001-analysis.json` 和上图。

只读复算（输出路径必须不存在）：

```bash
python3 logs/analyze-gravity-live-trial.py \
  logs/20260929-gravity-live-bias-001-confirmed.json \
  --output /tmp/gravity-live-bias-001-reanalysis.json
```

该命令只读本地文件，不连接 ROS。`logs/plot-gravity-live-bias.py` 依赖 matplotlib，固定输出已有曲线路径；本轮保留原图，没有重跑覆盖。`logs/`、`snapshots/` 被 Git 忽略，后续交接须单独携带。

本轮远程开发未连接 PC1/PC2。历史两端全量测试为 276 项通过；本轮本地验证及待部署清单见 [会话交接](session-20260929-offline-handoff.md)。下一步候选见 [受限诊断方案](proposals/limited-gravity-diagnostic.md)，不能据当前结果自动加幅或提高增益。
