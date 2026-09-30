# 有界重力偏置的匹配对照与离线判定

当前软件可以预览、记录并分析相同分段的补偿开关对照。**没有执行新的实机试验，没有证明补偿有效，机械臂位置验收仍未通过。** 本次开发只使用本地参考仓库、模型和历史日志，不连接PC1/PC2。

## 从参考仓库得到的改进

再次核查固定提交 `2f6302ebf6f1b8a2c82c98b5c11ba248858944ea`：公开入口经MC/URS，将重力力矩除以假定刚度后加到位置；`x2_sim_ros.py::_send_upper` 明确将零偏置上限用作纯位置对照。公开固定配置使用40、12°及pelvis，这些不是本机的标定结果。

采用的是“相同期望运动、补偿开关对照”和区分 `q_desired/q_sent` 的方法。本项目独立实现，没有复制或运行上游在线入口。仍保持单轴+0.002rad上限，不采用全臂默认补偿、最终目标静默裁剪、每段从编码器重建基线、缺腰角填零、缓存重力重复滤波或直发HAL力矩路径。参考代码的具体依据及差异见 [仓库审查](upstream-ik-audit.md)。

发现并修复一个本地诊断缺口：原先只在实时模型检查中验证 `q_desired` 的URDF限位；`q_sent` 仅检查项目限位。现在在任何模式切换前，对整条计划的期望与发送目标同时检查项目/URDF限位，任一帧越界整段拒绝。测试覆盖“期望轨迹合法，但加偏置后越过URDF限位”的情况。没有修改URDF或放宽限位。

## 两组使用同一条期望轨迹

| 阶段 | 时长 | 帧数 | off偏置 | on偏置 |
| --- | ---: | ---: | ---: | ---: |
| 原始基线 | 1s | 50 | 0 | 0 |
| 偏置渐入 | 2s | 100 | 0 | 五次曲线0→0.002rad |
| 带偏置基线驻留 | 1s | 50 | 0 | 0.002rad |
| 期望+0.01rad去程 | 2s | 100 | 0 | 0.002rad |
| 目标驻留 | 1s | 50 | 0 | 0.002rad |
| 返回原始期望基线 | 2s | 100 | 0 | 0.002rad |
| 回位驻留 | 1s | 50 | 0 | 0.002rad |
| 偏置渐出 | 2s | 100 | 0 | 五次曲线0.002→0rad |
| 最终恢复驻留 | 1s | 50 | 0 | 0 |

两组各650帧、名义13秒、50Hz；保持同一HAL基线、同一原始期望和同一组保护。off组仍检查六假设重力、姿态、故障和数据新鲜度，保留所有阶段；它只将**施加**偏置置零。阶段名称中的 `compensated_*` 在off组保留用于对齐，不表示off组施加了补偿。

`scripts/gravity_baseline_session.py` 新增必填 `--compensation on|off`。旧调用若省略该选项，会在ROS初始化前拒绝，防止误选试验组。`--bias-only` 仍表示350帧、没有+0.01rad期望运动；它不能与650帧位置试验比较补偿效果。`--dry-run` 仍依赖真实传感器，是现场shadow而非离线模拟，本轮没有运行。

日志增加 `compensation_requested` 与 `diagnostic_protocol`（分段、名义频率、幅度、限幅、假定刚度等）。`gravity_compensation_enabled` 只在真实执行on组时为true；shadow仍用 `physical_publish_count=0` 和 `computed_count`，不伪装成发布。每次调用只执行一条计划，不自动切组或重复试验，磁盘控制仍必须关闭。

## 不连接机器人的预览与复核

```bash
uv run x2 plan-gravity-comparison \
  --urdf logs/official-model-audit/pc2-sdk/x2_ultra.urdf \
  --baseline examples/gravity-diagnostic-baseline.json \
  --output logs/gravity-pair-preview-new.json

uv run x2 analyze-gravity-trial \
  logs/20260929-gravity-live-bias-001-confirmed.json \
  --output logs/gravity-bias-review-new.json
```

输出必须不存在。示例基线是历史HAL姿态的圆整数值，**不是当前现场观测或定位命令**。预览锁定已审计URDF哈希并检查两组全部目标，on偏置是假设通过六假设资格检查后的候选值；没有实时IMU，不能从预览文件直接执行或取得现场资格。

将来分别采得新的off/on完整原始trace后，离线比较：

```bash
uv run x2 compare-gravity-trials logs/control-off.json logs/treatment-on.json \
  --output logs/gravity-pair-review-new.json
```

这是文件分析命令；`control-off.json` 和 `treatment-on.json` 是待采集的路径占位符，本轮没有这些实机证据。三个CLI分支均在配置/ROS初始化前返回，测试明确禁止创建X2Client或重启ROS环境。预览/单次分析/比较都不授权动作。

## 判定规则

分析器位于 `src/lingxi_x2/gravity_trial_report.py`。从原始字节计算SHA256，拒绝重复JSON键和非有限数字；不接受已有分析摘要代替原始trace。检查完整分段及严格递增时钟、丢弃/解析错误、连续URS站立、故障与最终恢复、固定其他臂轴和手目标、全部发布帧、逐帧期望/偏置/发送目标、六假设记录一致性、数据年龄/流间偏差、发布间隔与sequence。HAL每个样本须匹配源时钟±50ms内的上层目标，驻留内每个HAL样本均核对目标，避免中位数掩盖错误。这一时钟窗口只用于关联，不是测得的因果延迟。

只有证据质量通过才判位置指标；否则为 `inconclusive`。所有位置窗口仍为URS驻留末尾0.2s。分别输出相对带偏置基线的运动增量、原始期望绝对误差、带偏置基线回位残余、完全撤销偏置后的回位残余。其他轴漂移检查覆盖全部活动样本，不能用驻留中位数隐藏瞬时漂移。阈值沿用既有工程标准（跟随0.002rad、目标绝对误差0.005rad、两种回位各0.001rad、其他轴0.005rad、窗口波动0.002rad）。

单次结果可能为 `fail`、`inconclusive`、`bias_only_diagnostic_complete` 或 `single_trial_pass_under_engineering_criteria`；最后两者都不会设置 `hardware_acceptance_complete=true`。历史旧trace可以复核，缺少新协议元数据时由固定计划重建协议再逐帧核对，不修改原文件。

配对先分别分析两份trace，要求off/on完整650帧、独立不重叠的源时间区间、相同计划/模型/标定/工程标准及相符的HAL基线和手目标。额外采用以下**对照筛选规则**，不是新增厂商安全阈值：实际各阶段时长差不超过较长者的10%，初始驻留14轴编码器差不超过0.001rad，六组重力方向中位数差不超过1°。不满足则 `not_comparable`，不计算“改善百分比”。真实增益、机器人身份/接触负载等仍需现场核对；报告不声称已经独立验证它们。

可比较时返回 `paired_numeric_comparison`，列出各项绝对误差的减少量。超过两个历史观测编码器阶梯（约0.000383377rad）才标注为超出该分辨率筛选线的数值下降/上升，否则为分辨率范围内未分清。这只是粗略筛选，不是统计显著性、厂家分辨率标定或因果有效性证明。即使误差下降，两次位置验收仍可能都失败；仍需同条件重复试验，当前比较器不会输出重复性通过。

CLI退出码：0表示预览生成、单次受限诊断完成或配对数值分析可用；3表示证据不完整、位置失败或不适合比较；2表示文件/JSON/参数错误。**比较命令的0不表示位置验收通过**，需同时查看两份 `sessions[].verdict`。`execution_authorized`、`hardware_acceptance_complete` 始终为false，配对的 `compensation_effectiveness_proven`、`repeatability_verified` 也始终为false。

## 本次历史复核与后续

本地生成 `logs/20260929-gravity-matched-pair-preview.json`（两组各650帧）及 `logs/20260929-gravity-live-bias-001-review-v1.json`。首次trace通过新增43项质量检查，原有五项编码器指标与旧分析逐项一致；全部活动样本中的其他轴最大漂移约0.000958443rad，驻留窗口波动为零。结论仍为bias-only诊断完成，没有新实机结果。

六假设下，0.002rad限幅与模型未限幅偏置之比范围约12.8%–37.3%。这是**模型假设下的截断程度**，不是实测抵消重力百分比；既不能证明重力是唯一问题，也不能据此直接加大偏置。模型/标定适配与机械静摩擦等问题仍未排除。

本轮代码尚未部署PC2。后续先校验现场91文件历史manifest及当前文件、备份并部署、执行PC2离线回归，然后重新确认现场条件。是否执行on/off比较及先后顺序应逐次决定；一次现场确认不覆盖自动两连跑。单轴位置与重复性通过后才继续多轴、手接触/压力和实机模型闭环。
