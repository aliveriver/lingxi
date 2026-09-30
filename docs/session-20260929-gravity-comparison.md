# 2026-09-29 重力匹配对照开发交接

后续用户确认X2 Ultra + O10，已完成 [官网URDF重新下载和兼容性核查](x2-ultra-o10-compatibility.md)。该专项没有生产代码变更，未替换本页冻结软件包；模型/映射缺口与后续所需资料见专项报告。

进一步[模型适配续查](session-20260929-model-adaptation.md)补查官方ZIP、O10 SDK文档/头文件并增加只读来源复算脚本。仍未取得可用的v1.3+O10装配与映射合同；新增材料没有放入本页冻结包，也没有部署PC2。

## 目标与限制

继续X2 Ultra / Agi v1.1.4安全上肢实验平台。用户要求根据参考仓库继续分析和改善代码；当前远程环境无法连接机器人，明确禁止尝试SSH连接PC2。本轮仅本地读代码、修改代码和离线验证，没有连接PC1/PC2、执行实机动作或部署。

最终目标仍为可靠机械臂/灵巧手控制、压力读取与可视化、采集回放及VLA/WAM接口。软件进展不能替代实体位置验收：跟随、URS内回位及重复性仍未通过，手接触/压力与实机模型闭环未开放。

## 本轮代码改进

- 只读核对参考副本 `logs/reference-x2-ik-2f6302e`，HEAD仍为 `2f6302ebf6f1b8a2c82c98b5c11ba248858944ea`、工作区干净。参考其零偏置对照、五次曲线与期望/发送目标分离方法；没有复制上游代码、采用12°默认上限、逐段编码器基线或HAL直发路径。依据写入 [审查报告](upstream-ik-audit.md)。
- `src/lingxi_x2/gravity_diagnostic.py` 新增匹配分段协议、补偿开关和整条计划的URDF目标检查。off/on各650帧、相同期望/分段；off仅施加零偏置，所有保护保持。两组只改变左肩pitch索引0，固定其他13轴HAL目标及手位置。
- 修复专用诊断的限位检查缺口：此前模型校验主要覆盖期望姿态，现在切模式前连同偏置后的发送目标一起逐帧检查，超限整段拒绝，不静默裁剪。
- `scripts/gravity_baseline_session.py` 新增必填 `--compensation on|off` 和协议元数据。旧调用缺少选项时会在ROS初始化前退出。`--bias-only`仍为350帧无期望运动；没有自动执行两组的入口。六假设、新鲜度、站立、故障、50ms发送间隔、finally恢复及磁盘控制关闭要求保持。
- 新增 `src/lingxi_x2/gravity_trial_report.py`，把诊断分析整理为正式可测试模块。检查原始trace而非摘要，分别判断证据质量、运动数值及配对条件；不完整数据为inconclusive，bias-only不标记为位置验收通过，配对可比较也不等于有效性或重复性已证明。
- CLI新增 `plan-gravity-comparison`、`analyze-gravity-trial`、`compare-gravity-trials`。全部离线、输出独占创建、原始字节SHA256关联；不初始化机器人。操作及退出码见 [使用文档](gravity-comparison.md)。
- README、arm、platform-status、validation、候选方案与补偿说明已更新。原收尾报告和冻结包保持可追溯，参见 [前一阶段交接](session-20260929-offline-handoff.md)。

## 验证与证据

本机全量 **330 passed, 1 warning in 35.64s**，记录 `logs/20260929-gravity-comparison-full-local-tests.txt`；警告仍为Starlette/httpx弃用提示。新增54项覆盖匹配轨迹、叠加偏置后越URDF限位、CLI参数/离线隔离、源数据损坏、HAL斜坡错误、时钟/新鲜度/故障、URS回位窗口及不可比较的试验。

一项时序对照负向测试首次失败，原因是合成夹具共享字典对象导致时间被重复缩放；转成与实际JSON文件一致的独立对象后通过。未因此放宽实现检查。合成测试仅用于软件验证，不是物理模拟或新的实机证据。

额外只读检查与新输出：

| 文件/检查 | 结果 |
| --- | --- |
| `logs/20260929-gravity-matched-pair-preview.json` | off/on各650帧，相同分段和期望；基线示例为圆整的历史HAL姿态，没有当前现场资格 |
| `logs/20260929-gravity-live-bias-001-review-v1.json` | 首次历史trace通过43项质量检查；原五项编码器指标与旧分析完全一致，结论仍为bias-only诊断完成 |
| `logs/20260929-gravity-comparison-verification.json` | 额外核验记录；对比上次部署包，默认on的350/650帧计划逐帧完全一致；原始证据及本地关闭配置哈希未变 |
| `examples/gravity-diagnostic-baseline.json` | 明示的离线基线输入示例，不向机器人发送定位命令 |

首次trace SHA256仍为 `638a088ceee0b311bae28824dc3440b17104070d406fade36161e4164b8a1631`。实机发送0.400→0.402→0.400rad，编码器增量与撤偏置后残余均为+0.000191689rad；没有+0.01rad期望移动，也没有0.2rad试验。本轮新分析显示限幅/模型未限幅偏置比例为12.8%–37.3%，只描述假设模型的截断程度，不能解释为实际抵消重力比例或加幅依据。

## 待部署材料

新本地冻结包 `logs/20260929-gravity-comparison-update.tar.gz`；清单 `logs/20260929-gravity-comparison-update-manifest.json`、包校验文件 `logs/20260929-gravity-comparison-update.sha256`，由 `logs/package-gravity-comparison-update.py` 生成。包内逐文件 `UPDATE-SHA256.json` 及侧车区分软件/证据。没有现场配置或自动安装/运动入口；仅为待审查同步材料，不是PC2已部署。

不要用上一阶段 `20260929-gravity-offline-closeout.tar.gz` 代替本次代码；也不要直接套旧 `apply-gravity-live*.py`，其文件白名单不同。旧冻结包及旧原始日志未改动。`logs/`、`snapshots/`被Git忽略，Git提交不会携带这些证据和包，需保留工作目录或单独转移。

PC2最后已知是 `/home/run/lingxi`、91文件manifest、276项全量通过；本轮没有复核现场文件或测试。本地 `config/x2.motion-test.yaml` 仍关闭且SHA256为 `b731e3ef26643fadbb7c682cef1c30d4b58ce5a03f74e63305d28e0e533423f5`，本地仍没有 `config/x2.yaml`。没有推测重建现场配置，也没有修改MC配置或增益。

## 接续顺序

1. 在仍无法连接机器人时，继续用离线CLI/测试审查数据和方案；预览文件不用于执行。保持参考仓库固定版本，不将上游默认参数视为本机标定。
2. 恢复现场访问后，先核验PC2当前manifest的全部文件及两份关闭配置，调查任何差异；校验新包成员/哈希并备份旧软件、manifest与现场配置，再部署。保留配置字节，证据路径如已存在先比对、不得覆盖历史日志。随后运行PC2全量离线测试。更详细步骤沿用前一阶段交接。
3. 核查模型/安装旋转方向、HAL是否已应用标定及腕部惯性是否含手等不确定项。下一轮要判断的是补偿能否改善跟随/回位，不能把偏置可发布当成目标完成。
4. 若现场决定执行匹配对照，每次重新确认稳定站立、活动区清空、现场可急停；同组先shadow。候选仍为期望+0.01rad、on上限+0.002rad、固定HAL基线及相同650帧分段，off只取消偏置。两组分开执行，每次后先分析，不预授权自动连跑、重试或加幅。
5. 用新比较器核对两组证据与条件，分别报告增量跟随、原始期望绝对误差、两种URS回位残余与其他轴漂移。单次运动阈值通过后才安排至少三次同条件重复性验收；当前配对分析不授予重复性通过。
6. 单轴及重复性通过后，依次多轴、灵巧手接触/压力、实机VLA/WAM闭环。一般补偿runner、Web动作和模型执行仍限mock。

禁止沿用历史现场确认、每段编码器重设全臂基线、绕过反馈/站立/故障检查。未授权修改增益、注入力矩、停止/重启MC、刷固件或直连总线；自由站立时不能重启维持平衡的MC。本轮没有Git提交、PC2部署或新的实机验收。
