# 部署预演与基线资格检查（2026-09-29）

本轮完成两个离线交付：**冻结包校验/部署预演工具**，以及**进入补偿配对比较前的基线资格检查**。工具不会连接PC1/PC2、发布命令、修改MC或延长实机轨迹。物理跟随、URS回位及重复性依旧未验收。

## 部署工具的实际能力与边界

[deployment_review.py](../src/lingxi_x2/deployment_review.py)读取压缩包与侧车清单，要求调用者显式提供可信SHA256。核对外层哈希、内外清单、软件/证据分类、全部成员哈希；拒绝重复成员、绝对/上跳/不规范路径、链接、非普通文件、文件与父目录冲突及超过大小预算的包。没有调用tar自动解包。软件路径白名单不包含`config/`、`logs/`、`snapshots/`和现场软件清单；历史证据独立分类。

`inspect_target()`对外部目录**只读**：要求真实`SOFTWARE-SHA256.json`全部文件校验通过，并且两份实际配置中`control.enabled`严格为布尔false，字符串`"false"`不算关闭。拒绝符号链接、被修改的已登记文件、不同内容的未登记文件碰撞及历史证据碰撞；相同内容的证据允许保留。不会补造缺失配置或修改现场manifest。

[rehearse_deployment.py](../scripts/rehearse_deployment.py)只在自己创建的临时目录内生成**合成现场**，包含新旧软件、带注释的关闭配置及历史证据哨兵。它验证备份字节、构造合并软件清单、模拟正常写入；在第0次至最后一次文件写入后逐一注入异常，恢复旧字节并移除本次在临时目录新增的文件，检查原有配置/证据保留。文件替换使用临时文件和`os.replace`，已有文件权限在正常替换时保留。

这是事务设计的本地预演，**不是PC2快照或实机安装器**。测试覆盖文件替换完成后的Python异常，不覆盖断电、磁盘写满、权限/所有者恢复、并发写入、文件系统持久化或aarch64/ROS运行环境。备份仅在该次临时预演中使用，结束后随临时目录释放；报告保留备份哈希和恢复清单，不将临时备份冒充现场可用备份。没有对用户目录提供写入或自动回滚入口。

对旧29文件冻结包的预演命令：

```bash
/home/a156/.local/bin/uv run python scripts/rehearse_deployment.py \
  --archive logs/20260929-gravity-comparison-update.tar.gz \
  --sidecar logs/20260929-gravity-comparison-update-manifest.json \
  --sha256 67439ba7c6f4b2737e841f3c763428d053b0dd6d02b85d2b0240589afaf5ca8e \
  --output /tmp/new-deployment-rehearsal.json
```

可选`--inspect-target /absolute/path/to/offline-copy`会先对已有离线副本做只读核验；缺配置/manifest则拒绝，不能把本机工作区当作现场副本。本机仍没有`config/x2.yaml`，工具未重建它。所有输出都独占创建，不能覆盖已有证据。

## 新基线检查，不改旧报告与执行器

[baseline_review.py](../src/lingxi_x2/baseline_review.py)复用原始trace质量检查，额外检查以下项目。旧`gravity_trial_report.py`、旧CLI、冻结报告和实机脚本不修改；新配对入口为[review_baseline.py](../scripts/review_baseline.py)。

| 项目 | 新离线资格标准 |
| --- | --- |
| 窗口 | 去程/加偏置前基线末0.4秒，分成两个0.2秒窗口 |
| 数据覆盖 | 最大接收间隔50ms；每半窗上层命令至少5帧，HAL/编码器各至少10帧 |
| 固定目标 | 整个基线段14轴上层/HAL目标与固定HAL基线误差≤1e−5rad |
| 编码器漂移 | 14轴两半窗中位数差绝对值≤0.000193rad |
| 编码器跨度 | 14轴末0.4秒各自最大减最小≤0.000385rad |
| HAL增益 | 从基线末0.4秒开始，直到URS恢复段结束，14轴每个样本均明确为40/2；缺失、非有限值或其他值拒绝 |

漂移/跨度约取一/两个历史观察阶梯并留文件表示余量，是显式工程审查准则，**不是厂家精度、统计显著性或最终静态平衡证明**。源码中保留准则数值，输出逐轴漂移、跨度、样本数、增益字段变化及所有检查结果。进入流早期增益过渡如发生在末尾窗口之前，会被报告而不自动否决；进入末尾窗口后发生过渡则拒绝比较。

```bash
# 单份历史trace：只判断新基线证据是否足够
/home/a156/.local/bin/uv run python scripts/review_baseline.py \
  logs/20260929-gravity-live-bias-001-confirmed.json \
  --output /tmp/new-baseline-review.json

# 未来取得真实off/on匹配trace后，使用新入口；此处是文件占位名
/home/a156/.local/bin/uv run python scripts/review_baseline.py \
  /path/to/off-trace.json /path/to/on-trace.json \
  --output /tmp/new-qualified-pair.json
```

成功返回0，不合格/不可比较返回2；不合格时仍保存原因报告。新配对入口必须同时通过旧配对条件和新基线检查，并核对两次只读分析之间源哈希不变。失败时不输出“合格的改善指标”；任何情况下`execution_authorized=false`，不授予重复性或有效性通过。

本轮五份历史trace均通过这项**追加的基线证据检查**：末尾双窗口14轴中位数差均为0，后续HAL为40/2。负向开流早期50/3→40/2变化仍清晰保留。这没有推翻原来的四份运动fail及一次bias-only诊断完成，也不让五份不同条件记录变成可比较配对。不能靠基线检查解释或解决物理跟随不足。

## 冻结交付、验证及下一步

[build_review_bundle.py](../scripts/build_review_bundle.py)根据显式软件/证据文件列表冻结到**全新目录**，拒绝配置路径、链接与输出目录复用，并在写完后复核全部成员。本轮新包独立保存在`logs/offline-readiness-20260929/current-software-bundle/`，旧29文件冻结包不变。

新包选择表为同证据目录的`bundle-selection.json`，包含当前src/scripts/tests/docs/examples、README、项目依赖声明和锁文件，以及本轮全量测试和基线复算证据。它不包含任何配置、原始运动trace、大型官方模型ZIP/STEP、依赖安装环境或自动安装器；模型来源审查仍须另外携带原证据目录。它是显式文件快照，不是已核对PC2版本差异的可直接覆盖清单。包内有测试/文档引用未随包复制的历史证据，不能把新包称为完整现场或完整证据备份。

本轮测试、包文件数/SHA256、旧包与新包预演结果见同目录`delivery-summary.json`和`verification.json`。测试包括路径/链接攻击、清单损坏、配置缺失/启用、证据碰撞、逐次写入失败恢复、早晚增益变化、其他轴漂移、缺失增益以及不合格配对不输出改善指标。

开发中曾发现新检查器把上层命令流错误套用反馈流每窗10帧要求，导致正常43–46Hz历史命令流被拒绝。最终将两类最低样本数分开，并补充25Hz命令流、反馈保持100Hz的合成测试；50ms覆盖保护未改。这是新增分析器修正，没有放宽旧验收阈值。所有合成数据仅证明软件行为。

下一次恢复现场时，仍先核验实际manifest、两份关闭配置与任何差异，制作真实备份，再审查部署方案及运行PC2离线测试；本工具不能替代这些步骤。每次实机动作仍需重新现场确认，本轮没有自动连跑、重试、加幅或开放闭环。`logs/`被Git忽略，交接需单独保存。
