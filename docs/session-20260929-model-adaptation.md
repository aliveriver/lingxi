# 2026-09-29：X2 Ultra + O10 模型适配续查

本次完成当前代码/文档/历史证据核对，并补查官方旧版模型 ZIP、机器人仓库分支/issue 和 O10 SDK 文档。**仍未取得可用于本机的完整装配模型与坐标合同，不能进入已校准组合模型或增加实机补偿。** 新发现是 SDK 文档、SDK 头文件和独立手 URDF 之间还有范围及被动轴表达差异，需要明确版本关系。

全程没有连接 PC1/PC2、初始化 ROS、加载或执行厂家 SDK、部署或进行新实机试验。当前现场状态未知。保留已有未提交改动与冻结包；本次新增资料复算脚本不接入控制路径。

## 补查的公开来源

证据目录：`logs/model-adaptation-followup-20260929/`。

| 来源 | 本次取得的证据 | 结论范围 |
| --- | --- | --- |
| [官方 SDK 下载页的 X2_URDF.zip](https://x2-aimdk.agibot.com/zh-cn/latest/_downloads/3f0ab2c08bf92924fd76963bbea7a23a/X2_URDF.zip) | `legacy-official-models.zip`，90,489,486 bytes；SHA256 `aeb0155a372111a7421397883f8570ca6b291908ee6e39214381b2a3652f48c2` | 读取9份URDF；未解压安装或运行包内内容 |
| [机器人仓库分支](https://api.github.com/repos/AgibotTech/agibot_x2_urdf/branches?per_page=100) | `robot-branches.json` | 本次只返回main，仍为 `575cc6b988f976c23550e0db85aa1e5475d3652d` |
| [全部状态issue列表](https://api.github.com/repos/AgibotTech/agibot_x2_urdf/issues?state=all&per_page=100) | `robot-issues.json` | 本次返回空列表；不是“厂家没有其它资料”的证明 |
| [O10 SDK固定提交](https://github.com/AgibotTech/agillink_omnihand_sdk/tree/c23801f03e396d8d667a307548a05b5d0aecf9aa) | `sdk-commit.json`、`blob-*.md`、`blob-omnihand_2025_solver.h` | 固定2026-09-25提交；没有证明与本机Agi v1.1.4的内置版本相同 |

raw.githubusercontent.com 的四次文档请求超时，失败保存在 `sources.json`；随后通过已保存树中的固定 Git blob 地址取得文档，见 `blob-downloads.json`。没有把超时解释为文档不存在。头文件新下载字节与前轮一致。

ZIP 中 v1.3 有 `x2_fist.urdf`、`x2_ultra_simple_collision.urdf`、`x2_ultra.urdf`，均未提供左右 wrist_roll 到 O10 掌根的连接。标准 `x2_ultra.urdf` 与前轮官方文件 SHA256 完全相同。含手掌安装 joint 的整机模型仍是 v1.4 的 `X2-Ultra_omnihand.urdf`；不能据此为非-N机型填入外参。逐模型成员、哈希和连接见 `legacy-model-inventory.json` 与复算报告。

## SDK 范围不能直接当成本机 HAL—URDF 映射

来源为固定提交的 [O10 C++ API](https://github.com/AgibotTech/agillink_omnihand_sdk/blob/c23801f03e396d8d667a307548a05b5d0aecf9aa/doc/zh_cn/API_CPP_O10.md) 第293行起左右手关节表，以及 solver 头文件。SDK 文档表的编号为1–10；本项目 HAL 数组下标为0–9。同后缀关节顺序相符，但不能把表编号当数组下标。

| 轴（HAL下标） | SDK左手文档 rad | 独立左URDF rad | SDK右手文档 rad | 独立右URDF rad |
| --- | --- | --- | --- | --- |
| thumb roll（0） | [-1.12, 0.03] | [0.034907, 1.274090] | [-0.03, 1.12] | [-0.523599, 0.698132] |
| thumb abad（1） | [-0.05, 1.64] | [-1.6424, 0.045379] | [-1.64, 0.05] | [-1.464235, 0.045379] |
| index abad（3） | [0, 0.16] | [-0.087266, 0.104720] | [-0.16, 0] | [-0.226893, 0] |
| 四指 PIP（4/5/7/9） | [0, 1.48] | [-π/2, 0] | [0, 1.48] | [0, π/2] |

只读复用首次偏置trace末尾 `final_hands`，原始trace SHA256仍为 `638a088ceee0b311bae28824dc3440b17104070d406fade36161e4164b8a1631`：

- 对独立URDF直接比较，左7/10、右0/10原值在范围外，与前轮一致。
- 对SDK文档表直接比较，左6/10、右4/10原值在范围外。双手四指PIP均约1.564–1.570rad，大于文档1.48rad；左食指abad约−0.000879rad、无名指abad约+0.000498rad也在文档区间外。
- 这些是**无容差的文件数值比较**，不是新的驱动故障、实物越限判断或新版安全限位。靠近零位的差异不应与大范围差异等同解释。报告不裁剪、不拟合零位、不修改控制符号。

头文件 `kLeftDirection=[-1,-1,-1,-1,+1,+1,-1,+1,-1,+1]` 与文档左右范围方向相符，但这是SDK内部声明，**不是 HAL→独立URDF 的方向表**。它尤其不能替代前轮“左手统一取反在一帧落入范围”的待验证假设。纯符号翻转也不能使上表全部区间一致；差异可能涉及零位、限位政策或硬件/软件版本，现有证据不够区分。

## 被动轴关系与全轴返回顺序还需确认

独立URDF每侧有6个mimic轴。四指DIP采用固定比例，左约−1.144444、右约+1.144444；拇指PIP为MCP的1.33倍，DIP为1.3倍。

同一SDK头文件声明：

| 字段 | 源码声明的系数数组 |
| --- | --- |
| `finger_pip2dip_poly_` | `[0, 2.192, -1.425, 0.747, -0.167]` |
| `thumb_mcp2pip_poly_` | `[0, 1.33]` |
| `thumb_mcp2dip_poly_` | `[0, 1.846, -0.853, 0.280]` |

这说明不能假定独立URDF的线性mimic就是当前SDK的全部耦合关系。**本轮没有取得并验证求值实现、系数幂次约定、左右方向应用位置或本机HAL调用关系**，因此不计算或宣称实际SDK—URDF的角度误差，更不改mimic。下一步需厂家给出适用版本的被动轴关系及准确度范围。

C++设备API文档称全轴结果为“前10主动、后6被动”，solver头文件的 `OmnihandJoint` 枚举却按手指穿插主动/被动轴（thumb PIP/DIP位于index abad前）。两者可能是不同接口做了重排，不能据此判定某个实际API输出错误。运动学C++文档还使用 `GetAllJointAngles`，该solver头文件声明的是 `GetAllJointPos`。未来适配时须逐接口确认，不能拿一种枚举解释另一种返回数组。

SDK自己的 `/o10/...` ROS2 接口文档不是本机 `/aima/hal/...` 的接口合同。其电流字段、触觉点数宣传和单位也不能移植为本机282格压力的标定。当前控制路径及压力语义保持原状。

## 资料交付清单（尚未向任何人发送）

需要厂家或已有本机交付资料明确以下内容。可以用完整装配URDF/CAD及说明代替分散表格，但必须指向 **X2 Ultra非-N / v1.3 + 本机O10版本 / Agi v1.1.4**。

| 项目 | 需要给出的具体信息 | 当前值 |
| --- | --- | --- |
| 左右安装外参 | 两侧分别给 `p_wrist = R_wrist_from_palm × p_palm + t_wrist_from_palm`；m/rad、右手系、矩阵方向；父帧为对应 `wrist_roll_link`，子帧为 `l_palm` / `R_palm`；附图纸/型号版本 | 未知，不能默认为单位阵或借v1.4 |
| 20个主动轴坐标 | 每轴HAL下标、HAL反馈/命令含义、URDF名、`q_model = s × q_HAL + b`中的s/b及单位、适用范围；若非仿射应提供实际关系；已知姿态及正方向证据 | 未知；不提交猜测符号或零位 |
| 12个被动轴 | 左右分别提供耦合源轴、函数/系数/零偏、适用范围，以及设备API和solver全轴返回顺序 | 独立URDF定义已知，本机适用性未知 |
| 惯性部件分账 | v1.3腕roll聚合惯性包括哪些部件；保留/拆除/新增的腕壳、板、护盖、螺钉、手本体逐项质量、COM及惯性张量/坐标；完整装配来源版本 | 未知，不能直接加0.5343kg或跨机型减质量 |
| 出厂标定与HAL | 臂/手零位是否已应用；哪些字段影响哪些流；胸/胯IMU四元数定义、外参应用位置和明确向量变换公式，标定矩阵是否需转置 | 未知，不重复应用标定 |

软件身份与硬件身份需关联：O10硬件版本、手固件、Agi内置驱动/solver版本，以及上述独立URDF适用序列。无需远程尝试连接机器人来填空；当前可先查本机已有交付文件或厂家提供的静态资料。

## 可复算结果与接续

新增 [只读复算脚本](../scripts/audit_o10_source_contract.py)，固定输入哈希，拒绝覆盖输出，只解析文本/XML/ZIP，不运行厂家代码或生成控制配置：

```bash
python3 scripts/audit_o10_source_contract.py \
  --output logs/model-adaptation-followup-review-new.json
```

`--root`可指向保留上述相对证据路径的工作区副本。退出码0只表示审查报告生成，`combined_model_ready=false`、`hardware_validated=false`、`execution_authorized=false`始终保持；文件缺失、哈希不同或输出已存在为退出码2。这个脚本固定审查本次来源，新版本必须另行审查，不通过修改预期哈希冒充已验证。

本次输出为 `source-contract-audit.json`；验证记录为 `verification.json`，包括源哈希、SDK blob与固定树关联、复算一致性、拒绝覆盖和修改输入后的拒绝。`manifest.json`覆盖本轮证据，原始下载失败记录也保留。没有生产控制代码变化，不重跑或扩大330项软件测试结论；PC2仍只有历史276项通过记录。脚本及本页不在前轮29文件冻结包中，本轮没有重打包。

本地关闭配置、原始bias-only trace、SDK锁定模型、出厂标定副本及冻结包五项哈希均与交接一致；没有本地 `config/x2.yaml`。后续先补齐上述资料，再独立版本化组合模型、验证FK/COM和质量分账；不自动替换实机锁定模型。位置跟随、URS回位、重复性、手接触压力、实机模型闭环的验收状态均未改变。
