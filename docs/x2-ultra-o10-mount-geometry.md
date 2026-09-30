# X2 Ultra + O10：直装接口与模型几何续查

2026-09-29，依据用户确认的“掌根直接固定安装在机器人手腕上”，继续离线检查官方模型几何及安装资料。本次取得了 **O10手侧接口图和左右STEP**，并确认 **v1.3腕roll网格包含整只固定手外形**。这使问题更明确：需要确定实际保留腕部到掌根的坐标关系，以及替换旧末端后的惯性，而不是给原模型简单挂上另一只手。

本轮未连接PC1/PC2、执行厂家SDK、部署或进行实机操作。没有修改控制符号、补偿幅度、锁定模型及配置，也没有输出已校准组合URDF。当前腕掌变换仍未建立。

## 新取得的官方机械资料

来源仍是 [O10官网资料页](https://www.agibot.com.cn/DOCS/OS/Omnihand-O10)，本轮从已保存的页面中提取此前未审查的链接。文件在 `logs/wrist-palm-geometry-20260929/downloads/`，来源、UTC时间、下载状态和哈希见 `sources.json`。

| 文件 | 官方链接 | SHA256 |
| --- | --- | --- |
| 产品说明书，35页 | [PDF](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787811897597743.pdf) | `9361ae69ac522846b3a3f4cc0132d2e170debe5e90165c2da3b35e08bc6cebec` |
| 左手三维模型ZIP | [左手模型](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813704283815.zip) | `9bfeda661dd24edf26f92bc939b89987dc495efefaf3e6515d0393f6d362f3e5` |
| 右手三维模型ZIP | [右手模型](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813704317306.zip) | `5c2733a56582e267361e777e4f02761638989f670bc09cf047eba5eb6a64622e` |

PDF第13–14页明确描述手腕后端盖、4颗M4×8螺钉及手侧机械接口图。图中可读到φ38 H7、φ27.5及4×φ4.4等尺寸，完整公差和孔位以原图为准；本轮没有将图纸值写入模型。第14页的ISO9409-1-50-4-M6描述属于“使用其他机械臂”的转接法兰参考，**不能当作X2直装接口定义**，也不据此假设本机有转接法兰。本页引用安装说明作为资料依据，不要求执行拆装。

已提取并查看原始图像和整页渲染：

- [手侧机械接口图](../logs/wrist-palm-geometry-20260929/manual-page-14-image-1.jpg)。
- [PDF第14页及适用上下文](../logs/wrist-palm-geometry-20260929/manual-page-14.png)。
- 第13页安装说明保存在同目录 `manual-page-13.png`，完整提取文本在 `downloads/o10-manual.txt`。

两个CAD ZIP各包含一份完整手STEP文件。已逐字节计算成员哈希、解析文本中的产品名称，发现“O10手腕保护板”等部件。没有加载CAD几何内核、辨识装配约束、材料密度或确认本机零件版本；文本中未找到X2装配关系，不能把“没有关键词”当作完整几何证明。

有效清单为 `cad-inventory-v2.json`。初版 `cad-inventory.json` 的搜索误把STEP Unicode标记 `\X2\…\X0\` 当成机器人名，并漏掉带空格的PRODUCT记录；已保留失败结果，v2先解码Unicode再搜索，得到左148、右189条PRODUCT记录。没有用初版关键词命中推断任何安装参数。

## v1.3腕roll网格包含固定手外形

从已锁定来源的官方ZIP读取左右 `wrist_roll_link.STL`，没有修改或重导出。可视化确认网格同时包含腕部支架及完整手掌、手指外形；URDF将其作为一个刚性link，未提供这只固定手的独立关节或部件质量。

| 网格 | 本地Z范围 mm | Z向跨度 mm |
| --- | ---: | ---: |
| v1.3左 `wrist_roll_link` | −181.702至+16.500 | 198.202 |
| v1.3右 `wrist_roll_link` | −181.722至+16.500 | 198.222 |
| 独立O10左掌根 | −0.043至+113.451 | 113.494 |
| 独立O10右掌根 | −0.042至+113.452 | 113.494 |

各网格三视图见 [投影图](../logs/wrist-palm-geometry-20260929/review/mesh-projections.png)。各行使用自身坐标，红色十字为本地原点；没有做跨模型安装、配准或碰撞验收。O10掌根网格还没有包含另外各指节link，不能用其113.5mm高度和整只固定手高度直接求安装偏移。

这解释了为什么“臂树止于wrist_roll_link”不等于“物理裸腕”。现有腕roll质量左0.303040kg、右0.303847kg的部件组成仍不能由网格单独确定：视觉几何没有材料密度/质量分账。不能直接保留全部聚合惯性再加0.5343kg，也不能依据可见外形减去一个猜测的旧手质量。

## 手包中旧CSV和当前URDF并不等价

左URDF包内还包含 `l_palm_old_frame_backup.STL`，与当前 `l_palm.STL` 各128,377个三角形，保留相同顶点次序。对385,131个对应顶点做严格旋转（禁止镜像）和平移拟合，得到：

```text
p_current_palm = R × p_old_palm + t
R ≈ [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
t ≈ [83.629692, -26.287473, 44.656273] mm
```

最大顶点残差约5.21×10⁻⁹m、RMS约2.26×10⁻⁹m，只反映同一mesh坐标改写与STL浮点精度，**不是物理测量精度，更不是腕掌安装外参**。

用此变换将左CSV的掌根COM转换到当前坐标，与URDF COM残差约4.98×10⁻¹⁰m，确认CSV的掌根COM仍在旧坐标下。然而同样转换CSV中5个掌根子关节的origin后，仍有约0.101–0.374mm差异。因此不能只换坐标就把整份CSV当作当前模型。

| 质量条目 kg | 左CSV | 左URDF | 右CSV | 右URDF |
| --- | ---: | ---: | ---: | ---: |
| 掌根 | 0.090609 | 0.217500 | 0.133697 | 0.217500 |
| 全手合计 | 0.217300 | 0.534300 | 0.235665 | 0.534300 |

右CSV掌根COM及5个掌根子关节origin与当前URDF数值一致，但质量仍不同。CSV可能来自不同导出/修订阶段，现有资料不能认定哪组质量代表本机。没有用CSV覆盖URDF，也没有把STEP体积换算成质量。

## 可复算脚本与验证

新增 [audit_wrist_palm_geometry.py](../scripts/audit_wrist_palm_geometry.py)，只读取三份固定哈希的公开ZIP，解析STL/CSV/XML。默认不依赖绘图库；输出目录必须不存在。例：

```bash
/home/a156/.local/bin/uv run python scripts/audit_wrist_palm_geometry.py \
  --output-dir logs/wrist-palm-review-new

# 可选：创建用于审查的独立PNG，不修改项目依赖
/home/a156/.local/bin/uv run --with matplotlib python scripts/audit_wrist_palm_geometry.py \
  --output-dir logs/wrist-palm-review-with-plot-new --plot
```

本轮输出在 `logs/wrist-palm-geometry-20260929/review/`。17项专项检查通过，记录为同目录上一级 `verification.json`：独立合成刚性变换、镜像不得冒充旋转、STL有限值/截断/长度检查、实际复算一致、输出目录拒绝复用、源哈希变化拒绝、旧manifest及五项交接哈希完整性等。已目视检查生成的网格投影与说明书接口图。没有生产控制代码变更，未重跑全量测试；330项本地和276项PC2均仍为此前记录。matplotlib、PDF阅读工具仅为临时uv工具依赖，`pyproject.toml`和`uv.lock`未改变。

## 当前可确定与下一步

已知：X2 Ultra非-N + O10；用户确认掌根直装、固定连接；手侧说明书/接口图/STEP已取得；v1.3腕roll视觉几何包含原固定手；手包CSV不能直接替代当前URDF。

下一份关键资料应是 **本机实际保留腕部（含拆除原固定手后的结构）的装配CAD/坐标定义或完整v1.3+O10模型**。它需把手侧接口面关联到 `left/right_wrist_roll_link`，给出左右旋转和平移，并说明保留/替换部件的惯性。手侧图纸中没有URDF父子坐标标记，仅凭直装和外形不足以消除这些未知。

HAL到独立手URDF的逐轴映射、被动轴版本和出厂标定应用仍按[来源续查](session-20260929-model-adaptation.md)待核；本轮几何证据不会解除它们。资料齐备后才能做组合模型FK/COM验证。仍不使用v1.4安装joint代替本机参数，不自动替换实机诊断锁定模型。
