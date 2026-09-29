# 具身智能实验与回放

当前可用：mock、只读采集、严格 JSONL 回放、模型推理提案日志。实机单轴跟随与回位仍未通过，手部接触/压力也未验收；网页运动和 `ExperimentRunner` 的实机动作循环现明确禁止，避免仅凭配置开关或 `allow_hardware_actions=True` 越过验收阶段。现场固定 HAL 基线诊断仍使用专用脚本并逐次确认。

## 控制台

```bash
uv run x2 --config config/mock.yaml web
```

打开终端显示的本地地址。实时监视展示编码器、双手、前置 RGB 和压力网格。压力使用固定 0–255 原始字节色标，保留源时间和接收龄；网格沿消息数组排列，不表示已标定物理朝向，不转换为 Pa/N。过期或读取失败会明确显示。接口 `verified` 只表示接口/样本检查，不表示运动验收。

“采集与回放”可启动最长 600 秒、1–50 Hz 的后台只读记录，随时请求停止。默认记录臂、手、压力；可选保存前置 RGB 图像字节。缺失必需数据会使任务失败，不伪造样本。任务使用随机 ID，文件保存在 `web.recordings_dir`（默认 `recordings/web`）；浏览器不能指定任意磁盘路径。JSONL 和独立任务状态文件均保留。`completed` 表示采集任务正常结束，`stopped` 表示主动停止；`failed`/`interrupted` 保留失败证据。停止等待当前有超时的传感器读取结束，不强杀线程。

结束后提供 SHA256、帧数、下载和逐帧回放。历史回放保留原时间戳、原 backend 标识，与实时面板分开；目前网页回放展示关节和压力，图像仅显示元数据，完整图像可由 Python 读取。回放没有机器人命令入口。CLI 可按相对时间播放。

网页控制仅限配置已启用的 mock；目标从读到的反馈初始化。发送后返回 `published`、发布统计及 `motion_verified=false`。网页不提供 MC 重启、改增益、力矩注入或真实模型闭环入口。

## 采集与离线验证

```bash
uv run x2 --config config/mock.yaml record recordings/demo.jsonl --duration 2 --rate 10 --camera rgbd_front_rgb --tactile
uv run x2 inspect-recording recordings/demo.jsonl
uv run x2 replay recordings/demo.jsonl --speed 1 --require-images
uv run x2 shadow-replay recordings/demo.jsonl recordings/shadow.jsonl --max-steps 20 --deadline 1
```

实机只读采集使用本机已有的站点配置，磁盘 `control.enabled` 保持 false。首行为 `lingxi-x2-jsonl-v1` 元数据，其余行是 `Observation`，包括源时间戳、接收单调时间和采集单调时间。相机保存 base64 字节或省略图像；`--omit-images` 的记录无法满足 `--require-images`。

读取器逐行校验完整 14 轴顺序、双手轴数、压力尺寸/字节范围、图像长度、时钟顺序，拒绝重复键、非有限数值和截断行。相机字节省略时从 typed observation 的 cameras 中移除，另行保留元数据，不能把空字节当作有效图像。一次读取最多 32 MiB/行；不支持的超大记录应使用独立图像存储。记录每行 flush，但不是断电持久化保证。

## VLA/WAM 模型接口

模型实现 `Policy.reset()` 和 `Policy.act(observation) -> PolicyAction`，或使用 `JointActionAdapter` 包装推理函数。显式输出约定：

```python
from lingxi_x2.shadow import JointActionAdapter

def create_policy():
    def infer(observation):
        # 接入自己的模型及图像预处理；此示例只回显反馈，不是 VLA 模型。
        return {
            "units": "rad",
            "representation": "absolute_joint_positions",
            "duration_s": 1.0,
            "arm_positions_rad": [j.position_rad for j in observation.arm.joints],
        }
    return JointActionAdapter(infer)
```

将工厂放进可导入的可信模块，例如 `my_model:create_policy`：

```bash
uv run x2 shadow-replay recordings/demo.jsonl recordings/model-shadow.jsonl \
  --policy my_model:create_policy --max-steps 20 --deadline 1 --require-images
```

适配器只接受绝对关节弧度位置和明确时长，不推断模型归一化尺度、不自动积分相对动作、不将末端位姿或 action chunk 展平成关节命令。双手字段可使用 `hand_positions_rad: {left: [10 项], right: [10 项]}`，停止使用 `{stop: true}`。具体模型权重、任务指令、相机外参、输入归一化和输出重定向需由模型适配器明确实现，当前尚未验证特定 VLA/WAM 模型。

`ShadowExperimentRunner` 在推理前写入观测，之后写提案、耗时和 deadline 结果，最后写汇总。每个事件 flush，异常写 error 后传播，既有文件拒绝覆盖。`executed_actions=0`；提案结构校验不等于碰撞、限位或实机安全验收。deadline 在推理返回后评估，超时停止后续推理，不能强制终止卡住的 Python 函数。插件是可信用户代码，runner 不具备机器人写接口，但不构成限制插件自行联网的沙箱。

实时只读模型实验可在 Python 中将 `client.stream_observations(...)` 交给 `ShadowExperimentRunner.run(...)`；需要调用方管理客户端生命周期。硬件动作必须等待单轴跟随/回位与重复性、多轴、灵巧手接触/压力全部通过，再实现具备验收证据检查的真实执行器。

离线重力工具见 [开源审查](upstream-ik-audit.md) 和 [官方模型核查](official-model-audit.md)。计算报告不生成实机位置补偿，不授予运动许可。
