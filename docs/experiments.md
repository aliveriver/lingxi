# 具身智能实验

策略只处理领域对象：

```python
from lingxi_x2 import X2Client
from lingxi_x2.experiments import ExperimentRunner, PolicyAction

class ObserveOnlyPolicy:
    def reset(self):
        pass

    def act(self, observation):
        image = observation.cameras["rgbd_front_rgb"].data
        arm = [joint.position_rad for joint in observation.arm.joints]
        # model inference here
        return PolicyAction(stop=True)

with X2Client("config/x2.yaml") as x2:
    ExperimentRunner(x2, ObserveOnlyPolicy()).run(max_steps=100, allow_hardware_actions=False)
```

`Observation` 包含捕获单调时钟、机械臂、双手、触觉和相机映射。模型适配器可将 JPEG 解码为 numpy/PyTorch，也可直接把压缩字节交给数据管线。`PolicyAction` 可返回双臂和多个手目标。

默认 `allow_hardware_actions=False`，即使策略意外输出动作也会被确认互锁拦截。实机动作开启前必须先用 mock 回放、再做现场单步验证。

2026-09-29 状态：VLA/WAM 可通过现有 `Policy` 接入 mock 或只读观测/采集；真实闭环实验尚未验收，机械臂三次小幅测试未证明跟随，灵巧手与有载触觉仍待验证。当前 `ExperimentRunner` 按顺序执行臂和手的阻塞轨迹，不提供同步全上肢 action chunk、实时推理 deadline 或模型专用适配器；不能将它描述为已可用的实机 VLA/WAM 闭环。

运动调用现在返回逐帧发布统计，最近一次结果也可从 `x2.publication_stats()` 获取。它提供本地发送时序证据，不是模型动作成功指标。触觉原始图可使用 `x2 tactile-map snapshots/tactile.png` 只读导出，保留 `raw_uint8` 与时间戳，避免把未标定原始值当成 Pa/N。

## 采集格式

JSONL 首行：`record_type=metadata`、格式版本、创建时间、连接能力。其余行是 `Observation`。相机可保存 base64 数据或只保存元数据。每行写后 flush，异常退出最多损失当前行。

```bash
uv run x2 --config config/x2.yaml record recordings/vla-001.jsonl --duration 300 --rate 10 --camera rgbd_front_rgb --omit-images
```

大规模数据集建议在策略层实现分片文件和独立图像/视频存储；当前 JSONL 适合接口验证、短实验和可审计日志。
