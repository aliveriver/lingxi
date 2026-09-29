# 启动

## 一键检查

```bash
uv sync
uv run x2 --config config/x2.yaml doctor --sample
```

此命令只读：发现图、等待双臂/双手反馈，并从配置的第一路相机取一帧。

## 后端与机器人接口

`backend: auto` 在 PC2 优先加载 ROS 2，否则回退 mock；验收时建议明确写 `backend: ros2`，避免误把回退当实机。离线开发用 `config/mock.yaml`。

```bash
uv run x2 --config config/x2.yaml status
uv run x2 --config config/x2.yaml observe --camera rgbd_front_rgb
```

## Demo

```bash
# 单张相机图
uv run x2 --config config/x2.yaml capture rgbd_front_rgb snapshots/rgb.jpg

# 60 秒观测采集
uv run x2 --config config/x2.yaml record recordings/episode-001.jsonl --duration 60 --rate 10 --camera rgbd_front_rgb

# 固件支持触觉时加入
uv run x2 --config config/x2.yaml record recordings/episode-touch.jsonl --duration 60 --rate 10 --camera rgbd_front_rgb --tactile
```

## 前端

```bash
uv run x2 --config config/x2.yaml web --host 0.0.0.0 --port 8080
```

浏览器访问 `http://10.0.1.41:8080`。REST 接口为 `/api/status`、`/api/state`、`/api/cameras/{name}/frame`、`/api/control/arm` 和 `/api/control/hand`。

## 完整实验环境

1. 机器人按官方使用指南进入安全、稳定状态，操作员持有急停。
2. 在 PC2 运行 `doctor --sample`，保存输出。
3. 启动 Web 或策略进程之一；不要同时启动多个控制进程。
4. 只读采集成功后再按模块文档完成控制权接管。
5. 实验结束先停止策略输出，再恢复 PC1 MC；核对命令 Topic 发布者与机器人模式。

当前 v1.1.4 已完成接口与只读数据复验，但尚未完成第 4 步的现场运动验收，因此保持 `control.enabled=false`。首次运动测试优先使用 `upper_body_mc`，不要停止 MC 或直接切换到 HAL。
