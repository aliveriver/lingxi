# 安装

## 开发机

安装 `uv` 后在仓库根目录运行。仓库的 `.python-version` 会让 `uv` 自动准备 Python 3.10，无需手工建环境：

```bash
uv sync
uv run pytest
uv run x2 --config config/mock.yaml status
```

项目不使用 `requirements.txt`，也不要求手工创建 venv。

## PC2

目标实机是 Ubuntu 22.04 / aarch64 / Python 3.10 / ROS 2 Humble。把仓库放在 PC2 的 `/home/run/lingxi`，然后：

```bash
cd /home/run/lingxi
uv sync
cp config/x2.example.yaml config/x2.yaml
uv run x2 --config config/x2.yaml status
```

CLI 会检测 `/agibot/software/common` 并把其 ament、Python、动态库路径加入环境后自重启一次。这保证加载的是当前固件附带的消息包。

不要在 PC1 (`10.0.1.40`) 安装或运行项目。外部开发机应使用 Ubuntu 22.04、ROS 2 Humble 和与机器人固件严格对应的 SDK；官方建议有线直连，开发机静态 IP `10.0.1.2/24`，PC2 为 `10.0.1.41`。

## 固件升级与用户环境

本机从 v0.9.7 升级到 v1.1.4 后，`/home/run` 用户目录被重置，原项目、虚拟环境和用户级 `uv` 均不再存在。升级前应备份代码、配置、记录和用户工具；升级后重新部署，并始终从新固件的 `/agibot/software/common` 加载消息包，不能复用旧固件生成的 ROS 类型。

AimDK 1.1.0 文档提示自 1.2.0 起机上二开必须在容器中运行。当前 v1.1.4 仍按 1.1.x 宿主模式复验；以后升级应按对应版本的官方容器指南迁移，不能假设 ABI 或用户目录保留策略不变。
