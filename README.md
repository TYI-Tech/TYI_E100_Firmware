<p align="center">
  <img src="assets/logo.png" alt="TYI Innovation" width="260">
</p>

# TYI E100 固件

面向 `TYI E100` 产品发布的 ROS1 基础固件仓库。
仓库内提供完整的可见源码与运行入口，可通过 `docker compose` 完成构建、部署与运行。
当前版本已经完成实机 fresh build 验证，支持从源码直接构建运行镜像。

English version: [README_EN.md](README_EN.md)

当前版本：[VERSION](VERSION)

## 本仓库提供的内容

- 面向 Ubuntu 20.04 + ROS1 Noetic 的可见源码部署包
- 通过 [machine.env](machine.env) 收敛机型差异配置
- 基于 Docker 的 `livox_ros_driver2`、`dlio`、`fastlio_to_mavros`、`mavros`、`mavlink`、`uav_base_bringup` 启动能力
- 面向产品固件发布的部署与运维脚本

## 运行链路

`Livox MID360 -> DLIO -> fastlio_to_mavros -> MAVROS -> PX4`

## 快速开始

```bash
git clone git@github.com:TYI-Tech/TYI_E100_Firmware.git
cd TYI_E100_Firmware
bash ./scripts/check_host.sh
vim machine.env
bash ./scripts/deploy.sh
```

部署完成后：

```bash
bash ./scripts/status.sh
bash ./scripts/logs.sh
bash ./scripts/enter.sh
```

## 最短路径

如果只想尽快完成首次部署，按下面顺序操作即可：

1. 拉取仓库并执行 `bash ./scripts/check_host.sh`
2. 只修改 [machine.env](machine.env)
3. 执行 `bash ./scripts/deploy.sh`
4. 用 `bash ./scripts/status.sh` 和 `bash ./scripts/logs.sh` 确认运行状态

说明：

- `deploy.sh` 会直接基于当前仓库源码构建镜像，不依赖外部隐藏源码包
- 构建阶段已内置 GeographicLib 关键 geoid 资源，并对基础 `apt` 安装增加重试处理
- 机型差异默认通过 `machine.env` 收敛，不需要手动改动多个配置文件

## 常用配置入口

- [machine.env](machine.env)
  机型 UART、MID360 序列号/IP、宿主机网卡设置
- [configs/fastlio_to_mavros/bridge.yaml](configs/fastlio_to_mavros/bridge.yaml)
  后续如需接入控制桥接，可在此调整桥接话题与参考坐标系
- [configs/dlio](configs/dlio)
  DLIO 运行参数
- [configs/mavros](configs/mavros)
  MAVROS 插件与 FCU 参数

## 建议先看

- [中文快速上手](docs/zh_CN/快速上手.md)
- [中文文档索引](docs/zh_CN/README.md)
- [中文版本说明](docs/zh_CN/版本说明.md)
- [文档总索引](docs/README.md)
- [English quick start](docs/en_US/quick_start.md)
- [English documentation](docs/en_US/README.md)
- [English release notes](docs/en_US/release_notes.md)
- [更新记录](CHANGELOG.md)

## 仓库结构

- `configs/`
  挂载进容器的运行配置
- `docker/`
  Docker 构建与运行入口
- `scripts/`
  固件运维脚本
- `third_party/`
  构建所需的内置第三方依赖
- `workspace/src/`
  运行时使用的 ROS 源码包

## 可选控制桥接包

如果后续需要控制桥接能力，可单独安装 `TYI_Plugin_Ctl`：

```bash
sudo apt install tyi-plugin-ctl
```

## 仓库定位

该仓库用于产品固件发布、部署与现场运行。
内部烟测入口和研发专用调试入口不会放在这里。
