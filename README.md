# TYI E100 固件

面向 `TYI E100` 产品发布的 ROS1 基础固件仓库。
仓库内提供完整的可见源码与运行入口，可通过 `docker compose` 完成构建、部署与运行。

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
git clone git@github.com:TYI-Tech/TYI_UAV_Firmware.git
cd TYI_UAV_Firmware
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
- [文档总索引](docs/README.md)
- [English quick start](docs/en_US/quick_start.md)
- [English documentation](docs/en_US/README.md)
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
