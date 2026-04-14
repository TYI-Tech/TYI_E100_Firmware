# TYI UAV Firmware

该仓库是 UAV-NX ROS1 基础栈的客户交付版本。
客户可以直接查看源码，并通过 `docker compose` 完成构建和运行。

英文版：[README.md](README.md)

当前版本：[VERSION](VERSION)

## 包含模块

- `livox_ros_driver2`
- `dlio`
- `fastlio_to_mavros`
- `mavros`
- `mavlink`
- `uav_base_bringup`

## 客户使用流程

1. 按机型修改 [machine.env](machine.env)。
2. 执行 `bash ./scripts/deploy.sh`。
3. 通过 `bash ./scripts/status.sh` 确认容器状态。
4. 日常运行时可使用 `bash ./scripts/logs.sh` 或 `bash ./scripts/enter.sh`。

该目录只保留面向客户的部署与运维入口。
内部烟测和研发调试脚本不会在这里保留。

## 快捷入口

- [英文快速上手](docs/en_US/quick_start.md)
- [中文快速上手](docs/zh_CN/快速上手.md)
- [英文文档索引](docs/en_US/README.md)
- [中文文档索引](docs/zh_CN/README.md)
- [更新记录](CHANGELOG.md)

## 可选控制桥接包

如果后续需要控制桥接能力，可单独安装 `TYI_Plugin_Ctl`：

```bash
sudo apt install tyi-plugin-ctl
```
