# TYI E100 Flight Core

E100 Orin2 的基础固件源码和部署入口。跟随 `main` 获取部署代码，`VERSION` 指向对应的 Linux ARM64 镜像发布。

## 部署

Jetson 需要 Docker、Docker Compose v2.24 或更高版本、Git、curl 和 zstd。首次部署时填写本机的飞控串口和波特率、MAVROS 系统 ID、地面站地址、LiDAR 广播码、IP 与接雷达的网口。升级已有设备前，备份原目录的 `.env`、`machine.env` 和 `configs/livox/MID360_config.json`，并保留该机的标定参数。

```bash
git clone --branch main https://github.com/TYI-Tech/TYI_E100_Firmware.git
cd TYI_E100_Firmware
cp -n machine.env.example machine.env
```

接雷达的网口已有正确地址时，直接部署。需要初始化专用网口时，先执行 `sudo bash scripts/configure_machine.sh --skip-restart`。

```bash
bash deploy.sh --pull
bash validate_stack.sh
```

后续更新执行 `git pull --ff-only`，再运行上述部署和检查命令。部署脚本从 GitHub 下载并校验 `VERSION` 对应的镜像。

## 源码构建

在 Linux ARM64 上执行 `bash deploy.sh --build`。构建使用 `docker/runtime/Dockerfile` 和 `docker-compose.build.yml`。使用已加载的本地镜像时执行 `bash deploy.sh --local`。

录包服务独立部署。第三方许可证随各组件保留。
