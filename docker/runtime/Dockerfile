FROM ros:noetic-ros-base-focal

ARG UBUNTU_PORTS_MIRROR=http://ports.ubuntu.com/ubuntu-ports
ARG ROS_APT_MIRROR=http://snapshots.ros.org/noetic/final/ubuntu

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

COPY docker/runtime/ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg

RUN printf 'Acquire::Retries "5";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\n' >/etc/apt/apt.conf.d/80-uav-retries \
  && sed -i "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" /etc/apt/sources.list \
  && sed -i "s|signed-by=/usr/share/keyrings/ros1-snapshots-archive-keyring.gpg|signed-by=/usr/share/keyrings/ros-archive-keyring.gpg|g" /etc/apt/sources.list.d/ros1-snapshots.list \
  && sed -i "s|http://snapshots.ros.org/noetic/final/ubuntu|${ROS_APT_MIRROR}|g" /etc/apt/sources.list.d/ros1-snapshots.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    libapr1-dev \
    sudo \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    tmux \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/uav/base_stack

COPY third_party /opt/uav/base_stack/third_party
COPY workspace /opt/uav/base_stack/workspace
COPY configs /opt/uav/configs
COPY docker/runtime /opt/uav/base_stack/docker/runtime

RUN chmod +x /opt/uav/base_stack/docker/runtime/*.sh \
  && /opt/uav/base_stack/docker/runtime/build_workspace.sh

RUN groupadd -g 1000 uav \
  && useradd -m -u 1000 -g 1000 -s /bin/bash uav \
  && dialout_group="$(getent group 20 | cut -d: -f1 || true)" \
  && video_group="$(getent group 44 | cut -d: -f1 || true)" \
  && plugdev_group="$(getent group 46 | cut -d: -f1 || true)" \
  && render_group="$(getent group 103 | cut -d: -f1 || true)" \
  && i2c_group="$(getent group 113 | cut -d: -f1 || true)" \
  && gpio_group="$(getent group 999 | cut -d: -f1 || true)" \
  && if [ -z "${dialout_group}" ]; then groupadd -g 20 dialout; dialout_group=dialout; fi \
  && if [ -z "${video_group}" ]; then groupadd -g 44 video; video_group=video; fi \
  && if [ -z "${plugdev_group}" ]; then groupadd -g 46 plugdev; plugdev_group=plugdev; fi \
  && if [ -z "${render_group}" ]; then groupadd -g 103 render; render_group=render; fi \
  && if [ -z "${i2c_group}" ]; then groupadd -g 113 i2c; i2c_group=i2c; fi \
  && if [ -z "${gpio_group}" ]; then groupadd -g 999 gpio; gpio_group=gpio; fi \
  && usermod -aG sudo,"${dialout_group}","${video_group}","${plugdev_group}","${render_group}","${i2c_group}","${gpio_group}" uav \
  && echo 'uav:tyitech' | chpasswd \
  && printf 'uav ALL=(ALL) ALL\n' >/etc/sudoers.d/90-uav \
  && chmod 0440 /etc/sudoers.d/90-uav \
  && chown -R uav:uav /opt/uav /home/uav \
  && printf '%s\n' \
      'export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:${ROS_MASTER_PORT:-11311}}"' \
      'export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"' \
      'export UAV_CONFIG_DIR="${UAV_CONFIG_DIR:-/opt/uav/configs}"' \
      'export ROS_LOG_DIR="${ROS_LOG_DIR:-/opt/uav/logs/ros}"' \
      'source /opt/ros/noetic/setup.bash' \
      'if [ -f /opt/uav/base_stack/workspace/install/setup.bash ]; then source /opt/uav/base_stack/workspace/install/setup.bash; fi' \
      'if [ -f /opt/uav/base_stack/workspace/install_isolated/setup.bash ]; then source /opt/uav/base_stack/workspace/install_isolated/setup.bash; fi' \
      'cd /opt/uav/base_stack' \
      >>/home/uav/.bashrc

ENV UAV_CONFIG_DIR=/opt/uav/configs
ENV ROS_LOG_DIR=/opt/uav/logs/ros
ENV HOME=/home/uav
ENV USER=uav

WORKDIR /home/uav

HEALTHCHECK --interval=30s --timeout=10s --start-period=45s --retries=5 \
  CMD /opt/uav/base_stack/docker/runtime/healthcheck.sh

USER uav

ENTRYPOINT ["/opt/uav/base_stack/docker/runtime/entrypoint.sh"]
