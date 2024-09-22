# 使用 Ubuntu 22.04 作为基础镜像
FROM ubuntu:22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV ROS_DISTRO=humble

# 更新软件包列表并安装必要的依赖
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        locales \
        curl \
        ca-certificates \
        gnupg \
        lsb-release \
        build-essential \
        cmake \
        git \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*

# 设置语言环境
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# 启用 universe 仓库
RUN add-apt-repository universe

# 添加 ROS 2 的 APT 软件源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# 更新软件包列表并安装 ROS 2 基础包及开发工具
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-vcstool \
        libopencv-dev python3-opencv \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-transport \
        && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init

# 添加 Python 3.10 的 APT 源并安装 Python 3.10
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3.10 python3.10-venv python3.10-dev && \
    rm -rf /var/lib/apt/lists/*

# 使用 update-alternatives 设置 python3 指向 python3.10
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1

# 确保 pip 是最新版本
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

# 创建非 root 用户
RUN useradd -m rosuser

# 切换到 rosuser 用户
USER rosuser

# 运行 rosdep update 作为非 root 用户
RUN rosdep update

# 切换回 root 用户
USER root

# 安装其他 Python 依赖，并创建虚拟环境
RUN python3.10 -m venv /opt/venv

# 激活虚拟环境并安装 Flask
RUN /opt/venv/bin/pip install flask

# 设置虚拟环境的 PATH，但不覆盖系统的 ROS 环境
ENV VENV_PATH="/opt/venv/bin"
ENV PATH="${VENV_PATH}:${PATH}"

# 创建工作目录
WORKDIR /ros2_ws

# 复制本地的 src 目录到容器中
COPY ./src /ros2_ws/src

# 使用 colcon 构建工作区，确保 ROS 2 环境被正确加载
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"

# 复制入口点脚本
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# 设置入口点
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
