## 1.1 Basic Environment Preparation

### 1. Install Docker

Why is Docker mandatory? The official page provides an explanation. Currently, the official ROS versions only support ARM and x86 architectures; there is no official support for the RISC-V architecture. However, they provide a build strategy. This is similar to a "cross-compilation" approach, where the target architecture environment is simulated within Docker to build from source. [GitHub - ros-tooling/cross_compile: A tool to build ROS and ROS2 workspaces for various targets](https://github.com/ros-tooling/cross_compile) However, the provided script does not offer a RISC-V version, and maintenance ceased in 2022. If you aim for a stable compilation implementation, this is arguably not the best choice.
![](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251029094937448.png)

> [!note] This is not "cross-compilation" in the traditional sense; it requires using QEMU within Docker to perform the compilation.

#### Installation using the official tutorial

Regarding the Docker version, Docker is currently very stable, so it is recommended to download the latest version following the official guide. [Install | Docker Docs](https://docs.docker.com/engine/install). Simply follow the tutorial to install.

> Since relatively new Docker features are used, it is recommended that Docker version >= 17.x.

```bash
➜  ~ docker --version
Docker version 29.1.1, build 0aedba5
➜  ~ docker buildx --help
Usage:  docker buildx [OPTIONS] COMMAND
Extended build capabilities with BuildKit
Options:
      --builder string   Override the configured builder instance
  -D, --debug            Enable debug logging
Management Commands:
  history      Commands to work on build records
  imagetools   Commands to work on images in registry
Commands:
  bake         Build from a file
  build        Start a build
  create       Create a new builder instance
  dial-stdio   Proxy current stdio streams to builder instance
  du           Disk usage
  inspect      Inspect current builder instance
  ls           List builder instances
  prune        Remove build cache
  rm           Remove one or more builder instances
  stop         Stop builder instance
  use          Set the current builder instance
  version      Show buildx version information
Run 'docker buildx COMMAND --help' for more information on a command.

```

If the same prompt appears, the build setup is successful.

### 2. Enable Docker Buildx Features

```bash
# Enable buildx features
sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
sudo docker buildx create --name riscv_ros_builder --use
sudo docker buildx inspect --bootstrap
```
![PixPin_2025-11-11_16-44-45.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251111164453517.png)
### 3. Install QEMU Emulator

Since Docker Buildx relies on QEMU, installing QEMU is mandatory.

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y qemu-user-static binfmt-support
```
![PixPin_2025-11-11_16-45-13.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251111164518913.png)

After installation is complete, you can test it using the following command:

```bash
sudo docker run --rm --platform linux/riscv64 riscv64/ubuntu:22.04 uname -m
```
![PixPin_2025-11-11_16-46-15.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251111164618315.png)
If the following output appears, the configuration is correct:
```bash
riscv64
```

## 1.2 Build the Compilation Script

The official tool library provides a script previously used for compiling ARM architectures. Although it is no longer maintained (since official support for ARM renders compilation unnecessary), the script's structure remains highly valuable for reference. [GitHub - ros-tooling/cross_compile: A tool to build ROS and ROS2 workspaces for various targets](https://github.com/ros-tooling/cross_compile) Therefore, our RISC-V architecture script is basically written following the above logic. However, due to insufficient support for the RISC-V architecture in `rosdep`, we still need to manually add missing dependency libraries. This task is quite tedious, as we do not know exactly where the dependencies for RISC-V will be missing. This trial-and-error process consumed most of the time during the initial script writing.

> [!important] Since some libraries in the official repository list do not natively support the RISC-V architecture, I forked a repository and maintained it to support RISC-V.

All scripts and code required for compilation have been open-sourced on GitHub.
## 1.3 Runtime Environment Preparation

**The following operations are performed on the development board (Runtime Environment)!!!!!!!!!!!!!!!!!!** Project Runtime Environment:

- Milkv-meles
    - OS: revyos
    - RAM: 16GB
    - CPU: C910 

The **most critical** aspect to note in the runtime environment is the Python version. Since using Conda, Pyenv, or UV on a RISC-V architecture system is not very convenient, ensure you verify the development board's Python environment at the very beginning of compilation, and try to maintain consistency across major versions.

Install missing dependencies for the runtime environment. This can largely follow the environment preparation in the build script; if it compiles successfully in Docker, there are usually no issues during runtime.

```bash
# 1. Install using the apt package manager
apt-get update && \
    apt-get install -y \
    build-essential cmake git curl wget \
    # Python build dependencies (Must be complete)
    libssl-dev zlib1g-dev libncurses5-dev libncursesw5-dev \
    libreadline-dev libsqlite3-dev libgdbm-dev libdb5.3-dev libbz2-dev \
    libexpat1-dev liblzma-dev libffi-dev uuid-dev \
    # ROS 2 Core C++ libraries
    libeigen3-dev libacl1-dev libldap-dev \
    liblttng-ust-dev liblttng-ctl-dev \
    libfmt-dev libspdlog-dev libtinyxml2-dev libasio-dev \
    libxml2-dev libxslt-dev libyaml-cpp-dev \
    libcurl4-openssl-dev libzip-dev libgts-dev libbullet-dev \
    libfreetype6-dev libopencv-dev ninja-build \
    libopencv-dev \
    libpcl-dev 
     
# 2. Install Python packages using pip
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip3 install -U \
    colcon-common-extensions \
    vcstool \
    rosdepc \
    numpy \
    pyyaml==6.0.1 \
    lark==1.1.9 \
    empy==3.3.4 \
    catkin_pkg==1.0.0 \
    netifaces \
    typing_extensions \
    pytest==7.4.4
```

Please adjust the specific content above according to the actual runtime situation.
