## 1.1编译准备
当前文环境准备好之后我们就可以开始编译ROS2了。
**以下工作是在PC上进行的！！！！！！！**
由于我们采用的是docker buildx进行构建，一开始是采用分阶段构建然后导出，可能是由于文件太大了，总是在导出的时候会卡住。于是我采用了“先编译成docker镜像然后再通过临时镜像进行导出”
>
>接下来以~/目录为例进行说明
### 1. 首先在工作目录下创建工作文件夹
```bash
mkdir -p ~/ros2_riscv && cd ~/ros2_riscv
```

### 2. 在从github克隆dockerfile文件
```bash
git clone https://github.com/RLC-Lab/riscv-ros2
```

文件目录如下所示：
```bash
.
├── LICENSE
├── README.md
├── README_ZH.md
├── script
│   ├── debian
│   └── ubuntu
└── tutorials
    ├── L01-环境准备.md
    ├── L02-编译安装ros2.md
    ├── L03-运行激光SLAM算法.md
    └── L04-运行视觉SLAM算法.md

5 directories, 7 files
```
进入script文件夹后可以看到如下文件目录：
```bash
.
├── debian
│   ├── dockerfile.base
│   └── dockerfile.py.base
└── ubuntu
    ├── dockerfile.base
    └── dockerfile.py.base
```
### 3. 对dockerfile文件进行修改
>[!note]
>此时的dockerfile.base文件可以直接进行使用，但是dockerfile.py.base文件必须根据你的运行环境进行微调
>

本项目提供两个脚本，一个是基础的base版本，直接运行开箱即用。一个是指定py版本，用于解决编译环境和运行环境python版本不符并且在riscv架构上不易使用conda、pyenv、uv等工具。

尽管本项目会提供基于比较主流的系统的编译脚本，但是本项目的最大优点就是**灵活**，可以根据需求进行灵活的调整，以便于让ROS2运行在各个系统之上。

以下是一些可能需要修改的地方：
[riscv-ros2/script/debian/dockerfile.py.base ](https://github.com/RLC-Lab/riscv-ros2/blob/5f1f28ca077524d0e51b5dfff163be89c89dc09a/script/debian/dockerfile.py.base#L3)对使用镜像的版本号进行修改
[riscv-ros2/script/debian/dockerfile.py.base ](https://github.com/RLC-Lab/riscv-ros2/blob/5f1f28ca077524d0e51b5dfff163be89c89dc09a/script/debian/dockerfile.py.base#L39)对指定的python版本进行修改

```bash
# 1.编译的基础镜像环境，发行版以及版本号尽量和运行环境保持一致，如果实在不能一致大概率也不会影响运行，主要是系统以及版本号，由于riscv近年来支持才算比较好，所以可能在官方库中没有较老的版本。

ARG ARM_ARCH=riscv64
FROM ${ARM_ARCH}/debian:trixie
# 2.python版本，这个十分重要，必须必须必须必须和运行环境保持一致。

WORKDIR /tmp
RUN wget https://www.python.org/ftp/python/3.11.9/Python-3.11.9.tgz \
    && tar -xvf Python-3.11.9.tgz \
    && cd Python-3.11.9 \
    # --enable-shared: 生成 libpython3.11.so (ROS 需要)
    # --prefix=/usr: 安装到系统目录，替换系统默认 python
    && ./configure --enable-shared --prefix=/usr LDFLAGS="-Wl,-rpath=/usr/lib" \
    && make -j$(nproc) \
    && make install \
    && cd .. && rm -rf Python-3.11.9*
    
# 3.由于版本变动带来的依赖包变动问题，所有的脚本笔者都会测试，所以可能性也不大。
```


- 由于我的运行环境中python的版本是3.11，所以我在脚本中直接指定了版本，当然**也可以**在运行环境用uv，pyenv，或者是conda等环境进行运行。只是考虑到riscv的生态没有那么完善而且性能有限，所以还是比较推荐直接在脚本中指定和运行环境一致的python版本。
- 如果编译过程中出现了某些依赖问题，请根据log自行添加。
- 最后我想说明一下脚本构建中的匹配优先级：OS发行分支>OS发行版=运行环境Python版本，选择发行版也必须要和OS的**内核版本**相适配。
进行完所有的准备工作之后就可以开始进行编译工作的启动了
### 4. （可选）进行换源
由于众所周知的原因，在进行docker镜像的拉取或者apt进行包安装的时候可能会出现无法连接的情况，如果你有能力让自己的网络变得十分可用的话可以忽略此部分。

>[!note]
>由于镜像源可能与官方源的某些版本不一致，所以无法保证一定可用。
>脚本保证在官方源下是可用的，所以有条件的话我还是推荐大家使用官方源。
>

如果你出现了类似的网络问题，以下会是一些解决方案：
在dockerfile中修改apt的镜像源
```dockerfile
# Ubuntu放在 Dockerfile 的第 2 步位置
RUN sed -i 's@//ports.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list

# debian放在 Dockerfile 的第 2 步位置
RUN sed -i 's/deb.debian.org/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.debian.org/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list
```

- 在PC上修改docker hub镜像源，可能会有一些帮助。
[# 2025 最新 Docker 国内镜像源加速列表](https://zhuanlan.zhihu.com/p/24461370776)
谨供参考
- 如果你使用的是阿里云或者腾讯云的服务器进行编译的话云提供有自己的加速镜像
[配置官方镜像加速器加速拉取Docker Hub镜像-容器镜像服务-阿里云](https://help.aliyun.com/zh/acr/user-guide/accelerate-the-pulls-of-docker-official-images?spm=a2c4g.11186623.0.i7)
腾讯云[镜像加速站](https://mirror.ccs.tencentyun.com)
使用方法请参考官方教程进行使用。
## 1.2 进行编译

### 1.构建命令
```bash
 docker buildx build --platform linux/riscv64 -f dockerfile.base -t ros2_riscv_base:jazzy --load .
```
参数说明：
- `platform`:指定平台，虽然脚本中已经写明，但显式的指定更加稳妥。
- `f`:指定dockerfile文件，默认值是Dockerfile.如果你使用的是指定py的版本请使用**dockerfile.py.base**
- `t`:指定镜像名称，可以根据自己的习惯进行修改
### 2.导出文件

- 创建临时容器
```bash
docker run -d --name temp_container ros2_riscv_base:jazzy tail -f /dev/null
```
- 将构建的文件导出到pc
通过上述命令构建一个运行的临时镜像

```bash
 docker cp temp_container:/opt/ros/jazzy ./jazzy
```
进入目录后,查看目录结构，证明成功导出。
```bash
.
├── bin
├── COLCON_IGNORE
├── etc
├── include
├── lib
├── local_setup.bash
├── local_setup.ps1
├── local_setup.sh
├── _local_setup_util_ps1.py
├── _local_setup_util_sh.py
├── local_setup.zsh
├── opt
├── setup.bash
├── setup.ps1
├── setup.sh
├── setup.zsh
├── share
├── src
└── tools
```

- 停止临时镜像，然后删除临时镜像
```bash
docker stop temp_container
docker rm temp_container
```
### 3.打包文件

```bash
tar -czvf ros2_jazzy_riscv64.tar.gz -C ./jazzy .
```

### 4.传输到运行环境 && 解压安装

我采用的方法是将板子和我的电脑连到**同一个局域网**下然后通过SSH进行开发。此方法相较于有线连接各有优劣，不过采用ssh同样能够完成开发需求。也可以通过u盘进行下载。

>[!note]
>如果运行过程中出现库缺失的问题的话可以按照构建脚本中的**依赖进行全量的安装**。
## 1.3 进行安装
**以下工作是在开发板上进行的！！！！！！**
为了避免出现权限问题，建议将压缩包传输到/tmp文件夹下
```bash
# 在 RISC-V 机器上执行
cd /tmp

# 解压
tar -xzf ros2_jazzy_riscv64.tar.gz

# 安装系统依赖
sudo apt update
sudo apt install -y \
    python3 \
    python3-pip \
    python3-yaml \
    python3-argcomplete \
    libpython3.10 \
    libtinyxml2-9 \
    libyaml-0-2 \
    libssl3 \
    libboost-all-dev

# 安装到 /opt
sudo mkdir -p /opt/ros/jazzy
sudo cp -r ./ros2_jazzy_riscv64/* /opt/ros/jazzy

# 配置环境变量
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 测试安装
ros2 --help
```

非常建议安装到 `/opt/ros/{your ros2 version:humble or jazzy}`路径下和官方的版本保持一致。这样在后期的一些兼容性上会比较的友好。
## 1.4 进行测试

### 1.基本功能测试
- 发布静态变换并且进行监听
```bash
ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom  
ros2 run tf2_ros tf2_echo base_link odom  
```


```bash
ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom  
ros2 run tf2_ros tf2_monitor  
```

![PixPin_2025-12-10_14-31-52.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251210143156073.png)


### 2.分布式通信测试
- 在开发版上和PC两台设备上启动，测试通信功能
```bash
ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom  
ros2 run tf2_ros tf2_echo base_link odom  
```

```bash
ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom  
ros2 run tf2_ros tf2_monitor  
```


![PixPin_2025-12-10_14-30-38.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251210143056219.png)


![PixPin_2025-12-10_14-31-18.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251210143120637.png)

至此，ros2在riscv架构下的编译和运行已经全部完成。
