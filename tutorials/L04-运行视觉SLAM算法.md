之前我们进行了激光slam的运行，相较于视觉slam来说，激光slam的兼容性会更高一些。所以运行视觉slam是一项十分有挑战的事情。

## 1.ORB-SLAM3
### 1.1简介
ORB-SLAM3是一个支持视觉、视觉加惯导、混合地图的SLAM系统，可以在单目，双目和RGB-D相机上利用针孔或者鱼眼模型运行。 他是第一个基于特征的紧耦合的VIO系统，仅依赖于最大后验估计(包括IMU在初始化时)。

### 1.2编译路径
整个ORB的编译安装以及和ROS2的结合的过程比较的繁琐，总的来说就是

>[!note]
>先安装pangolin依赖--->编译ORB-SLAM3算法本体--->编译SLAM和ROS2的接口

过程有点类似于给ros2外挂一个算法，所以过程中需要有一些配置文件让双方知道自己的存在。
## 2.编译ORB-SLAM3

**以下操作在运行环境进行！！！！！！！！！！！！！！！！！！！！！！！！！！**
### 2.1 安装pangolin
这是一个可视化的工具，用于将ORB-SLAM的输入和运算过程给可视化，ORB-SLAM在运算的过程中会依赖这个工具，所以在编译之前需要先编译完成
```bash
# 1.进入你的代码目录，clone代码仓库
cd ~/your_code_dir
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
# 2.用官方脚本安装依赖
./scripts/install_prerequisites.sh -m brew all

# 3.创建build目录进行编译
mkdir build && cd build 
cmake ..
make -j2
# 4.编译完成之后进行安装
sudo make install
```

### 2.2 编译ORB-SLAM算法

```bash
# 1.clone代码仓库
cd ~/your_code_dir
git clone -b c++14_cope https://github.com/jialog0301/ORB_SLAM3.git
# 2.使用官方的脚本进行构建
cd ORB_SLAM3
chmod +x ./build.sh
./build.sh
```

这个脚本会进行全自动的安装，这个编译应该不会出现什么问题，完整编译之后整个ORB-SLAM3算法就可以在开发版上运行了。接下来就是将ros2和slam算法链接起来。

	若有sophus找不到的问题，那么就到ORB-SLAM3的空间，然后安装sophus库
### 2.3编译接口文件

```bash
# 1.进入ros2安装目录的src文件夹
cd /opt/ros/{your ros2 version:humble or jazzy}/src
git clone -b humble https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
cd orbslam3_ros2
```

注意接下的**文件配置**非常的关键。
#### 配置python-site路径
在此行[ORB\_SLAM3\_ROS2/CMakeLists.txt ](https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeLists.txt#L5)配置你的python site-packages路径，一般在`/opt/ros/{your ros2 version:humble or jazzy}/lib/python3.x/`目录下，请根据实际情况进行修改
#### 配置ORB-SLAM3路径
[ORB\_SLAM3\_ROS2/CMakeModules/FindORB\_SLAM3.cmake ](https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeModules/FindORB_SLAM3.cmake#L8)将此行的路径修改为你的ORB-SLAM3算法实际路径，让ros2知道对方的存在。

#### 进入src目录进行编译

```bash
cd /opt/ros/{your ros2 version:humble or jazzy}/src
colcon build --symlink-install --packages-select orbslam3
```

>[!note]
编译完成之后一定记得source一下工作空间，否则终端可能找不到这个包。

## 3.测试算法

### 3.1 下载数据集
[kmavvisualinertialdatasets – ASL Datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)在此处下载ASL数据集

```bash
# 1.创建数据集目录
cd ~/
mkdir dataset
# 将下载好的数据集解压到dataset
unzip easy_MH_01.zip -d MH01

```

### 3.2 运行测试命令

#### 启动算法
```bash
# 1. 准备环境 (防止库找不到) 
source /opt/ros/jazzy/src/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/jazzy/src/orbslam3/lib
 # 2. 启动 Mono 节点
  # 使用 xvfb-run 防止无屏幕报错 
  # 使用 --remap 解决话题不匹配问题 
   ros2 run orbslam3 mono \ 
   /opt/ros/jazzy/src/orbslam3/vocabulary/ORBvoc.txt \    /opt/ros/jazzy/src/orbslam3/config/Monocular/EuRoC.yaml \
   --ros-args --remap /camera:=/cam0/image_raw
```
#### 播放数据

```bash
ros2 bag play ~/MH01
```



**以下操作是在PC上！！！**

#### 在RViz中进行可视化

  