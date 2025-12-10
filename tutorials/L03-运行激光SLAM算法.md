
## 1.SLAM算法的选择
### 1.1 SLAM分类的分类
- 根据数据的来源可以大致分为激光SLAM和视觉SLAM算法
- 根据建图类型可以大致的分为2D和3D算法
### 1.2 SLAM算法选择的依据
- 一开始我是准备直接移植ORB-SLAM3，确实也移植成功了，但是开发板性能有限，不能正常的运行
- 之后考虑到开发版性能的问题开始着手移植激光slam算法，我选择了比较保守的2D slam算法：**Gmapping**
- 我会将我的移植成果都开源出来，如果有性能更强劲的硬件可以尝试着运行一下。

## 2.Gmapping算法的编译与运行

### 2.1 简介
`Gmapping`是基于滤波`SLAM`框架的常用开源`SLAM`算法。其基本原理基于`RBpf`粒子滤波算法，即将定位和建图过程分离，先进行定位再进行建图。粒子滤波算法是早期的一种建图算法，其基本原理为机器人不断地通过运动、观测的方式，获取周围环境信息，逐步降低自身位置的不确定度，最终得到准确的定位结果。用上一时刻的地图和运动模型预测当前时刻的位姿，然后计算权重，重采样，更新粒子的地图，如此往复。  
### 2.2 在开发板上进行编译

#### 1.从源码编译GTSAM

**以下操作是在开发版（运行环境）上进行的！！！！！！！！！！！！！！！！！！**

>[!note]
>此库为计算CPP库，非ROS库，所以在什么目录编译都可以


```bash
#1.clone源码
git clone https://github.com/borglab/gtsam.git
#2.进入release4.2分支
cd gtsam
git checkout -b 4.2

```

在编译的时候选择打开**UNSTABLE**选项，有一些特性会被使用到

```bash
# 1.在gtsam中
mkdir build && cd build

# 2.对编译选项进行选择
cmake .. \ -DCMAKE_BUILD_TYPE=Release \ -DGTSAM_BUILD_UNSTABLE_LIBS=ON \ -DGTSAM_USE_SYSTEM_EIGEN=ON \ -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \ -DGTSAM_WITH_TBB=ON

# 3.进行编译。根据设备性能自行调节线程数，大约15分钟左右
make -j2

# 4.编译完成之后进行安装
sudo make install
```

#### 2.下载Gmapping源码进行编译

>[!note]
>此文件要与ROS2结合，所以尽量下载在ros安装目录下的src文件中便于管理
>

```bash
# 1.从仓库下载适配后的源码
cd /opt/ros/{your ros2 version:humble or jazzy}/src
git clone https://github.com/jialog0301/ros2_gmapping.git
# 2.进入文件夹切换至jazzy分支
cd ros2_gmapping
git checkout -b jazzy
cd ..
colcon build
```

然后回到上级目录（src目录）使用colcon 进行构建，构建完成之后会出现在ros可用包的列表，可以直接进行调用。


使用以下命令确认gmapper
```bash
# 为新编译的包设置环境变量，如果嫌每次打开一个新的端口都要输入的话可以写进~/.bashrc中
source /opt/ros/{your ros2 version:humble or jazzy}/src/install/setup.bash
ros2 pkg list | grep gmapper
```
### 2.3 进行测试

>[!important]
>由于开发版性能有限，所有可视化和仿真都在PC进行，两者使用分布式通信。

使用官方推荐的方法进行测试，在Gazebo使用TurtleBot3进行小车数据的仿真，然后通过局域网进行通信。

**以下过程都是在你的PC上进行的！！！！！！！！！！！！！！！！！！！！！！！**
#### 如何使用TurtleBot3 Simulation 在Gazebo中

要使用此 **ROS 2** 实现可视化映射过程，您可以在 **Gazebo** 中模拟 **TurtleBot3**。以下是设置和使用模拟环境的分步指南:
#### 环境依赖
确认你的PC上拥有以下环境，
- **ROS 2** (Humble/Foxy, 或者其他一些支持的版本).
- **Gazebo** 用于模拟.
- **TurtleBot3** 包 (`turtlebot3`, `turtlebot3_simulations`).
#### 1.设置环境变量

首先为**TurtleBot3**设置环境变量，在你的终端中运行以下命令：
```shell
export TURTLEBOT3_MODEL=burger    
```
这是将**TurtleBot3**模型参数设置为**Burger**.你也可以根据需要将参数换成waffle或者waffle_pi如果有需要的话
#### 2.启动Gazebo模拟
运行以下命令在**Gazebo**中启动**TurtleBot3**模拟。
```shell
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

此命令将启动 **Gazebo** 环境，并将 **TurtleBot3** 放置在预定义的设置中

#### 3.在RViz中进行可视化
启动Rviz用于机器人的可视化以及建图的过程
```shell
ros2 launch turtlebot3_bringup rviz2.launch.py
```
在 **RViz** 中，确保添加 **/map** 主题以可视化占用网格，并添加 **/particles** 主题以查看粒子。您应该看到 **TurtleBot3** 实时导航并创建环境地图，并显示粒子以说明定位不确定性。
#### 4.控制机器人 
使用以下命令启动控制窗口
```shell
ros2 run turtlebot3_teleop teleop_keyboard
```

使用键盘的方向键驱动机器人四处走动，并观看 **RBPF** 构建地图并跟踪机器人的姿势。

**以下过程在开发版上！！！！！！！！！！！！！！！！！！！！！！！！！！！！**
#### 启动SLAM节点
开发板上启动Gmapping SLAM节点
```shell
ros2 run gmapper gmap  
```

确保 **TurtleBot3** 和 **Gmapping** 节点通过同一网络进行通信。

### 2.4测试结果

#### SLAM node运行
![PixPin_2025-11-29_22-39-40.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251129223957378.png)

#### PC端建图
![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=1590168177872301768&skey=@crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D1590168177872301768%26skey%3D%40crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3%26mmweb_appid%3Dwx_webfilehelper.jpg)

#### 控制小车移动
![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=8758044517908589535&skey=@crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D8758044517908589535%26skey%3D%40crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3%26mmweb_appid%3Dwx_webfilehelper.jpg)

## 3.LIO_SAM算法的编译与运行

### 3.1.简介
LIO-SAM ([LiDAR-Inertial Odometry via Smoothing and Mapping](https://www.google.com/search?q=LiDAR-Inertial+Odometry+via+Smoothing+and+Mapping&sca_esv=4bcba3fdc8f587fd&sxsrf=AE3TifMweojvmBrz_JL4GPcXPrVLnxt-TQ%3A1765013579072&ei=S_gzabGRBMnZ1e8P6f-68QQ&ved=2ahUKEwifnZ3906iRAxUVoa8BHacFABkQgK4QegQIARAB&uact=5&oq=lio_sam%E7%AE%80%E4%BB%8B&gs_lp=Egxnd3Mtd2l6LXNlcnAiDWxpb19zYW3nroDku4syBRAAGO8FMgUQABjvBTIFEAAY7wUyCBAAGKIEGIkFMggQABiiBBiJBUjRE1CvCViYD3ABeAGQAQCYAeQBoAG7DKoBBTAuNi4yuAEDyAEA-AEBmAIJoALbDMICChAAGLADGNYEGEfCAgcQIxiwAhgnwgIIEAAYExgNGB7CAgoQABgTGAoYDRgemAMAiAYBkAYJkgcFMS42LjKgB6QXsgcFMC42LjK4B9MMwgcFMC43LjLIBxWACAA&sclient=gws-wiz-serp&mstk=AUtExfCvO7iDX1sZ0B9lVJIgF9-juWjlY0eppFseTJ7m9zOrEa9x3m30IxszbMiP1-pUNJRAMVsaS0Korb3P_l6rqHqFXXvhJhGpPttYDqrE6hdzisC5qvg8HFTv8Wf2DDAnExEJ1Obd_evopqwXj1oXZ1BKB5Y3v45KpSB480OmkOcVq5fhhy15rW7obv-RxVk5r5BTYXas3GUSK-pHf9zIr0pkRcbd_t5wT9lXrTwdGB7CM_SvD6DHfpvQ_xfnh2fy-d6FqabjD1tsiaGecohwFuo92Lagoo6KUBJqmQHgApGdn4tpbWYPPf3ovMJBsH_f0jntjsY4tXPfosvR3FFc7eToFOLTzv5jvntlpE2v5INeU25gSJI7R8VtVItCa0YZS-hckDNdIQmuJQtaQj6cNA&csui=3)) 是一种由 [GitHub - TixiaoShan/LIO-SAM: LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping](https://github.com/TixiaoShan/LIO-SAM) 提出的、高效且高精度的激光雷达与惯性测量单元 (IMU) 紧耦合的SLAM框架，它利用因子图优化技术，将高频率的IMU数据和低频率的LiDAR点云数据融合，实现了实时的机器人姿态估计与地图构建，具有**高精度、鲁棒性强、实时性好**等特点，广泛应用于无人驾驶和机器人导航领域.

### 3.2 在开发版上进行编译
#### 1.从源码编译GTSAM

**以下操作是在开发版（运行环境）上进行的！！！！！！！！！！！！！！！！！！**

>[!note]
>此库为计算CPP库，非ROS库，所以在什么目录编译都可以


```bash
#1.clone源码
git clone https://github.com/borglab/gtsam.git
#2.进入release4.2分支
cd gtsam
git checkout -b 4.2

```

在编译的时候选择打开**UNSTABLE**选项，有一些特性会被使用到

```bash
# 1.在gtsam中
mkdir build && cd build

# 2.对编译选项进行选择
cmake .. \ -DCMAKE_BUILD_TYPE=Release \ -DGTSAM_BUILD_UNSTABLE_LIBS=ON \ -DGTSAM_USE_SYSTEM_EIGEN=ON \ -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \ -DGTSAM_WITH_TBB=ON

# 3.进行编译。根据设备性能自行调节线程数，大约15分钟左右
make -j2

# 4.编译完成之后进行安装
sudo make install
```

#### 2.下载LIO_SAM进行编译
base版本中已经包含了构建LIO_SAM所需要的包，所以可以直接clone源码进行构建。

```bash
cd /opt/ros/{your ros2 version:humble or jazzy}/src
git clone https://github.com/jialog0301/LIO-SAM.git
cd LIO-SAM
git checkout ros2
cd ..
colcon build 
```


### 3.3 进行测试

#### 获取数据集进行包格式的转换
由于隐私原因，官方没有提供可供ROS2使用的数据集，所以需要使用第三方的包转换工具进行转换，这些工作都可以放在PC上进行，所以整体来说压力也不算大。

```bash
# 安装rosbags工具
pip install rosbags

# 使用工具进行转换
rosbags-convert --src your.bag --dst ./ros2_bag

```

#### 启动运行脚本
1.在你的开发板上启动
```bash
ros2 launch lio_sam run.launch.py
```

在源文件的`/config`目录下有两个配置文件，[LIO-SAM/config/params.yaml ](https://github.com/jialog0301/LIO-SAM/blob/ros2/config/params.yaml)[LIO-SAM/config/robot.urdf.xacro ](https://github.com/jialog0301/LIO-SAM/blob/ros2/config/robot.urdf.xacro)这两个配置文件和数据的类型高度绑定，具体的配置细节可以搜索学习进行了解。

2.在你的PC上或者开发版上进行播报
```bash
ros2 bag play ./ros2_bag
```

### 3.4 测试结果

#### 在RViz中查看结果


![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=7328584673507725830&skey=@crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D7328584673507725830%26skey%3D%40crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107%26mmweb_appid%3Dwx_webfilehelper.jpg)



![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=6648611987850404950&skey=@crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D6648611987850404950%26skey%3D%40crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107%26mmweb_appid%3Dwx_webfilehelper.jpg)

![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=400563685215131411&skey=@crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D400563685215131411%26skey%3D%40crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107%26mmweb_appid%3Dwx_webfilehelper.jpg)
