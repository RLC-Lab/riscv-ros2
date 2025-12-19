## 1. Choosing a SLAM Algorithm

### 1.1 Classification of SLAM

- **By Data Source**: Generally divided into **Lidar SLAM** and **Visual SLAM** algorithms.
- **By Mapping Type**: Generally categorized into **2D** and **3D** algorithms.

### 1.2 Selection Criteria

- Initially, I attempted to port **ORB-SLAM3**. While the migration was successful, the development board's limited performance prevented it from running smoothly.
- Due to these hardware constraints, I shifted focus to Lidar SLAM and chose a more conservative 2D SLAM algorithm: **Gmapping**.
- I am open-sourcing all my porting results. Those with more powerful hardware are encouraged to try running the more demanding algorithms.

---

## 2. Compiling and Running Gmapping

### 2.1 Introduction

`Gmapping` is a widely used open-source SLAM algorithm based on the filtered SLAM framework. Its core principle relies on the **RBPF (Rao-Blackwellized Particle Filter)** algorithm, which decouples the localization and mapping processes—performing localization first, followed by mapping.

Particle filtering is an early mapping technique where the robot continuously gathers environmental information through movement and observation, gradually reducing its positional uncertainty to achieve accurate localization. It uses the previous moment's map and motion model to predict the current pose, calculates weights, performs resampling, and updates the particle maps in a recursive cycle.

### 2.2 Compiling on the Development Board

#### 1. Compiling GTSAM from Source

**Note: The following operations are performed on the Development Board (Runtime Environment)!**

> [!NOTE] This is a C++ computational library, not a ROS-specific library, so it can be compiled in any directory.


```Bash

# 1. Clone the source code
git clone https://github.com/borglab/gtsam.git
# 2. Checkout the release 4.2 branch
cd gtsam
git checkout -b 4.2
```

When compiling, enable the **UNSTABLE** option, as some features are required:

```Bash
# 1. Inside the gtsam directory
mkdir build && cd build

# 2. Configure build options
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_UNSTABLE_LIBS=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_WITH_TBB=ON

# 3. Compile. Adjust thread count based on your device performance (approx. 15 mins)
make -j2

# 4. Install after compilation
sudo make install
```

#### 2. Downloading and Compiling Gmapping Source

> [!NOTE] Since this must integrate with ROS 2, it is recommended to download it into the `src` folder of your ROS installation directory for easier management.

```Bash
# 1. Download the adapted source code from the repository
cd /opt/ros/{your ros2 version:humble or jazzy}/src
git clone https://github.com/jialog0301/ros2_gmapping.git

# 2. Enter the folder and switch to the jazzy branch
cd ros2_gmapping
git checkout -b jazzy
cd ..
colcon build
```

Return to the parent directory (`src`) and use `colcon` to build. Once finished, it will appear in the ROS package list and can be called directly.

Confirm the `gmapper` package:

```Bash
# Source the workspace (add to ~/.bashrc if you don't want to repeat this)
source /opt/ros/{your ros2 version:humble or jazzy}/src/install/setup.bash
ros2 pkg list | grep gmapper
```

### 2.3 Testing

> [!IMPORTANT] Due to the development board's limited performance, all visualization and simulation are performed on the **PC**. Both devices communicate via **Distributed Communication**.

We use the official recommended method for testing: simulating **TurtleBot3** data in **Gazebo** on the PC, then communicating over the local network.

**The following process is performed on your PC!**

#### Using TurtleBot3 Simulation in Gazebo

To visualize the mapping process, follow these steps to set up the simulation:

**Environment Dependencies:** Ensure your PC has:

- **ROS 2** (Humble/Foxy, or other supported versions).
- **Gazebo** for simulation.
- **TurtleBot3** packages (`turtlebot3`, `turtlebot3_simulations`).

#### 1. Set Environment Variables

Set the TurtleBot3 model (e.g., Burger, Waffle, or Waffle_pi):



```Shell
export TURTLEBOT3_MODEL=burger    
```

#### 2. Launch Gazebo Simulation



```Shell
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This launches the Gazebo environment with the TurtleBot3 in a predefined world.

#### 3. Visualization in RViz

```Shell
ros2 launch turtlebot3_bringup rviz2.launch.py
```

In **RViz**, add the **/map** topic to visualize the occupancy grid and the **/particles** topic to see the particles. You should see the TurtleBot3 navigating and creating a map in real-time.
#### 4. Control the Robot

Launch the teleoperation node:

```Shell
ros2 run turtlebot3_teleop teleop_keyboard
```

Use the keyboard arrows to drive the robot and watch the RBPF build the map.

**The following process is performed on the Development Board!**
#### Launch SLAM Node

Start the Gmapping SLAM node on the board:

```Shell
ros2 run gmapper gmap  
```

Ensure the **TurtleBot3** (PC) and **Gmapping** (Board) nodes are communicating over the same network.

### 2.4 Test Results

#### SLAM Node Running
![PixPin_2025-11-29_22-39-40.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251129223957378.png)
#### PC-side Mapping
![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=1590168177872301768&skey=@crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D1590168177872301768%26skey%3D%40crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3%26mmweb_appid%3Dwx_webfilehelper.jpg)
#### Controlling the Robot
![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=8758044517908589535&skey=@crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D8758044517908589535%26skey%3D%40crypt_d5955b74_9a03b2f3aec116276195950e6a1cfba3%26mmweb_appid%3Dwx_webfilehelper.jpg)

---

## 3. Compiling and Running LIO-SAM

### 3.1 Introduction

**LIO-SAM** (LiDAR-Inertial Odometry via Smoothing and Mapping) is a high-efficiency, high-precision SLAM framework that features tight coupling between Lidar and IMU data. It utilizes factor graph optimization to fuse high-frequency IMU data with low-frequency LiDAR point clouds. This achieves real-time robot pose estimation and map construction with **high precision, strong robustness, and excellent real-time performance**.

### 3.2 Compiling on the Development Board

#### 1. Compiling GTSAM

_(Follow the same steps as in Section 2.2.1)_

#### 2. Downloading and Compiling LIO-SAM

The "base" version already includes the necessary packages for LIO-SAM, so you can clone and build directly.

Bash

```
cd /opt/ros/{your ros2 version:humble or jazzy}/src
git clone https://github.com/jialog0301/LIO-SAM.git
cd LIO-SAM
git checkout ros2
cd ..
colcon build 
```

### 3.3 Testing

#### Dataset Acquisition and Format Conversion

Due to privacy reasons, official ROS 2 datasets are often unavailable. You will need to use a third-party tool to convert `.bag` files. This can be done on the **PC** to save resources.



```Bash
# Install rosbags tool
pip install rosbags

# Convert the bag
rosbags-convert --src your.bag --dst ./ros2_bag
```

#### Launching the Scripts

1. **On the Development Board:**

```Bash
ros2 launch lio_sam run.launch.py
```

There are two configuration files in the `/config` directory: `params.yaml` and `robot.urdf.xacro`. These are highly dependent on your data type (e.g., Lidar model, IMU mounting).

2. **On your PC (or Board):** Play back the data:

```Bash
ros2 bag play ./ros2_bag
```

### 3.4 Test Results

#### Viewing Results in RViz
![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=7328584673507725830&skey=@crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D7328584673507725830%26skey%3D%40crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107%26mmweb_appid%3Dwx_webfilehelper.jpg)


![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=6648611987850404950&skey=@crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D6648611987850404950%26skey%3D%40crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107%26mmweb_appid%3Dwx_webfilehelper.jpg)

![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=400563685215131411&skey=@crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D400563685215131411%26skey%3D%40crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107%26mmweb_appid%3Dwx_webfilehelper.jpg)


The ROS 2 compilation and deployment on the RISC-V architecture is now complete.
