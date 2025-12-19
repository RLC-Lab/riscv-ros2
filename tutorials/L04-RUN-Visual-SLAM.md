Previously, we executed Lidar SLAM. Compared to Visual SLAM, Lidar SLAM generally offers better compatibility. Therefore, running Visual SLAM is a significantly more challenging endeavor.

## 1. ORB-SLAM3

### 1.1 Introduction

ORB-SLAM3 is a versatile SLAM system that supports Visual, Visual-Inertial (VIO), and Multi-map SLAM. It can run on Monocular, Stereo, and RGB-D cameras using either pinhole or fisheye models. It is the first feature-based, tightly-coupled VIO system that relies solely on Maximum A Posteriori (MAP) estimation (including during IMU initialization).

### 1.2 Compilation Workflow

The overall process of compiling, installing, and integrating ORB-SLAM3 with ROS 2 is quite intricate. In summary:

> [!NOTE] Install Pangolin dependencies —> Compile the ORB-SLAM3 core algorithm —> Compile the SLAM-to-ROS 2 interface.

The process is similar to "plugging in" an external algorithm to ROS 2; therefore, specific configuration files are required so that both components can communicate.

---

## 2. Compiling ORB-SLAM3

**Perform the following operations in the runtime environment (Development Board)!**

### 2.1 Installing Pangolin

Pangolin is a visualization tool used to display the input and computation process of ORB-SLAM. Since the algorithm depends on this tool for its GUI and rendering, it must be compiled first.

```Bash
# 1. Enter your code directory and clone the repository
cd ~/your_code_dir
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# 2. Install prerequisites using the official script
./scripts/install_prerequisites.sh -m brew all

# 3. Create a build directory and compile
mkdir build && cd build 
cmake ..
make -j2

# 4. Install after compilation
sudo make install
```

### 2.2 Compiling the ORB-SLAM Core


```Bash
# 1. Clone the code repository
cd ~/your_code_dir
git clone -b c++14_cope https://github.com/jialog0301/ORB_SLAM3.git

# 2. Build using the official script
cd ORB_SLAM3
chmod +x ./build.sh
./build.sh
```

This script handles the installation automatically. Under normal circumstances, this should complete without issues. Once finished, the core ORB-SLAM3 algorithm is ready for the development board. Next, we link it to ROS 2.

> [!TIP] If you encounter a "Sophus not found" error, go to the ORB-SLAM3 workspace and install the Sophus library manually.

### 2.3 Compiling the Interface Files



```Bash
# 1. Enter the src directory of your ROS 2 installation
cd /opt/ros/{your ros2 version:humble or jazzy}/src
git clone -b humble https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
cd orbslam3_ros2
```

The following **file configurations** are critical:

#### Configure the Python Site-Packages Path

In [ORB_SLAM3_ROS2/CMakeLists.txt](https://www.google.com/search?q=https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeLists.txt%23L5), configure your Python site-packages path. It is typically found under `/opt/ros/{your ros2 version}/lib/python3.x/`. Modify this according to your actual environment.
#### Configure the ORB-SLAM3 Path
In [ORB_SLAM3_ROS2/CMakeModules/FindORB_SLAM3.cmake](https://www.google.com/search?q=https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeModules/FindORB_SLAM3.cmake%23L8), change the path on this line to the actual location of your ORB-SLAM3 algorithm so ROS 2 can locate it.
#### Build the Package

```Bash
cd /opt/ros/{your ros2 version:humble or jazzy}/src
colcon build --symlink-install --packages-select orbslam3
```

> [!NOTE] Remember to `source` the workspace after compilation; otherwise, the terminal will not find the package.

---

## 3. Algorithm Testing

### 3.1 Download Dataset

Download the ASL dataset (EuRoC) from: [kmavvisualinertialdatasets – ASL Datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

```Bash
# 1. Create a dataset directory
cd ~/
mkdir dataset
# Extract the downloaded dataset to the directory
unzip easy_MH_01.zip -d MH01
```

### 3.2 Running the Test Commands

#### Start the Algorithm

```Bash
# 1. Prepare environment (to ensure libraries are found)
source /opt/ros/jazzy/src/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/jazzy/src/orbslam3/lib

# 2. Launch the Monocular node
# Use 'xvfb-run' if you are on a headless system to prevent display errors
# Use '--remap' to resolve topic mismatches
ros2 run orbslam3 mono \
/opt/ros/jazzy/src/orbslam3/vocabulary/ORBvoc.txt \
/opt/ros/jazzy/src/orbslam3/config/Monocular/EuRoC.yaml \
--ros-args --remap /camera:=/cam0/image_raw
```

#### Play Data

```Bash
ros2 bag play ~/MH01
```
