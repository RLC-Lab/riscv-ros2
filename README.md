# What is riscv-ros2?

This project provides a complete set of ROS 2 build scripts supporting the RISC-V architecture, along with experimental ports of several mainstream SLAM algorithms.

It offers more than just a fixed build script; it provides extensive customizability. Users can fine-tune the project to meet specific requirements and flexibly build a usable ROS 2 environment. The project aims to maximize flexibility and portability through a script-based build approach.

## Key Features

- **Cross-Platform Support**
    - Runs on Linux, macOS, and even Windows (Docker-based).  
    - Primarily leverages `docker buildx` capabilities for building.
        
- **High Usability**
    - Optimized ROS 2 build scripts specifically for Ubuntu and Debian target platforms. 
    - Provides compilation scripts that allow specifying the Python version, ensuring a more reliable build process.
        
- **Portability and Flexibility**
    - Build script contents can be customized to tailor ROS 2 for specific devices.
    - Architecture-agnostic and does not rely on a specific system, offering excellent versatility.
        
- **ROS 2 Compatibility**
    - Capable of normal distributed communication with ROS 2 nodes running on other architectures.
        
- **SLAM Algorithm Compatibility**
    - Successfully runs the 2D LiDAR **Gmapping** algorithm and the 3D LiDAR **LIO-SAM** algorithm. 
    - Successfully ported the Visual SLAM **ORB-SLAM3** algorithm.
        

## Code

The latest version can be found in the [GitHub](https://github.com/RLC-Lab/riscv-ros2.git) repository:

```bash
https://github.com/RLC-Lab/riscv-ros2.git
```

## Usage Guide
[Set up the Environment](https://github.com/RLC-Lab/riscv-ros2/blob/main/tutorials/L01-setup.md) 

[Build ROS2](https://github.com/RLC-Lab/riscv-ros2/blob/main/tutorials/L02-build.md)

[Run LiDAR SLAM on ROS2](https://github.com/RLC-Lab/riscv-ros2/blob/main/tutorials/L03-Run-LiDAR-SLAM.md)

[Run Visual SLAM on ROS2](https://github.com/RLC-Lab/riscv-ros2/blob/main/tutorials/L04-RUN-Visual-SLAM.md)

## Using the Docker Image
Pull the image:

```bash
docker pull jialog/ros2_riscv_base:humble or jazzy
```
You can run the image directly, or follow the [guide](https://github.com/RLC-Lab/riscv-ros2/blob/main/tutorials/L02-build.md#2-export-files) to export the compiled binaries.

## Development & Test Environment

Ensure your setup meets these minimum specifications to build and run the project:

* **Host Workstation:**
  * **CPU:** No specific requirement.
  * **RAM:** At least 16GB.
* **Target Device:**
  * **RAM:** At least 4GB.

The project is developed and tested using the following setup:

* **Host Workstation:**
  * **CPU:** AMD Ryzen 9 9950X
  * **RAM：** 128GB
  * **OS:** Ubuntu 24.04 LTS
* **Target Device:**
  * **Hardware:** Milk-V Meles (RISC-V SOC TH1520)
  * **RAM：** 16GB
  * **OS:** RevyOS (based on Debian 13 Trixie)
* **Network:** Both devices are connected to the same Local Area Network (LAN) via Wi-Fi.


## Verification & Demos

- ros2 basic test
Test TF2 static transform publishing and listening.

**Terminal 1 (Publisher):**

```Bash
ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom
```

**Terminal 2 (Listener):**

```Bash
# Method A: Print transform info
ros2 run tf2_ros tf2_echo base_link odom

# Method B: Monitor frequency
ros2 run tf2_ros tf2_monitor
```
![PixPin_2025-12-10_14-31-52.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251210143156073.png)

### 2. Distributed Communication Test

Run nodes on both the development board and the PC to test cross-device communication.

**Device A (e.g., Dev Board):**

```Bash
ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom
```
![PixPin_2025-12-10_14-30-38.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251210143056219.png)

**Device B (e.g., PC):**

```Bash
ros2 run tf2_ros tf2_echo base_link odom
# Or
ros2 run tf2_ros tf2_monitor
```
![PixPin_2025-12-10_14-31-18.png|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251210143120637.png)


- SLAM Algorithms
**Gmapping**:
![](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/20251211165254338.png)

**LIO-SAM:**
![_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=6648611987850404950&skey=@crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107&mmweb_appid=wx_webfilehelper.jpg|650](https://weijiale.oss-cn-shanghai.aliyuncs.com/picgo/_cgi-bin_mmwebwx-bin_webwxgetmsgimg__%26MsgID%3D6648611987850404950%26skey%3D%40crypt_d5955b74_1ac07951e10d1b5e8d26f3183e912107%26mmweb_appid%3Dwx_webfilehelper.jpg)


## TODO List
- [x] 完成 RISC-V 基础环境构建脚本 (Docker)
- [x] 移植 Gmapping 激光建图算法 
- [x] 移植 LIO-SAM 3D 激光 SLAM 
- [ ] 优化在RISCV架构上的性能
	- [ ] 指令级层面的并行优化
	- [ ] 中间件与通信的优化
	- [ ] 操作系统与调度优化
	- [ ] 编译工具链优化
