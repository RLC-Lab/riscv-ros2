## 1.1 Compilation Preparation

Before starting the ROS 2 compilation, ensure that the software and hardware environments meet the following requirements.

---

> [!IMPORTANT] Note
The following steps are performed on the PC (Development Host)!

This project uses `docker buildx` for cross-platform builds. Since building in stages and directly exporting can sometimes cause the process to hang due to large file sizes, we adopt the strategy: **"Compile into a Docker image first, then export files via a temporary container."**

> The following instructions use `~/` as the example directory.

### 1. Create a Work Directory

Create the project folder in your working directory:

```Bash
mkdir -p ~/ros2_riscv && cd ~/ros2_riscv
```

### 2. Clone the Project Code

Clone the repository from GitHub and download the Dockerfile and configuration files:

```bash
git clone https://github.com/RLC-Lab/riscv-ros2
```

The directory structure after cloning is as follows:

```bash
.
├── LICENSE
├── README.md
├── README_ZH.md
├── script
│   ├── debian
│   └── ubuntu
└── tutorials
    ├── L01-Environmental_Preparation.md
    ├── L02-Compile_and_Install_ROS2.md
    ├── L03-Run_Laser_SLAM.md
    └── L04-Run_Visual_SLAM.md
```

Entering the `script` folder, you will see build scripts for different distributions:


```bash
.
├── debian
│   ├── dockerfile.base       # Basic version
│   └── dockerfile.py.base    # Specified Python version
└── ubuntu
    ├── dockerfile.base
    └── dockerfile.py.base
```

### 3. Modify the Dockerfile (Optional)

> [!NOTE]
> 
> dockerfile.base works out of the box. However, if you use dockerfile.py.base, you must fine-tune it according to your actual runtime environment.

This project provides two types of scripts:

1. **Base Version**: A basic version that works out of the box.
    
2. **Specified Python Version**: Used to resolve inconsistencies between the compilation environment and the runtime environment's Python version, especially useful on RISC-V architectures where tools like conda, pyenv, or uv are difficult to use.
The core advantage of this project is **flexibility**, allowing adjustments to adapt to different system environments. Below are the parameters in `dockerfile.py.base` that may need modification:

- [riscv-ros2/script/debian/dockerfile.py.base](https://www.google.com/search?q=https://github.com/RLC-Lab/riscv-ros2/blob/5f1f28ca077524d0e51b5dfff163be89c89dc09a/script/debian/dockerfile.py.base%23L3): Modify the base image version.

- [riscv-ros2/script/debian/dockerfile.py.base](https://www.google.com/search?q=https://github.com/RLC-Lab/riscv-ros2/blob/5f1f28ca077524d0e51b5dfff163be89c89dc09a/script/debian/dockerfile.py.base%23L39): Modify the specified Python version.




```Dockerfile
# 1. Base image environment for compilation
# Try to keep the distribution and version consistent with the runtime environment.
# Since RISC-V support is relatively new, older versions may not be available in the official library.
ARG ARM_ARCH=riscv64
FROM ${ARM_ARCH}/debian:trixie

# 2. Python version
# This is crucial and must be strictly consistent with the target runtime environment.
WORKDIR /tmp
RUN wget https://www.python.org/ftp/python/3.11.9/Python-3.11.9.tgz \
    && tar -xvf Python-3.11.9.tgz \
    && cd Python-3.11.9 \
    # --enable-shared: Generate libpython3.11.so (Required by ROS)
    # --prefix=/usr: Install to system directory, replacing the default python
    && ./configure --enable-shared --prefix=/usr LDFLAGS="-Wl,-rpath=/usr/lib" \
    && make -j$(nproc) \
    && make install \
    && cd .. && rm -rf Python-3.11.9*
```

**Configuration Suggestions:**

- **Python Consistency**: Since the target runtime environment (RevyOS/Debian 13) likely comes with Python 3.11, it is recommended to specify this version directly in the script. While you can use uv, pyenv, or conda, keeping the system-level Python version consistent is the most stable solution given the developing RISC-V ecosystem and limited device performance.
    
- **Dependency Issues**: If missing package errors occur during compilation, please check the logs and add the necessary `apt install` dependencies manually.
    
- **Matching Priority**: OS Distribution Branch > OS Distribution Version = Runtime Python Version. Note that the selected distribution must also adapt to the target OS **kernel version**.
    

### 4. Configure Mirrors (Optional)

Due to network restrictions, pulling Docker images or installing packages via apt may fail. You can skip this step if your network environment is good.

> [!WARNING]
> 
> Third-party mirror versions may not be consistent with official sources. The scripts are guaranteed to work with official sources, so use them if conditions permit.

**Solutions:**

1. **Modify apt sources in Dockerfile**:


```  Dockerfile
   # Ubuntu: Place in Step 2 of the Dockerfile
   RUN sed -i 's@//ports.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list
   # Debian: Place in Step 2 of the Dockerfile
  RUN sed -i 's/deb.debian.org/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
       sed -i 's/security.debian.org/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list
   ```
    
2. **Configure Docker Hub Acceleration on PC**:
    
    - Reference: [2025 Latest Docker Domestic Mirror Acceleration List](https://zhuanlan.zhihu.com/p/24461370776) (Chinese Context)
        
    - Cloud Provider Acceleration (If compiling on a cloud server):
        
        - Aliyun: [Configure Official Image Accelerator](https://help.aliyun.com/zh/acr/user-guide/accelerate-the-pulls-of-docker-official-images?spm=a2c4g.11186623.0.i7)
            
        - Tencent Cloud: [Mirror Station](https://mirror.ccs.tencentyun.com/)
            

---

## 1.2 Compilation

### 1. Build Command

Execute the following command to start building:

```Bash
docker buildx build --platform linux/riscv64 -f dockerfile.base -t ros2_riscv_base:jazzy --load .
```

**Parameter Explanation:**

- `--platform`: Specifies the target platform (`linux/riscv64`).
- `-f`: Specifies the Dockerfile. If using the specific Python version script, change this to **dockerfile.py.base**.
- `-t`: Specifies the image name (Tag), which can be customized.
- `--load`: Loads the build result into the local Docker image library.

### 2. Export Files

Step A: Create a Temporary Container
Start a background container using the built image:
```Bash
docker run -d --name temp_container ros2_riscv_base:jazzy tail -f /dev/null
```

Step B: Export Build Artifacts from Container

Export the ROS 2 installation directory from the container to the local PC:
```Bash
docker cp temp_container:/opt/ros/jazzy ./jazzy
```

After exporting, check the local `jazzy` directory structure. It should contain the following:
```Bash
.
├── bin
├── COLCON_IGNORE
├── etc
├── include
├── lib
├── local_setup.bash
├── opt
├── setup.bash
├── share
├── src
└── tools
... (and other setup scripts)
```

Step C: Clean Up Temporary Resources
Stop and remove the temporary container:

```Bash
docker stop temp_container
docker rm temp_container
```

### 3. Package Files

Compress the exported directory for easy transfer:

```Bash
tar -czvf ros2_jazzy_riscv64.tar.gz -C ./jazzy .
```

### 4. Transfer to Development Board

It is recommended to connect the development board and PC to the **same LAN** and transfer files via SSH (`scp`). You can also use a USB drive.

> [!NOTE]
> 
> If missing library issues occur during subsequent runtime, refer to the dependencies in the build script and perform a full installation on the development board.

---

## 1.3 Installation

> [!IMPORTANT] Note
The following steps are performed on the Development Board (RISC-V)!

It is suggested to transfer the compressed package to the `/tmp` folder to avoid permission issues.



```Bash
# Execute on the RISC-V machine
cd /tmp

# 1. Decompress
tar -xzf ros2_jazzy_riscv64.tar.gz

# 2. Install System Dependencies (For Debian/RevyOS)
sudo apt update
sudo apt install -y \
    python3 \
    python3-pip \
    python3-yaml \
    python3-argcomplete \
    libpython3.11 \
    libtinyxml2-9 \
    libyaml-0-2 \
    libssl3 \
    libboost-all-dev
# Note: Ensure the libpython version matches your system (e.g., libpython3.11)

# 3. Install to /opt
sudo mkdir -p /opt/ros/jazzy
sudo cp -r ./ros2_jazzy_riscv64/* /opt/ros/jazzy

# 4. Configure Environment Variables
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Verify Installation
ros2 --help
```

**Recommendation:** It is strongly recommended to keep the install path as `/opt/ros/{version}` (e.g., humble or jazzy) to align with official standards and ensure compatibility.

---

## 1.4 Testing

### 1. Basic Function Test

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

At this point, the compilation and environment setup for ROS 2 on the RISC-V architecture is complete.
