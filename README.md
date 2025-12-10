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

The latest version can be found in the [GitHub](https://www.google.com/search?q=https://github.com/jialog0301/RVROS2.git) repository:



```bash
https://github.com/RLC-Lab/riscv-ros2.git
```

## Usage Guide