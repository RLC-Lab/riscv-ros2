# riscv-ros2 是什么？

本项目的主要内容是一套完整的支持riscv的ROS2构建脚本，同时也尝试性的移植了一些主流的SLAM算法。

它不仅仅提供了一个固定的构建脚本，同时也提供了客制化的可能，使用者可以根据自己的某些特殊需求对项目进行微调灵活的构建一个可用的ROS2.通过构建脚本的形式最大限度的提升灵活性和可移植性。

## 主要特性

- 跨平台运行
	- 可以在Linux，macOS，甚至是Windows上运行（基于docker）
	- 主要利用到docker buildx功能进行构建
- 极高的可用性
	- 对于目标平台是ubuntu和debian的ros2构建脚本进行了优化
	- 同时提供了可以指定python版本的编译脚本，让编译更可用。
- 可移植性和灵活性
	- 构建脚本中的内容可以根据需求进行客制化，为自己的设备定制ROS2
	- 不依赖于特定的系统或者架构，具有不错的通用性。
- 兼容其他版本ROS2
	- 可以通过分布式通信和其他架构的ROS2进行正常的通信
- 兼容常用的SLAM算法
	- 成功运行2D激光Gmapping算法，3D激光LIO-SAM算法
	- 成功移植视觉ORB-SLAM3算法。
## Code

可以在[GitHub ](https://github.com/jialog0301/RVROS2.git)仓库找到最新的版本：
```bash
https://github.com/RLC-Lab/riscv-ros2.git
```

## 使用指南

