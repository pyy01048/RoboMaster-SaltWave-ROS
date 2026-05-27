# RoboMaster哨兵机器人导航系统

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](https://opensource.org/licenses/Apache-2.0)

一个基于ROS2 Humble的高性能哨兵机器人自主导航系统，专为RoboMaster竞赛设计。

---

## 📖 目录

- [系统特性](#系统特性)
- [系统架构](#系统架构)
- [硬件要求](#硬件要求)
- [软件要求](#软件要求)
- [快速开始](#快速开始)
- [功能模块](#功能模块)
- [参数配置](#参数配置)
- [常见问题](#常见问题)

---

## 🎯 系统特性

### 核心能力

| 特性 | 性能指标 | 说明 |
|------|----------|------|
| **定位精度** | ±10cm | 多传感器EKF融合 |
| **融合延迟** | <50ms | 200Hz实时融合 |
| **自瞄延迟** | <50ms | 端到端瞄准系统 |
| **安全距离** | 0.5m | CBF安全过滤保证 |
| **导航速度** | 1.5m/s | 麦克纳姆轮全向移动 |

### 技术亮点

- ✅ **多速率传感器融合**：IMU(500Hz) + LiDAR(100Hz) + 视觉(60Hz)
- ✅ **CBF安全过滤**：基于控制屏障函数的碰撞避免
- ✅ **行为树决策**：ReactiveFallback实现战术切换
- ✅ **TEB全向规划**：麦克纳姆轮优化局部规划器
- ✅ **模块化视觉管道**：检测→跟踪→弹道→云台

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    哨兵机器人导航系统                         │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│ 传感器融合层  │    │  决策规划层  │    │  执行控制层  │
└──────────────┘    └──────────────┘    └──────────────┘
        │                     │                     │
        ├─ sensor_fusion      ├─ decision_making    ├─ sentinel_navigation
        │  ├─ IMU预滤波        │  ├─ 行为树决策      │  ├─ CBF安全过滤
        │  ├─ LiDAR ICP       │  └─ 状态管理        │  ├─ Nav2集成
        │  ├─ 视觉PnP         │                      │  └─ 巡逻管理
        │  └─ EKF融合         │                      │
        │                     │                     │
        └─────────────────────┴─────────────────────┘
                              │
                              ▼
                    ┌──────────────┐
                    │  视觉瞄准层  │
                    └──────────────┘
```

---

## 🔧 硬件要求

| 传感器 | 型号 | 用途 | 接口 |
|--------|------|------|------|
| **LiDAR** | Livox Mid-360 | 定位与建图 | 以太网 |
| **IMU** | 达妙IM1R | 姿态估计 | USB串口 |
| **相机** | 海康威视 | 自瞄检测 | USB/GigE |
| **底盘** | 麦克纳姆轮 | 全向移动 | CAN总线 |

**推荐计算平台：** NVIDIA Jetson Xavier NX / Orin NX

---

## 💻 软件要求

### 操作系统

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill

### 核心依赖

```bash
# ROS2核心
sudo apt-get install ros-humble-desktop-full

# 导航栈
sudo apt-get install ros-humble-nav2-bringup ros-humble-navigation2

# TEB规划器
sudo apt-get install ros-humble-teb-local-planner

# 行为树
sudo apt-get install ros-humble-behaviortree-cpp-v3

# 其他依赖
sudo apt-get install libeigen3-dev libpcl-dev
pip3 install opencv-python numpy pyyaml
```

---

## 🚀 快速开始

### 1. 克隆仓库

```bash
cd ~/Documents
git clone https://github.com/pyy01048/RoboMaster-SaltWave-ROS.git
cd RoboMaster-SaltWave-ROS
```

### 2. 一键编译

```bash
chmod +x build.sh
./build.sh
```

### 3. 启动传感器

**终端1: Mid360 LiDAR**
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端2: FAST-LIO2**
```bash
ros2 launch fast_lio mapping_mid360.launch.py
```

**终端3: IMU**
```bash
ros2 run im1r_ros2_driver im1r_node --ros-args -p serial_port:=/dev/ttyUSB0
```

### 4. 启动导航系统

```bash
source install/setup.bash
ros2 launch sentinel_bringup bringup_all.launch.py
```

### 5. 发送导航目标

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"
```

---

## 📦 功能模块

### sensor_fusion (传感器融合)

**话题：**
- 输入：`/imu/data`, `/fastlio/odom`
- 输出：`/odom_filtered`

### sentinel_navigation (导航控制)

**节点：**
- `velocity_safety_cbf` - CBF安全过滤器
- `patrol_waypoints` - 巡逻路径管理器

### vision_aiming (视觉瞄准)

**流程：** 检测(60Hz) → 跟踪(60Hz) → 弹道(100Hz) → 云台(100Hz)

### decision_making (决策系统)

**行为树：** 敌人可见 → 交战 | 否则 → 巡逻

---

## ⚙️ 参数配置

### EKF融合参数

```yaml
ekf:
  process_noise_pos: 0.01      # 位置过程噪声
  process_noise_theta: 0.001   # 角度过程噪声
  lidar_noise_pos: 0.05        # LiDAR观测噪声
```

### CBF安全参数

```yaml
safe_distance: 0.5          # 安全距离阈值
critical_distance: 0.3      # 临界距离
cbf_alpha: 2.0              # CBF衰减系数
```

### TEB规划参数（麦克纳姆轮）

```yaml
min_turning_radius: 0.0     # 允许原地旋转
holonomic_robot: true       # 全向移动
weight_kinematics_nh: 50.0  # 降低非完整约束
```

---

## 🔍 常见问题

### Q: 编译失败，找不到PCL

**A:** 安装PCL依赖：
```bash
sudo apt-get install ros-humble-pcl-ros ros-humble-perception-pcl
```

### Q: 定位漂移严重

**A:** 降低EKF过程噪声：
```yaml
ekf.process_noise_pos: 0.005
```

### Q: CBF过滤器过度减速

**A:** 调整安全距离：
```yaml
safe_distance: 0.4
cbf_alpha: 1.5
```

---

## 📄 许可证

本项目采用 Apache 2.0 许可证

---

## 👥 作者

**pyy00707** - *初始工作* - [GitHub](https://github.com/pyy01048)

---

**最后更新：** 2026-05-27
