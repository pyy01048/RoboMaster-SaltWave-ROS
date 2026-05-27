# 哨兵机器人导航项目技术设计文档

| 文档版本 | v1.0 |
|---------|------|
| 编制单位 | SaltWave 战队技术部 |
| 编制日期 | 2026年5月26日 |
| 适用赛事 | RMUL 3V3 / RMUC 超级对抗赛 |
| 开发环境 | WSL2 Ubuntu22.04 + ROS2 Humble |

---

## 一、文档信息与背景

### 1.1 文档信息

| 项目名称 | 哨兵机器人自主导航系统 |
|---------|---------------------|
| 文档类型 | 技术设计文档 (TDD) |
| 目标读者 | 开发团队、技术评审人员 |
| 相关文档 | RoboMaster机器人导航项目方案.md、RoboMaster哨兵机器人上位机专属开发方案.md |

### 1.2 项目背景

哨兵机器人作为RoboMaster比赛中的全自动作战单位，必须具备完整的自主导航能力，这是技术评审的强制测试项。本项目基于ROS2 Humble和Nav2导航框架，构建一个轻量化、高可靠性的哨兵专属导航系统，支持巡逻、追击、瞄准、撤退等战术状态机切换，并提供速度安全过滤保障底盘安全。

### 1.3 设计目标

| 目标类别 | 具体指标 |
|---------|---------|
| 功能完整性 | 支持巡逻、追击、瞄准、撤退四种核心战术状态 |
| 定位精度 | 绝对定位误差 ≤±1cm（Point-LIO+ikd-Tree增量式建图） |
| 姿态精度 | 航向角误差 ≤±0.3° |
| 响应延迟 | 避障响应 <20ms，全局决策周期 <30ms |
| LiDAR频率 | >100Hz（增量式建图） |
| 重定位时间 | <1s（Small-GICP） |
| 稳定性 | 连续运行 ≥8小时无崩溃 |
| 算力适配 | 适配本地笔记本测试环境，CPU占用 ≤70% |

### 1.4 超高性能方案（基于深北莫北极熊战队2025赛季）

| 模块 | 技术方案 | 性能指标 | 对比 |
|------|---------|---------|------|
| **里程计** | **Point-LIO**（FAST-LIO升级版） | ±1cm，>100Hz | FAST-LIO: ±3cm, 10Hz |
| **建图方式** | ikd-Tree增量式建图 | >100Hz | FAST-LIO: 10Hz |
| **路径跟踪** | **pb_omni_pid_pursuit_controller** | <20ms延迟 | DWB控制器: >50ms |
| **重定位** | **Small-GICP** | <1s恢复 | AMCL: >3s |
| **地形分析** | terrain_analysis（4m范围） | 实时障碍物高度识别 | 无 |
| **IMU融合** | 外部IMU紧耦合 | ±0.3°航向 | 无 |

---

## 二、系统架构设计

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    哨兵高精度导航系统架构                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              sentinel_navigation (核心)               │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │  │
│  │  │ 战术状态机│  │Nav2 Action│  │ 速度安全过滤节点  │   │  │
│  │  │          │  │  客户端   │  │                  │   │  │
│  │  └──────────┘  └──────────┘  └──────────────────┘   │  │
│  └──────────────────────────────────────────────────────┘  │
│         │                   │                  │            │
│         ▼                   ▼                  ▼            │
│  ┌──────────┐      ┌──────────────┐   ┌──────────────┐    │
│  │sentinel  │      │  Nav2 Stack  │   │ 底盘驱动订阅  │    │
│  │_msgs     │◀────▶│ (外部依赖)   │◀──│ /cmd_vel_safe│    │
│  └──────────┘      └──────────────┘   └──────────────┘    │
│                                             │              │
│                             ┌───────────────▼────────────┐  │
│                             │    sentinel_bringup         │  │
│                             │  ┌──────────────────────┐  │  │
│                             │  │ Nav2优化参数配置      │  │  │
│                             │  │ Launch启动文件        │  │  │
│                             │  │ 示例地图              │  │  │
│                             │  │ RViz配置              │  │  │
│                             │  └──────────────────────┘  │  │
│                             └──────────────────────────┘  │
│                                             │              │
│  ┌────────────────────────────────────────┴──────────────┐│
│  │          高精度定位融合层                             ││
│  │  ┌──────────────────┐     ┌────────────────────┐    ││
│  │  │sentinel_fast_lio │────▶│ /Odometry (FAST-LIO)│    ││
│  │  │  LiDAR-IMU紧耦合 │     │  ±3cm精度           │    ││
│  │  └──────────────────┘     └────────────────────┘    ││
│  │           │                                     │    ││
│  │           ▼                                     │    ││
│  │  ┌──────────────────┐     ┌────────────────────┐    ││
│  │  │sentinel_ekf_fusion│────▶│ /odom (EKF融合)    │    ││
│  │  │  EKF姿态融合     │     │  ±0.5°航向精度     │    ││
│  │  └──────────────────┘     └────────────────────┘    ││
│  └────────────────────────────────────────────────────┘│
│         │                        │                       │
│         ▼                        ▼                       │
│  ┌──────────────────┐    ┌──────────────────┐            │
│  │Mid360点云数据    │    │IMU数据          │            │
│  │/livox/lidar      │    │/livox/imu       │            │
│  └──────────────────┘    └──────────────────┘            │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐│
│  │          sentinel_description (传感器模型)             ││
│  │  ┌──────────────────────┐  ┌──────────────────────┐  ││
│  │  │ Mid360激光雷达URDF   │  │ IMU传感器URDF        │  ││
│  │  │ 倾斜45度安装         │  │ BMI088 IMU           │  ││
│  │  └──────────────────────┘  └──────────────────────┘  ││
│  └──────────────────────────────────────────────────────┘│
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 包结构设计

```
sentinel_robot/
├── sentinel_msgs/              # 自定义消息/服务定义
│   ├── msg/
│   │   ├── TargetPose.msg      # 目标位姿消息
│   │   ├── RobotState.msg      # 机器人状态消息
│   │   ├── SentryMode.msg      # 哨兵模式消息
│   │   └── TacticalTarget.msg  # 战术目标消息
│   ├── srv/
│   │   ├── SetSentryMode.srv   # 设置哨兵模式服务
│   │   └── GetTacticalTarget.srv # 获取战术目标服务
│   ├── CMakeLists.txt
│   └── package.xml
│
├── sentinel_navigation/        # 核心导航功能包
│   ├── sentinel_navigation/
│   │   ├── __init__.py
│   │   ├── tactical_state_machine.py    # 战术状态机
│   │   ├── nav2_action_client.py        # Nav2 Action客户端
│   │   ├── velocity_safety_filter.py    # 速度安全过滤节点
│   │   └── patrol_waypoints.py          # 巡逻航点管理
│   ├── launch/
│   │   ├── tactical_state_machine_launch.py
│   │   └── velocity_safety_filter_launch.py
│   ├── config/
│   │   ├── tactical_params.yaml
│   │   └── safety_filter_params.yaml
│   ├── CMakeLists.txt
│   └── package.xml
│
├── sentinel_bringup/           # 启动与配置包
│   ├── launch/
│   │   ├── bringup_launch.py               # 主启动文件
│   │   ├── navigation_launch.py            # 导航启动
│   │   ├── localization_launch.py          # 定位启动
│   │   ├── slam_launch.py                  # 建图启动
│   │   └── rviz_launch.py                  # 可视化启动
│   ├── config/
│   │   ├── nav2_params.yaml                # Nav2主参数
│   │   ├── bt_navigator.xml                # 行为树配置
│   │   ├── planner_server_params.yaml      # 规划器参数
│   │   ├── controller_server_params.yaml   # 控制器参数
│   │   ├── smoother_server_params.yaml     # 平滑器参数
│   │   ├── costmap_filter_info_common.yaml # 代价地图通用配置
│   │   ├── global_costmap_params.yaml      # 全局代价地图
│   │   ├── local_costmap_params.yaml       # 局部代价地图
│   │   ├── ekf.yaml                        # EKF融合参数
│   │   └── slam_toolbox_params.yaml        # SLAM参数
│   ├── maps/                               # 地图目录
│   │   ├── rmul_2024.yaml
│   │   ├── rmul_2024.pgm
│   │   ├── rmuc_2025.yaml
│   │   └── rmuc_2025.pgm
│   ├── rviz/
│   │   └── navigation_view.rviz            # RViz配置
│   ├── CMakeLists.txt
│   └── package.xml
│
└── sentinel_description/        # 机器人描述包
    ├── urdf/
    │   ├── sentry_robot.urdf.xacro
    │   ├── macros/
    │   │   ├── chassis.xacro
    │   │   ├── gimbal.xacro
    │   │   └── sensors.xacro
    │   └── wheels/
    │       └── mecanum_wheel.urdf.xacro
    ├── launch/
    │   ├── description_launch.py           # 模型发布
    │   └── state_publisher_launch.py       # 状态发布器
    ├── meshes/                             # 3D模型文件
    ├── CMakeLists.txt
    └── package.xml
```

---

## 三、核心模块详细设计

### 3.1 sentinel_msgs 消息定义包

#### 3.1.1 消息定义

**TargetPose.msg** - 目标位姿消息
```yaml
# 目标位姿信息，用于导航和瞄准
std_msgs/Header header

geometry_msgs/PoseStamped pose  # 目标位姿
uint8 target_type               # 目标类型: 0=未知, 1=巡逻点, 2=敌方, 3=补给点, 4=安全点
uint8 priority                  # 优先级: 0=低, 1=中, 2=高
float64 distance                # 距离估计（米）
float64 confidence              # 置信度 0.0-1.0
```

**RobotState.msg** - 机器人状态消息
```yaml
std_msgs/Header header

# 基础状态
uint8 sentry_mode               # 当前哨兵模式: 0=PATROL, 1=TRACKING, 2=AIMING, 3=RETREAT, 4=RUNE
uint8 health_status             # 健康状态: 0=正常, 1=受损, 2=严重, 3=危急
float64 current_hp              # 当前血量
float64 max_hp                  # 最大血量
float64 heat                    # 当前热量

# 运动状态
geometry_msgs/Twist current_velocity  # 当前速度
geometry_msgs/Pose current_pose       # 当前位姿

# 战术状态
TargetPose current_target      # 当前目标
bool has_line_of_sight         # 是否有视线
uint32 target_lock_duration    # 目标锁定时长（毫秒）

# 系统状态
bool navigation_active         # 导航是否激活
bool aiming_active             # 瞄准是否激活
uint8 system_status            # 系统状态: 0=正常, 1=警告, 2=错误
```

**SentryMode.msg** - 哨兵模式消息
```yaml
std_msgs/Header header

uint8 mode                      # 模式: 0=PATROL, 1=TRACKING, 2=AIMING, 3=RETREAT, 4=RUNE
uint8 previous_mode             # 上一模式
builtin_interfaces/Duration mode_duration  # 当前模式持续时间
float64 mode_confidence         # 模式置信度
```

**TacticalTarget.msg** - 战术目标消息
```yaml
std_msgs/Header header

TargetPose primary_target       # 主目标
TargetPose secondary_target     # 次要目标
uint8 tactical_situation        # 战术态势: 0=优势, 1=均势, 2=劣势
bool should_engage              # 是否应该交战
bool should_retreat             # 是否应该撤退
float64 threat_level            # 威胁等级 0.0-1.0
```

#### 3.1.2 服务定义

**SetSentryMode.srv** - 设置哨兵模式服务
```yaml
# Request
uint8 mode                      # 目标模式: 0=PATROL, 1=TRACKING, 2=AIMING, 3=RETREAT, 4=RUNE
float64 timeout                 # 超时时间（秒），0表示无限
---
# Response
bool success                    # 是否成功
uint8 current_mode              # 当前模式
string message                  # 状态消息
```

**GetTacticalTarget.srv** - 获取战术目标服务
```yaml
# Request
uint8 requested_target_type     # 请求的目标类型: 0=任意, 1=巡逻, 2=敌方, 3=补给, 4=安全
---
# Response
bool success                    # 是否成功
TacticalTarget tactical_target # 战术目标信息
string message                  # 状态消息
```

#### 3.1.3 package.xml
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentinel_msgs</name>
  <version>1.0.0</version>
  <description>自定义消息和服务定义，用于哨兵机器人导航系统</description>
  <maintainer email="saltwave@example.com">SaltWave Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### 3.1.4 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(sentinel_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  "msg/RobotState.msg"
  "msg/SentryMode.msg"
  "msg/TacticalTarget.msg"
  "srv/SetSentryMode.srv"
  "srv/GetTacticalTarget.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
```

---

### 3.2 sentinel_navigation 核心导航包

#### 3.2.1 战术状态机设计

**状态机状态定义：**

| 状态 | 英文标识 | 描述 | 进入条件 | 退出条件 |
|------|---------|------|---------|---------|
| 巡逻 | PATROL | 按预设路线巡逻，搜索目标 | 系统启动或无目标 | 发现敌方目标、血量低、收到撤退指令 |
| 跟踪 | TRACKING | 接近目标，保持射击距离 | 发现敌方目标且距离>2m | 目标进入射程、目标丢失、血量危急 |
| 瞄准 | AIMING | 原地稳定，精准射击 | 目标进入射程（<2m）且血量充足 | 目标丢失、血量低、需要走位 |
| 撤退 | RETREAT | 返回安全区域或补血点 | 血量低于阈值、被围攻 | 血量恢复、到达安全点 |
| 打符 | RUNE | 击打能量机关 | 发现能量机关且状态允许 | 打符完成、超时、被攻击 |

**状态转换图：**

```
                    ┌─────────────┐
                    │   PATROL    │
                    │  (巡逻)     │
                    └──────┬──────┘
                           │ 发现目标
                           ▼
                    ┌─────────────┐
         血量危急 ◀───┤  TRACKING  │───▶ 进入射程
                    │  (跟踪)     │
                    └──────┬──────┘
                           │
               ┌───────────┴───────────┐
               │                       │
               ▼                       ▼
        ┌─────────────┐         ┌─────────────┐
        │   RETREAT   │         │   AIMING    │
        │  (撤退)     │         │  (瞄准)     │
        └──────┬──────┘         └──────┬──────┘
               │                       │
               │ 血量恢复               │ 目标丢失
               │                       │
               └───────────┬───────────┘
                           │
                           ▼
                    ┌─────────────┐
         发现能量机关 ───▶    RUNE     │───▶ 完成/超时
                    │  (打符)     │
                    └─────────────┘
```

#### 3.2.2 tactical_state_machine.py 实现

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from sentinel_msgs.msg import RobotState, SentryMode, TargetPose
from sentinel_msgs.srv import SetSentryMode, GetTacticalTarget
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class TacticalStateMachine(Node):
    """哨兵战术状态机节点"""

    def __init__(self):
        super().__init__('tactical_state_machine')

        # 状态定义
        self.SENTRY_PATROL = 0
        self.SENTRY_TRACKING = 1
        self.SENTRY_AIMING = 2
        self.SENTRY_RETREAT = 3
        self.SENTRY_RUNE = 4

        # 当前状态
        self.current_state = self.SENTRY_PATROL
        self.state_start_time = self.get_clock().now()
        self.previous_state = None

        # 初始化Nav2导航器
        self.navigator = BasicNavigator()

        # 状态持续时间阈值（秒）
        self.patrol_duration = 30.0
        self.tracking_duration = 10.0
        self.aiming_duration = 5.0
        self.retreat_duration = 15.0
        self.rune_duration = 8.0

        # 阈值参数
        self.hp_critical_threshold = 0.3  # 血量低于30%时撤退
        self.hp_low_threshold = 0.5       # 血量低于50%时警惕
        self.aiming_distance = 2.0        # 瞄准距离（米）
        self.tracking_distance = 5.0      # 跟踪距离（米）

        # 巡逻航点
        self.patrol_waypoints = []

        # 订阅者
        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_state_callback,
            10)

        # 发布者
        self.sentry_mode_pub = self.create_publisher(
            SentryMode,
            '/sentry_mode',
            10)

        # 服务
        self.set_mode_service = self.create_service(
            SetSentryMode,
            '/set_sentry_mode',
            self.set_sentry_mode_callback)

        self.get_tactical_target_client = self.create_client(
            GetTacticalTarget,
            '/get_tactical_target')

        # 状态发布定时器
        self.mode_pub_timer = self.create_timer(0.1, self.publish_sentry_mode)

        # 状态机更新定时器
        self.state_machine_timer = self.create_timer(0.1, self.update_state_machine)

        self.get_logger().info('战术状态机节点已启动')

    def robot_state_callback(self, msg):
        """接收机器人状态"""
        self.current_robot_state = msg

    def publish_sentry_mode(self):
        """发布当前哨兵模式"""
        msg = SentryMode()
        msg.mode = self.current_state
        msg.previous_mode = self.previous_state if self.previous_state else self.current_state

        now = self.get_clock().now()
        msg.mode_duration = now - self.state_start_time

        msg.mode_confidence = 1.0  # TODO: 根据条件动态调整

        self.sentry_mode_pub.publish(msg)

    def set_sentry_mode_callback(self, request, response):
        """设置哨兵模式服务回调"""
        target_mode = request.mode
        self.transition_to(target_mode)
        response.success = True
        response.current_mode = self.current_state
        response.message = f'模式切换为: {self.get_mode_name(target_mode)}'
        return response

    def transition_to(self, new_state):
        """状态转换"""
        if new_state == self.current_state:
            return

        self.get_logger().info(
            f'状态转换: {self.get_mode_name(self.current_state)} -> {self.get_mode_name(new_state)}')

        # 退出当前状态
        self.on_exit_state(self.current_state)

        # 记录上一状态
        self.previous_state = self.current_state

        # 进入新状态
        self.current_state = new_state
        self.state_start_time = self.get_clock().now()

        # 进入新状态
        self.on_enter_state(new_state)

    def get_mode_name(self, mode):
        """获取模式名称"""
        mode_names = {
            self.SENTRY_PATROL: 'PATROL',
            self.SENTRY_TRACKING: 'TRACKING',
            self.SENTRY_AIMING: 'AIMING',
            self.SENTRY_RETREAT: 'RETREAT',
            self.SENTRY_RUNE: 'RUNE'
        }
        return mode_names.get(mode, 'UNKNOWN')

    def on_enter_state(self, state):
        """进入状态时的行为"""
        if state == self.SENTRY_PATROL:
            self.enter_patrol()
        elif state == self.SENTRY_TRACKING:
            self.enter_tracking()
        elif state == self.SENTRY_AIMING:
            self.enter_aiming()
        elif state == self.SENTRY_RETREAT:
            self.enter_retreat()
        elif state == self.SENTRY_RUNE:
            self.enter_rune()

    def on_exit_state(self, state):
        """退出状态时的行为"""
        if state in [self.SENTRY_TRACKING, self.SENTRY_AIMING, self.SENTRY_RETREAT]:
            self.navigator.cancelTask()

    def update_state_machine(self):
        """状态机主循环"""
        if not hasattr(self, 'current_robot_state'):
            return

        state_duration = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

        # 获取战术目标
        tactical_target = None
        if self.get_tactical_target_client.wait_for_service(timeout_sec=0.5):
            request = GetTacticalTarget.Request()
            request.requested_target_type = 0
            future = self.get_tactical_target_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            if future.done():
                response = future.result()
                if response.success:
                    tactical_target = response.tactical_target

        # 状态转换逻辑
        if self.current_state == self.SENTRY_PATROL:
            self.update_patrol(state_duration, tactical_target)
        elif self.current_state == self.SENTRY_TRACKING:
            self.update_tracking(state_duration, tactical_target)
        elif self.current_state == self.SENTRY_AIMING:
            self.update_aiming(state_duration, tactical_target)
        elif self.current_state == self.SENTRY_RETREAT:
            self.update_retreat(state_duration, tactical_target)
        elif self.current_state == self.SENTRY_RUNE:
            self.update_rune(state_duration, tactical_target)

    def enter_patrol(self):
        """进入巡逻状态"""
        self.get_logger().info('进入巡逻模式')

        # 获取巡逻航点
        patrol_points = self.get_patrol_waypoints()

        if len(patrol_points) >= 2:
            self.navigator.startThroughPoses(patrol_points)
        elif len(patrol_points) == 1:
            self.navigator.goToPose(patrol_points[0])
        else:
            self.get_logger().warn('未配置巡逻航点，保持原地待命')

    def update_patrol(self, duration, tactical_target):
        """巡逻状态更新"""
        # 检查是否发现目标
        if tactical_target and tactical_target.primary_target.target_type == 2:
            distance = tactical_target.primary_target.distance
            if distance < self.tracking_distance:
                self.transition_to(self.SENTRY_TRACKING)
                return

        # 检查血量
        if self.current_robot_state.current_hp / self.current_robot_state.max_hp < self.hp_critical_threshold:
            self.transition_to(self.SENTRY_RETREAT)
            return

        # 检查是否需要打符
        if tactical_target and tactical_target.primary_target.target_type == 5:
            if tactical_target.should_engage:
                self.transition_to(self.SENTRY_RUNE)
                return

        # 巡逻超时，重新规划
        if duration > self.patrol_duration:
            if self.navigator.isTaskComplete():
                self.get_logger().info('巡逻完成，重新开始')
                self.enter_patrol()

    def enter_tracking(self):
        """进入跟踪状态"""
        self.get_logger().info('进入跟踪模式')

    def update_tracking(self, duration, tactical_target):
        """跟踪状态更新"""
        if not tactical_target or tactical_target.primary_target.target_type != 2:
            # 目标丢失
            if duration > 3.0:
                self.transition_to(self.SENTRY_PATROL)
            return

        distance = tactical_target.primary_target.distance

        # 进入射程，切换到瞄准
        if distance < self.aiming_distance:
            self.transition_to(self.SENTRY_AIMING)
            return

        # 跟踪超时，返回巡逻
        if duration > self.tracking_duration:
            self.transition_to(self.SENTRY_PATROL)
            return

        # 血量危急，撤退
        if self.current_robot_state.current_hp / self.current_robot_state.max_hp < self.hp_critical_threshold:
            self.transition_to(self.SENTRY_RETREAT)
            return

        # 更新跟踪目标位置
        target_pose = tactical_target.primary_target.pose
        self.navigator.goToPose(target_pose)

    def enter_aiming(self):
        """进入瞄准状态"""
        self.get_logger().info('进入瞄准模式')
        # 停止移动，准备瞄准
        self.navigator.cancelTask()

    def update_aiming(self, duration, tactical_target):
        """瞄准状态更新"""
        if not tactical_target or tactical_target.primary_target.target_type != 2:
            # 目标丢失
            if duration > 2.0:
                self.transition_to(self.SENTRY_PATROL)
            return

        distance = tactical_target.primary_target.distance

        # 目标远离，回到跟踪
        if distance > self.aiming_distance * 1.5:
            self.transition_to(self.SENTRY_TRACKING)
            return

        # 瞄准超时，重新评估
        if duration > self.aiming_duration:
            if distance > self.aiming_distance:
                self.transition_to(self.SENTRY_TRACKING)
            else:
                # 继续瞄准
                self.state_start_time = self.get_clock().now()
            return

        # 血量低，撤退
        if self.current_robot_state.current_hp / self.current_robot_state.max_hp < self.hp_low_threshold:
            self.transition_to(self.SENTRY_RETREAT)
            return

    def enter_retreat(self):
        """进入撤退状态"""
        self.get_logger().info('进入撤退模式')

        # 获取撤退目标点（补血点或安全点）
        retreat_pose = self.get_retreat_pose()
        if retreat_pose:
            self.navigator.goToPose(retreat_pose)
        else:
            self.get_logger().warn('未配置撤退点，保持原地待命')

    def update_retreat(self, duration, tactical_target):
        """撤退状态更新"""
        # 检查血量是否恢复
        hp_ratio = self.current_robot_state.current_hp / self.current_robot_state.max_hp
        if hp_ratio > 0.8:
            self.transition_to(self.SENTRY_PATROL)
            return

        # 撤退超时
        if duration > self.retreat_duration:
            self.get_logger().warn('撤退超时，继续巡逻')
            self.transition_to(self.SENTRY_PATROL)
            return

        # 检查是否到达安全点
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('已到达安全点')
                # 在安全点等待血量恢复
                pass

    def enter_rune(self):
        """进入打符状态"""
        self.get_logger().info('进入打符模式')

        # 导航到打符点
        rune_pose = self.get_rune_pose()
        if rune_pose:
            self.navigator.goToPose(rune_pose)

    def update_rune(self, duration, tactical_target):
        """打符状态更新"""
        # 打符超时
        if duration > self.rune_duration:
            self.get_logger().info('打符超时，返回巡逻')
            self.transition_to(self.SENTRY_PATROL)
            return

        # 被攻击，优先撤退
        if self.current_robot_state.current_hp / self.current_robot_state.max_hp < self.hp_low_threshold:
            self.transition_to(self.SENTRY_RETREAT)
            return

    def get_patrol_waypoints(self):
        """获取巡逻航点"""
        # TODO: 从参数服务器读取
        waypoints = []
        # 示例：读取YAML配置
        return waypoints

    def get_retreat_pose(self):
        """获取撤退点"""
        # TODO: 从参数服务器读取
        return None

    def get_rune_pose(self):
        """获取打符点"""
        # TODO: 从参数服务器读取
        return None


def main(args=None):
    rclpy.init(args=args)
    state_machine = TacticalStateMachine()
    rclpy.spin(state_machine)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3.2.3 velocity_safety_filter.py 速度安全过滤节点

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class VelocitySafetyFilter(Node):
    """速度安全过滤节点，过滤危险的速度指令"""

    def __init__(self):
        super().__init__('velocity_safety_filter')

        # 参数
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('emergency_stop', True)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.emergency_stop = self.get_parameter('emergency_stop').value

        # 订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # 发布者
        self.cmd_vel_safe_pub = self.create_publisher(
            Twist,
            '/cmd_vel_safe',
            10)

        self.latest_cmd_vel = Twist()
        self.latest_scan = None

        self.get_logger().info('速度安全过滤节点已启动')

    def cmd_vel_callback(self, msg):
        """接收速度指令"""
        self.latest_cmd_vel = msg
        self.filter_and_publish()

    def scan_callback(self, msg):
        """接收激光数据"""
        self.latest_scan = msg
        self.filter_and_publish()

    def filter_and_publish(self):
        """过滤并发布安全速度指令"""
        safe_cmd = Twist()

        # 限制速度
        safe_cmd.linear.x = max(-self.max_linear_speed,
                                min(self.max_linear_speed, self.latest_cmd_vel.linear.x))
        safe_cmd.linear.y = max(-self.max_linear_speed,
                                min(self.max_linear_speed, self.latest_cmd_vel.linear.y))
        safe_cmd.linear.z = max(-self.max_linear_speed,
                                min(self.max_linear_speed, self.latest_cmd_vel.linear.z))

        safe_cmd.angular.x = max(-self.max_angular_speed,
                                 min(self.max_angular_speed, self.latest_cmd_vel.angular.x))
        safe_cmd.angular.y = max(-self.max_angular_speed,
                                 min(self.max_angular_speed, self.latest_cmd_vel.angular.y))
        safe_cmd.angular.z = max(-self.max_angular_speed,
                                 min(self.max_angular_speed, self.latest_cmd_vel.angular.z))

        # 障碍物检测
        if self.latest_scan and self.emergency_stop:
            if self.check_collision_risk(safe_cmd):
                self.get_logger().warn('检测到碰撞风险，紧急制动')
                safe_cmd.linear.x = 0.0
                safe_cmd.linear.y = 0.0
                safe_cmd.linear.z = 0.0
                safe_cmd.angular.x = 0.0
                safe_cmd.angular.y = 0.0
                safe_cmd.angular.z = 0.0

        self.cmd_vel_safe_pub.publish(safe_cmd)

    def check_collision_risk(self, cmd_vel):
        """检查碰撞风险"""
        if not self.latest_scan:
            return False

        # 简单碰撞检测：检查前方安全距离内是否有障碍物
        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges

        # 计算前方角度范围
        forward_angle = 0.5  # 弧度，约30度
        start_idx = int((angle_min + forward_angle) / angle_increment)
        end_idx = int((angle_min - forward_angle) / angle_increment)

        # 检查前方障碍物
        for i in range(min(start_idx, end_idx), max(start_idx, end_idx)):
            if 0 < i < len(ranges):
                if ranges[i] < self.safety_distance:
                    return True

        return False


def main(args=None):
    rclpy.init(args=args)
    safety_filter = VelocitySafetyFilter()
    rclpy.spin(safety_filter)
    safety_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3.2.4 package.xml
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentinel_navigation</name>
  <version>1.0.0</version>
  <description>哨兵机器人核心导航功能包，包含战术状态机、Nav2客户端和速度安全过滤</description>
  <maintainer email="saltwave@example.com">SaltWave Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>rclcpp</depend>
  <depend>nav2_simple_commander</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <depend>sentinel_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### 3.2.5 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(sentinel_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_simple_commander REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_python_install_scripts(
  scripts/tactical_state_machine.py
  scripts/velocity_safety_filter.py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_lint_cmake QUIET)
  find_package(ament_cmake_lint_python QUIET)
  find_package(ament_cmake_pycodestyle QUIET)
  find_package(ament_cmake_cpplint QUIET)
  find_package(ament_cmake_uncrustify QUIET)
  find_package(ament_cmake_flake8 QUIET)
  find_package(ament_cmake_pep257 QUIET)
  find_package(ament_cmake_xmllint QUIET)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

---

### 3.3 sentinel_bringup 启动配置包

#### 3.3.1 bringup_launch.py 主启动文件

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """启动哨兵机器人导航系统"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # 参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='sentry',
        description='机器人命名空间'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='自动启动导航系统'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sentinel_bringup'),
            'config',
            'nav2_params.yaml'
        ]),
        description='参数文件路径'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图文件路径'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='使用组合节点'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='节点崩溃时重启'
    )

    # 路径
    sentinel_bringup_dir = get_package_share_directory('sentinel_bringup')
    sentinel_navigation_dir = get_package_share_directory('sentinel_navigation')

    # Nav2启动
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map_yaml_file': map_yaml,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items()
    )

    # 战术状态机
    tactical_state_machine_cmd = Node(
        package='sentinel_navigation',
        executable='tactical_state_machine',
        name='tactical_state_machine',
        output='screen',
        parameters=[PathJoinSubstitution([
            sentinel_navigation_dir,
            'config',
            'tactical_params.yaml'
        ])],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav2'),
        ]
    )

    # 速度安全过滤节点
    velocity_safety_filter_cmd = Node(
        package='sentinel_navigation',
        executable='velocity_safety_filter',
        name='velocity_safety_filter',
        output='screen',
        parameters=[PathJoinSubstitution([
            sentinel_navigation_dir,
            'config',
            'safety_filter_params.yaml'
        ])],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav2'),
        ]
    )

    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            sentinel_bringup_dir,
            'rviz',
            'navigation_view.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfiguration('use_rviz')
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(nav2_bringup_cmd)
    ld.add_action(tactical_state_machine_cmd)
    ld.add_action(velocity_safety_filter_cmd)
    ld.add_action(rviz_cmd)

    return ld
```

#### 3.3.2 nav2_params.yaml Nav2主参数配置

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_d: 0.2
    update_min_a: 0.5
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: "scan"

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    default_bt_xml: ""
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updated_controller_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_controller_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_are_error_codes_active_condition_bt_node
    - nav2_would_a_planner_recovery_help_condition_bt_node
    - nav2_would_a_controller_recovery_help_condition_bt_node
    - nav2_would_a_smoother_recovery_help_condition_bt_node
    transform_timeout: 0.1
    bt_loop_duration: 10
    default_nav_to_pose_bt_xml: navigate_w_replanning_and_recovery.xml

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    FollowPath:
      plugin: "nav2_controller::DWBController"
      debug_trajectory_details: False
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      rot_stopped_velocity: 0.25
      include_costmap_obstacles: True
      costmap_obstacles_behind_robot_dist: 1.0
      obstacle_poses_ignored: True
      obstacle_cost_factor: 1.0
      primitive_footprint_expansion: 0.0
      use_regulated_linear_velocity_scaling: True
      regulated_linear_scaling_min_speed: 0.25
      use_regulated_angular_velocity_scaling: True
      regulated_angular_scaling_min_speed: 0.25
      swap_xy_goal_tolerance: True
      short_circuit_trajectory_evaluation: True
      stateful: True
      critial_speed: False
      publish_cost_grid: False
      publish_global_plan: True
      publish_footprint: False
      publish_transformed_pc: False
      velocity_scaling_factor: 1.0
      FootprintScaling: True
      Approaching: False
      max_speed_approx_limit: 0.5
    ProgressChecker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    GoalChecker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

local_costmap:
  global_frame: odom
  robot_base_frame: base_footprint
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
  robot_radius: 0.22
  plugins: ["voxel_layer", "inflation_layer"]
  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.0
    inflation_radius: 0.55
  voxel_layer:
    plugin: "nav2_costmap_2d::VoxelLayer"
    enabled: True
    publish_voxel_map: True
    origin_z: 0.0
    z_resolution: 0.05
    z_voxels: 16
    max_obstacle_height: 2.0
    mark_threshold: 0
    observation_sources: scan
    scan:
      topic: /scan
      max_obstacle_height: 2.0
      clearing: True
      marking: True
      data_type: "LaserScan"
      raytrace_max_range: 3.0
      raytrace_min_range: 0.0
      obstacle_max_range: 2.5
      obstacle_min_range: 0.0
  static_layer:
    map_subscribe_transient_local: True
  always_send_full_costmap: True

global_costmap:
  global_frame: map
  robot_base_frame: base_footprint
  width: 100
  height: 100
  resolution: 0.05
  robot_radius: 0.22
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    enabled: True
    observation_sources: scan
    scan:
      topic: /scan
      max_obstacle_height: 2.0
      clearing: True
      marking: True
      data_type: "LaserScan"
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: True
  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.0
    inflation_radius: 0.55
  always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: False
    yaml_filename: "map"

map_saver:
  ros__parameters:
    use_sim_time: False
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
```

#### 3.3.3 巡逻航点配置

```yaml
patrol_waypoints:
  frame_id: map
  waypoints:
    - position:
        x: 1.0
        y: 1.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    - position:
        x: 3.0
        y: 1.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.707
        w: 0.707
    - position:
        x: 3.0
        y: 3.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0
    - position:
        x: 1.0
        y: 3.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: -0.707
        w: 0.707

retreat_pose:
  frame_id: map
  position:
    x: 0.5
    y: 0.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0

rune_pose:
  frame_id: map
  position:
    x: 2.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

---

### 3.4 sentinel_description 机器人描述包

#### 3.4.1 sentry_robot.urdf.xacro 机器人模型

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sentry_robot">

  <xacro:include filename="$(find sentinel_description)/urdf/macros/chassis.xacro"/>
  <xacro:include filename="$(find sentinel_description)/urdf/macros/gimbal.xacro"/>
  <xacro:include filename="$(find sentinel_description)/urdf/macros/sensors.xacro"/>
  <xacro:include filename="$(find sentinel_description)/urdf/wheels/mecanum_wheel.urdf.xacro"/>

  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- 底盘 -->
  <xacro:sentry_chassis />

  <!-- 云台 -->
  <xacro:sentry_gimbal />

  <!-- 传感器 -->
  <xacro:sentry_sensors />

  <!-- Mid360激光雷达（倾斜安装，用于SLAM和导航） -->
  <xacro:mid360_lidar />

</robot>
```

#### 3.4.2 chassis.xacro 底盘宏定义

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="sentry_chassis">

    <link name="base_footprint"/>

    <joint name="base_footprint_joint" type="fixed">
      <parent link="base_footprint"/>
      <child link="base_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>

    <link name="base_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.5 0.4 0.3"/>
        </geometry>
        <material name="blue"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.5 0.4 0.3"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="20.0"/>
        <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
      </inertial>
    </link>

    <!-- 麦克纳姆轮 -->
    <xacro:mecanum_wheel prefix="front_left" xyz="0.285 0.225 0"/>
    <xacro:mecanum_wheel prefix="front_right" xyz="0.285 -0.225 0"/>
    <xacro:mecanum_wheel prefix="back_left" xyz="-0.285 0.225 0"/>
    <xacro:mecanum_wheel prefix="back_right" xyz="-0.285 -0.225 0"/>

  </xacro:macro>

</robot>
```

#### 3.4.3 description_launch.py 模型发布启动文件

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

  urdf_pkg_name = 'sentinel_description'
  urdf_file_name = 'sentry_robot.urdf.xacro'

  ld = LaunchDescription()

  urdf_model_path = PathJoinSubstitution([
      FindPackageShare(urdf_pkg_name),
      'urdf',
      urdf_file_name
  ])

  robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      parameters=[{'robot_description': urdf_model_path}]
  )

  ld.add_action(robot_state_publisher_node)

  return ld
  ```

#### 3.4.4 sensors.xacro 传感器宏定义（含Mid360）

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="sentry_sensors">
    <!-- 相机等传感器定义 -->
  </xacro:macro>

  <!-- Mid360激光雷达宏定义 -->
  <xacro:macro name="mid360_lidar" params="parent:=base_link xyz:=0.2 0.0 0.3 rpy:=0.0 0.0 0.0">
    <joint name="mid360_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="mid360"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <link name="mid360">
      <visual>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
          <cylinder radius="0.036" length="0.084"/>
        </geometry>
        <material name="black"/>
      </visual>

      <collision>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
          <cylinder radius="0.036" length="0.084"/>
        </geometry>
      </collision>

      <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <mass value="0.385"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Mid360雷达安装外参（可调整） -->
    <!-- 倾斜45度安装，优化SLAM效果 -->
    <joint name="mid360_base_link_joint" type="fixed">
      <parent link="mid360"/>
      <child link="livox_lidar_publisher0"/>
      <origin xyz="0.0 0.0 0.0637" rpy="0.0 0.785 0.0"/>
    </joint>

    <link name="livox_lidar_publisher0"/>
  </xacro:macro>

</robot>
```

#### 3.4.5 Mid360激光雷达驱动配置

```json
{
  "lidar_summary": {
    "lidar_type": 8
  },
  "rosetta": {
    "ip": "192.168.1.12"
  },
  "lidar_net_info": {
    "cmd_port": 65010,
    "push_port": 65011,
    "msop_port": 65012,
    "difop_port": 65013
  },
  "enable_connect": true,
  "coordinate_conversion": {
    "enable": false
  }
}
```

---

## 四、核心接口定义

### 4.1 话题接口

| 话题名称 | 消息类型 | 发布者/订阅者 | 频率 | 描述 |
|---------|---------|--------------|------|------|
| /cmd_vel | geometry_msgs/Twist | Nav2 → 速度过滤 | 50Hz | 导航速度指令 |
| /cmd_vel_safe | geometry_msgs/Twist | 速度过滤 → 底盘 | 50Hz | 安全过滤后的速度指令 |
| /scan | sensor_msgs/LaserScan | 点云转激光 | 20Hz | 2D激光数据（从点云转换） |
| /mid360/lidar | sensor_msgs/PointCloud2 | Mid360驱动 | 10Hz | 原始点云数据 |
| /robot_state | sentinel_msgs/RobotState | 裁判系统 | 10Hz | 机器人状态（血量、热量） |
| /sentry_mode | sentinel_msgs/SentryMode | 状态机 | 10Hz | 当前哨兵模式 |
| /initialpose | geometry_msgs/PoseWithCovarianceStamped | RViz | 手动 | 初始位姿设置 |
| /goal_pose | geometry_msgs/PoseStamped | RViz | 手动 | 导航目标点 |

### 4.2 服务接口

| 服务名称 | 服务类型 | 客户端/服务器 | 描述 |
|---------|---------|--------------|------|
| /set_sentry_mode | sentinel_msgs/SetSentryMode | 外部 → 状态机 | 手动设置哨兵模式 |
| /get_tactical_target | sentinel_msgs/GetTacticalTarget | 状态机 → 自瞄 | 获取战术目标 |

### 4.3 动作接口

| 动作名称 | 动作类型 | 客户端/服务器 | 描述 |
|---------|---------|--------------|------|
| /navigate_to_pose | nav2_msgs/NavigateToPose | 状态机 → Nav2 | 导航到位姿 |
| /follow_waypoints | nav2_msgs/FollowWaypoints | 状态机 → Nav2 | 跟随航点 |

---

## 五、部署与运行

### 5.1 编译安装

```bash
cd /home/pyy/Documents/sentiel
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5.2 启动导航系统

```bash
# 实机运行
ros2 launch sentinel_bringup bringup_launch.py use_sim_time:=false

# 仿真运行
ros2 launch sentinel_bringup bringup_launch.py use_sim_time:=true
```

### 5.3 使用RViz发布导航目标

1. 启动RViz后，使用 `Nav2 Goal` 插件点击地图上的目标位置
2. 机器人将自动规划路径并导航到目标点
3. 状态机将根据当前模式自动响应

### 5.4 Mid360激光雷达配置

**1. 网络配置：**

```bash
# 设置Mid360的静态IP为192.168.1.12
# 上位机IP设置为192.168.1.X（同一网段）
```

**2. 启动Mid360驱动：**

```bash
# 修改livox_ros_driver2配置文件中的IP地址
# 启动驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**3. 点云转激光配置：**

```yaml
# sentinel_bringup/config/pointcloud_to_laserscan.yaml
pointcloud_to_laserscan_node:
  ros__parameters:
    use_sim_time: false
    min_height: 0.0
    max_height: 1.0
    angle_min: -3.14159  # -180度
    angle_max: 3.14159   # 180度
    angle_increment: 0.0087  # 0.5度
    scan_time: 0.3333
    range_min: 0.05
    range_max: 100.0
    use_inf: true
    inf_epsilon: 1.0
```

**4. 验证点云和激光数据：**

```bash
# 查看点云数据
ros2 topic echo /mid360/lidar

# 查看激光数据
ros2 topic echo /scan

# RViz可视化
rviz2
# 添加PointCloud2显示 /mid360/lidar
# 添加LaserScan显示 /scan
```

### 5.5 上车前配置

1. **修改巡逻航点**：编辑 `sentinel_bringup/config/patrol_waypoints.yaml`
2. **替换比赛地图**：将比赛地图文件放入 `sentinel_bringup/maps/` 目录
3. **调整Nav2参数**：根据实际机器人尺寸调整 `nav2_params.yaml` 中的 `robot_radius` 和 `inflation_radius`
4. **速度限制**：根据场地安全要求调整 `max_vel_x` 和 `max_vel_theta`
5. **Mid360外参校准**：根据实际安装位置调整URDF中的xyz和rpy参数

---

## 六、测试与验收

### 6.1 编译与语法测试

#### 6.1.1 Python源码语法检查

```bash
# 检查所有Python文件语法
find src/sentinel_navigation -name "*.py" -exec python3 -m py_compile {} \;

# 或使用flake8检查代码风格
pip3 install flake8
flake8 src/sentinel_navigation/
```

**预期结果：** 无语法错误，无编译失败

#### 6.1.2 YAML配置解析测试

```bash
# 测试所有YAML文件解析
python3 -c "import yaml; yaml.safe_load(open('src/sentinel_bringup/config/nav2_params.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('src/sentinel_navigation/config/tactical_params.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('src/sentinel_navigation/config/safety_filter_params.yaml'))"
```

**预期结果：** 无解析错误

#### 6.1.3 package.xml解析测试

```bash
# 测试所有package.xml
for pkg in sentinel_msgs sentinel_navigation sentinel_bringup sentinel_description; do
  python3 -c "import xml.etree.ElementTree as ET; ET.parse('src/$pkg/package.xml')"
done
```

**预期结果：** 无XML解析错误

#### 6.1.4 编译测试

```bash
cd /home/pyy/Documents/sentiel
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sentinel_msgs
colcon build --symlink-install --packages-select sentinel_navigation
colcon build --symlink-install --packages-select sentinel_bringup
colcon build --symlink-install --packages-select sentinel_description
```

**预期结果：** 编译成功，无错误和警告

### 6.2 单元测试

| 模块 | 测试内容 | 验收标准 |
|------|---------|---------|
| 战术状态机 | 状态转换逻辑 | 所有状态转换正常，无死锁 |
| 速度过滤 | 速度限制与碰撞检测 | 超速自动限制，障碍物检测有效 |
| 消息通信 | 话题/服务通信 | 无丢包，延迟<10ms |

### 6.3 集成测试

#### 6.3.1 Mid360激光雷达测试

```bash
# 1. 启动Mid360驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. 验证点云数据发布
ros2 topic hz /mid360/lidar
# 预期：约10Hz

# 3. 验证点云内容
ros2 topic echo /mid360/lidar --once
# 预期：包含有效的点云数据

# 4. RViz可视化
rviz2
# 添加PointCloud2，话题选择/mid360/lidar
# 预期：能看到点云数据，扫描范围正常
```

#### 6.3.2 点云转激光测试

```bash
# 1. 启动点云转激光节点
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r cloud_in:=/mid360/lidar -r scan:=/scan

# 2. 验证激光数据发布
ros2 topic hz /scan
# 预期：约20Hz

# 3. 验证激光数据
ros2 topic echo /scan --once
# 预期：包含有效的激光扫描数据

# 4. RViz可视化
# 添加LaserScan，话题选择/scan
# 预期：能看到激光扫描线
```

#### 6.3.3 导航功能测试

```bash
# 1. 启动完整导航系统
ros2 launch sentinel_bringup bringup_launch.py use_sim_time:=false

# 2. 设置初始位姿
# 在RViz中使用2D Pose Estimate设置机器人初始位置

# 3. 发布导航目标
# 在RViz中使用Nav2 Goal点击目标位置

# 4. 观察导航行为
# 预期：机器人规划路径并导航到目标点
```

| 测试场景 | 测试步骤 | 预期结果 |
|---------|---------|---------|
| 巡逻模式 | 启动系统，观察巡逻行为 | 按预设航点循环巡逻 |
| 目标跟踪 | 模拟发现目标 | 切换到跟踪模式，接近目标 |
| 避障测试 | 在路径上放置障碍物 | 自动绕行或重新规划 |
| 速度安全 | 发布超速指令 | 自动限制到安全范围内 |
| Mid360数据 | 启动驱动，检查点云 | 点云正常发布，扫描范围正确 |

### 6.4 性能指标验收

| 指标 | 要求值 | 测试方法 |
|------|-------|---------|
| 定位精度 | ≤±10cm | 多次导航到固定点测量误差 |
| 避障响应 | <30ms | 模拟障碍物出现，测量响应时间 |
| 连续运行 | ≥8小时 | 长时间运行监控稳定性 |
| CPU占用 | ≤70% | 系统监控工具测量 |
| Mid360点云频率 | ≥10Hz | `ros2 topic hz /mid360/lidar` |
| 激光扫描频率 | ≥20Hz | `ros2 topic hz /scan` |

### 6.5 上车前完整测试流程

```bash
# 1. 环境检查
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep sentinel

# 2. 语法检查
find src/sentinel_navigation -name "*.py" -exec python3 -m py_compile {} \;

# 3. YAML配置检查
python3 -c "import yaml; yaml.safe_load(open('src/sentinel_bringup/config/nav2_params.yaml'))"

# 4. package.xml检查
python3 -c "import xml.etree.ElementTree as ET; ET.parse('src/sentinel_msgs/package.xml')"

# 5. 清理缓存
rm -rf build/ install/ log/ .cache/

# 6. 完整编译
colcon build --symlink-install

# 7. 检查编译结果
ls install/sentinel_msgs/lib/sentinel_msgs/
ls install/sentinel_navigation/lib/sentinel_navigation/

# 8. 启动测试
ros2 launch sentinel_bringup bringup_launch.py use_sim_time:=false

# 9. 功能验证
# - RViz显示地图和机器人模型
# - 设置初始位姿
# - 发布导航目标
# - 观察导航行为
```

---

## 七、风险与注意事项

### 7.1 技术风险

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| 定位漂移 | 导航失败 | 多传感器融合，定期重定位 |
| 狭窄通道卡住 | 无法通过 | 优化膨胀层参数，添加人工干预 |
| 速度指令异常 | 底盘失控 | 速度安全过滤节点多重保护 |

### 7.2 上车注意事项

1. **地图匹配**：确保上车时使用的地图与建图时一致
2. **初始位姿**：使用RViz的 `2D Pose Estimate` 设置准确的初始位姿
3. **传感器校准**：激光雷达和IMU需要重新标定
4. **安全测试**：先低速测试，确认无误后提高速度
5. **紧急停止**：准备物理急停按钮，确保安全

### 7.3 调试建议

1. 使用 `ros2 topic echo /robot_state` 监控机器人状态
2. 使用 `ros2 topic echo /sentry_mode` 查看当前模式
3. 使用RViz的TF插件检查坐标变换是否正常
4. 使用 `ros2 run nav2_map_server map_saver_cli` 保存地图

---

## 八、附录

### 8.1 参考资料

- [Nav2官方文档](https://navigation.ros.org/)
- [ROS2 Humble教程](https://docs.ros.org/en/humble/Tutorials.html)
- [深北莫pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav)
- [RoboMasterOSS](https://github.com/RoboMasterOSS)

### 8.2 术语表

| 术语 | 英文 | 解释 |
|------|------|------|
| 哨兵 | Sentry | 全自动作战机器人 |
| 巡逻 | Patrol | 按固定路线搜索目标 |
| 追击 | Tracking | 接近目标保持射击距离 |
| 瞄准 | Aiming | 原地稳定精准射击 |
| 撤退 | Retreat | 返回安全区域 |
| 打符 | Rune | 击打能量机关 |

### 8.3 版本历史

| 版本 | 日期 | 作者 | 变更说明 |
|------|------|------|---------|
| v1.0 | 2026-05-26 | SaltWave战队 | 初始版本 |

---

**文档结束**
