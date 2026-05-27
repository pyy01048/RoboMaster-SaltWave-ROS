# sentiel

ROS 2 Humble navigation workspace for a RoboMaster sentinel robot. It is designed for:

- Windows 11 + WSL2
- Ubuntu 22.04
- ROS 2 Humble
- VSCode Remote WSL

## Packages

- `sentinel_msgs`: custom messages/services for robot state, targets, tactical goals and modes.
- `sentinel_navigation`: Nav2-backed patrol/target/retreat state machine and velocity safety filter.
- `sentinel_bringup`: Nav2 parameters, maps, RViz config and launch files.
- `sentinel_description`: URDF/Xacro and robot state publisher launch.

## Build

```bash
cd /mnt/d/文档/sentiel
source /opt/ros/humble/setup.bash
bash scripts/install_deps.sh
colcon build --symlink-install
source install/setup.bash
```

## Run

With a saved map:

```bash
ros2 launch sentinel_bringup bringup.launch.py \
  map:=src/sentinel_bringup/maps/empty_rm_field.yaml \
  use_sim_time:=false
```

For SLAM mapping:

```bash
ros2 launch sentinel_bringup bringup.launch.py slam:=true use_sim_time:=false
```

## Required Runtime Topics

- `/scan` (`sensor_msgs/LaserScan`)
- `/odom` (`nav_msgs/Odometry`)
- `/tf`, `/tf_static`
- `/robot_state` (`sentinel_msgs/RobotState`, optional but recommended)
- `/armor_target` (`sentinel_msgs/ArmorTarget`, optional perception input)
- `/cmd_vel` (`geometry_msgs/Twist`, Nav2 raw output)
- `/cmd_vel_safe` (`geometry_msgs/Twist`, filtered output to chassis; use this for the real chassis driver)

## Useful Commands

```bash
ros2 service call /sentinel/set_mode sentinel_msgs/srv/SetMode "{mode: 1, reason: patrol}"
ros2 topic echo /sentinel/mode
ros2 topic echo /cmd_vel
```

Replace the example map and patrol points before real field testing.
