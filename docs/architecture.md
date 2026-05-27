# Sentinel Navigation Architecture

This workspace follows the WSL2 + Ubuntu 22.04 + ROS 2 Humble workflow.

## Runtime Graph

- `/scan`, `/odom`, `/tf`: provided by simulation or the real chassis/localization stack.
- `nav2_bringup`: AMCL, planner, controller, behavior tree navigator and costmaps.
- `sentinel_commander`: patrol, tracking, aiming, retreat and rune state machine.
- `velocity_safety_filter`: clamps Nav2 velocity and gates it on robot health/localization state.
- `/cmd_vel`: final chassis command after safety filtering.

## Integration Points

- Replace `sentinel_bringup/maps/empty_rm_field.yaml` with a real map.
- Feed target detections to `/armor_target` using `sentinel_msgs/msg/ArmorTarget`.
- Feed referee/chassis state to `/robot_state` using `sentinel_msgs/msg/RobotState`.
- Connect the chassis driver to `/cmd_vel`.
- Connect the gimbal controller to `/yaw_pitch_cmd` and `/fire_command` if needed.

## Modes

- `PATROL`: loops through configured map waypoints.
- `TRACKING`: sends Nav2 goals toward the latest visible target.
- `AIMING`: stops navigation and publishes fire command when a target is shootable.
- `RETREAT`: navigates to the configured safe pose when health is low or commanded.
- `RUNE`: reserved for fixed-point rune behavior.
- `EMERGENCY_STOP`: cancels Nav2 goal and outputs zero velocity.
