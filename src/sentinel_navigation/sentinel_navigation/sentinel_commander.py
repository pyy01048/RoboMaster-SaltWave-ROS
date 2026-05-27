from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sentinel_msgs.msg import ArmorTarget, RobotState, SentinelMode, TacticalGoal, WaypointArray
from sentinel_msgs.srv import SetMode
from std_msgs.msg import Bool


class Mode(IntEnum):
    IDLE = SentinelMode.IDLE
    PATROL = SentinelMode.PATROL
    TRACKING = SentinelMode.TRACKING
    AIMING = SentinelMode.AIMING
    RETREAT = SentinelMode.RETREAT
    RUNE = SentinelMode.RUNE
    EMERGENCY_STOP = SentinelMode.EMERGENCY_STOP


@dataclass
class Waypoint:
    pose: PoseStamped
    hold_time: float
    label: str


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def make_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = quaternion_from_yaw(yaw)
    return pose


class SentinelCommander(Node):
    """High level sentinel navigation state machine backed by Nav2 actions."""

    def __init__(self) -> None:
        super().__init__('sentinel_commander')
        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('patrol_loop', True)
        self.declare_parameter('goal_timeout_sec', 30.0)
        self.declare_parameter('target_timeout_sec', 0.6)
        self.declare_parameter('aiming_distance_m', 2.2)
        self.declare_parameter('retreat_hp_threshold', 120)
        self.declare_parameter('retreat_goal', [-2.0, 0.0, 3.14159])
        self.declare_parameter(
            'patrol_points',
            [
                1.0, 0.8, 0.0,
                1.0, -0.8, -1.57,
                -1.0, -0.8, 3.14159,
                -1.0, 0.8, 1.57,
            ],
        )

        self._global_frame = self.get_parameter('global_frame').value
        self._patrol_loop = bool(self.get_parameter('patrol_loop').value)
        self._goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self._target_timeout = float(self.get_parameter('target_timeout_sec').value)
        self._aiming_distance = float(self.get_parameter('aiming_distance_m').value)
        self._retreat_hp_threshold = int(self.get_parameter('retreat_hp_threshold').value)

        self._mode = Mode.IDLE
        self._current_goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._active_goal_label = ''
        self._last_goal_sent_time: Optional[Time] = None
        self._patrol_index = 0
        self._hold_until: Optional[Time] = None
        self._last_target: Optional[ArmorTarget] = None
        self._last_target_time: Optional[Time] = None
        self._robot_state: Optional[RobotState] = None

        self._patrol_waypoints = self._load_patrol_points()
        self._retreat_pose = self._load_retreat_pose()

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )

        self._mode_pub = self.create_publisher(SentinelMode, 'sentinel/mode', 10)
        self._fire_pub = self.create_publisher(Bool, 'fire_command', 10)
        self._nav_ready_pub = self.create_publisher(Bool, 'sentinel/nav_ready', 10)

        self.create_subscription(
            ArmorTarget,
            'armor_target',
            self._target_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            RobotState,
            'robot_state',
            self._robot_state_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            TacticalGoal,
            'sentinel/tactical_goal',
            self._tactical_goal_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            WaypointArray,
            'sentinel/patrol_waypoints',
            self._waypoints_cb,
            10,
            callback_group=self._cb_group,
        )

        self.create_service(
            SetMode,
            'sentinel/set_mode',
            self._set_mode_cb,
            callback_group=self._cb_group,
        )

        self._timer = self.create_timer(0.05, self._tick, callback_group=self._cb_group)
        self.get_logger().info(
            f'Sentinel commander started with {len(self._patrol_waypoints)} patrol waypoints'
        )

    def _load_patrol_points(self) -> List[Waypoint]:
        values = list(self.get_parameter('patrol_points').value)
        if len(values) % 3 != 0:
            self.get_logger().warn('patrol_points length is not a multiple of 3; ignoring tail values')
        points: List[Waypoint] = []
        for idx in range(0, len(values) - 2, 3):
            pose = make_pose(self._global_frame, float(values[idx]), float(values[idx + 1]), float(values[idx + 2]))
            points.append(Waypoint(pose=pose, hold_time=0.2, label=f'patrol_{idx // 3}'))
        return points

    def _load_retreat_pose(self) -> PoseStamped:
        values = list(self.get_parameter('retreat_goal').value)
        if len(values) < 3:
            values = [-2.0, 0.0, 3.14159]
        return make_pose(self._global_frame, float(values[0]), float(values[1]), float(values[2]))

    def _target_cb(self, msg: ArmorTarget) -> None:
        if not msg.visible:
            return
        self._last_target = msg
        self._last_target_time = self.get_clock().now()
        if self._mode in (Mode.IDLE, Mode.PATROL) and msg.priority >= 0.5:
            self._change_mode(Mode.TRACKING, 'target acquired')

    def _robot_state_cb(self, msg: RobotState) -> None:
        self._robot_state = msg
        if msg.hp > 0 and msg.hp <= self._retreat_hp_threshold and self._mode not in (
            Mode.RETREAT,
            Mode.EMERGENCY_STOP,
        ):
            self._change_mode(Mode.RETREAT, 'low hp')

    def _tactical_goal_cb(self, msg: TacticalGoal) -> None:
        if msg.goal_type == TacticalGoal.HOLD:
            self._change_mode(Mode.AIMING, msg.label or 'hold command')
            self._cancel_goal()
            return
        if msg.goal_type == TacticalGoal.RETREAT:
            self._retreat_pose = msg.pose
            self._change_mode(Mode.RETREAT, msg.label or 'retreat command')
            return
        if msg.goal_type == TacticalGoal.RUNE:
            self._change_mode(Mode.RUNE, msg.label or 'rune command')
            self._send_goal(msg.pose, msg.label or 'rune')
            return
        self._change_mode(Mode.TRACKING if msg.goal_type == TacticalGoal.CHASE_TARGET else Mode.PATROL, msg.label)
        self._send_goal(msg.pose, msg.label or 'tactical_goal')

    def _waypoints_cb(self, msg: WaypointArray) -> None:
        points = [
            Waypoint(pose=item.pose, hold_time=float(item.hold_time_sec), label=item.label)
            for item in msg.waypoints
        ]
        if not points:
            self.get_logger().warn('received empty patrol waypoint list')
            return
        self._patrol_waypoints = points
        self._patrol_loop = bool(msg.loop)
        self._patrol_index = 0
        self._change_mode(Mode.PATROL, 'waypoints updated')

    def _set_mode_cb(self, request: SetMode.Request, response: SetMode.Response) -> SetMode.Response:
        try:
            mode = Mode(request.mode)
        except ValueError:
            response.accepted = False
            response.message = f'unknown mode {request.mode}'
            return response
        self._change_mode(mode, request.reason)
        response.accepted = True
        response.message = f'mode changed to {mode.name}'
        return response

    def _tick(self) -> None:
        self._publish_nav_ready()
        self._publish_mode()
        self._update_goal_result()

        if self._mode == Mode.EMERGENCY_STOP:
            self._cancel_goal()
            self._publish_fire(False)
            return

        if self._mode == Mode.IDLE:
            if self._patrol_waypoints:
                self._change_mode(Mode.PATROL, 'default patrol')
            return

        if self._mode == Mode.PATROL:
            self._tick_patrol()
            return

        if self._mode == Mode.TRACKING:
            self._tick_tracking()
            return

        if self._mode == Mode.AIMING:
            self._tick_aiming()
            return

        if self._mode == Mode.RETREAT:
            if not self._goal_active():
                self._send_goal(self._retreat_pose, 'retreat')
            return

        if self._mode == Mode.RUNE:
            self._publish_fire(self._target_is_fresh() and bool(self._last_target and self._last_target.shootable))

    def _tick_patrol(self) -> None:
        if not self._patrol_waypoints:
            self._change_mode(Mode.IDLE, 'no patrol waypoints')
            return
        if self._hold_until and self.get_clock().now() < self._hold_until:
            return
        if self._goal_active():
            self._check_goal_timeout()
            return

        waypoint = self._patrol_waypoints[self._patrol_index]
        self._send_goal(waypoint.pose, waypoint.label or f'patrol_{self._patrol_index}')
        self._patrol_index += 1
        if self._patrol_index >= len(self._patrol_waypoints):
            self._patrol_index = 0 if self._patrol_loop else len(self._patrol_waypoints) - 1

    def _tick_tracking(self) -> None:
        if not self._target_is_fresh():
            self._publish_fire(False)
            self._change_mode(Mode.PATROL, 'target lost')
            return
        assert self._last_target is not None

        distance = math.hypot(self._last_target.pose.position.x, self._last_target.pose.position.y)
        if self._last_target.shootable and distance <= self._aiming_distance:
            self._change_mode(Mode.AIMING, 'target in firing window')
            self._cancel_goal()
            return

        if not self._goal_active() or self._goal_age_sec() > 1.0:
            pose = PoseStamped()
            pose.header = self._last_target.header
            pose.pose = self._last_target.pose
            if not pose.header.frame_id:
                pose.header.frame_id = self._global_frame
            self._send_goal(pose, f'track_target_{self._last_target.target_id}')

    def _tick_aiming(self) -> None:
        if not self._target_is_fresh():
            self._publish_fire(False)
            self._change_mode(Mode.PATROL, 'target lost while aiming')
            return
        target = self._last_target
        assert target is not None
        self._publish_fire(target.shootable and target.confidence >= 0.65)
        if not target.shootable:
            self._change_mode(Mode.TRACKING, 'target no longer shootable')

    def _send_goal(self, pose: PoseStamped, label: str) -> None:
        if not self._nav_client.server_is_ready():
            self.get_logger().warn('Nav2 navigate_to_pose action server is not ready')
            return

        stamped = PoseStamped()
        stamped.header = pose.header
        stamped.pose = pose.pose
        if not stamped.header.frame_id:
            stamped.header.frame_id = self._global_frame
        stamped.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = stamped
        goal.behavior_tree = ''
        self._goal_future = self._nav_client.send_goal_async(goal)
        self._goal_future.add_done_callback(self._goal_response_cb)
        self._last_goal_sent_time = self.get_clock().now()
        self._active_goal_label = label
        self.get_logger().info(
            f'sent Nav2 goal "{label}" at ({stamped.pose.position.x:.2f}, {stamped.pose.position.y:.2f})'
        )

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Nav2 rejected goal "{self._active_goal_label}"')
            self._current_goal_handle = None
            return
        self._current_goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()

    def _update_goal_result(self) -> None:
        if self._result_future is None or not self._result_future.done():
            return
        result = self._result_future.result()
        status = result.status
        label = self._active_goal_label
        self._result_future = None
        self._current_goal_handle = None
        self._active_goal_label = ''
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Nav2 goal "{label}" reached')
            self._hold_until = self.get_clock().now() + Duration(nanoseconds=200_000_000)
        else:
            self.get_logger().warn(f'Nav2 goal "{label}" finished with status {status}')

    def _cancel_goal(self) -> None:
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
        self._current_goal_handle = None
        self._result_future = None
        self._active_goal_label = ''

    def _goal_active(self) -> bool:
        return self._current_goal_handle is not None and self._result_future is not None

    def _goal_age_sec(self) -> float:
        if self._last_goal_sent_time is None:
            return float('inf')
        return (self.get_clock().now() - self._last_goal_sent_time).nanoseconds * 1e-9

    def _check_goal_timeout(self) -> None:
        if self._goal_age_sec() > self._goal_timeout:
            self.get_logger().warn(f'goal "{self._active_goal_label}" timed out; canceling')
            self._cancel_goal()

    def _target_is_fresh(self) -> bool:
        if self._last_target is None or self._last_target_time is None:
            return False
        age = (self.get_clock().now() - self._last_target_time).nanoseconds * 1e-9
        return age <= self._target_timeout

    def _change_mode(self, mode: Mode, reason: str = '') -> None:
        if self._mode == mode:
            return
        self.get_logger().info(f'mode {self._mode.name} -> {mode.name}: {reason}')
        self._mode = mode
        if mode in (Mode.IDLE, Mode.AIMING, Mode.EMERGENCY_STOP):
            self._cancel_goal()

    def _publish_mode(self) -> None:
        msg = SentinelMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame
        msg.mode = int(self._mode)
        msg.reason = self._active_goal_label
        self._mode_pub.publish(msg)

    def _publish_fire(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self._fire_pub.publish(msg)

    def _publish_nav_ready(self) -> None:
        msg = Bool()
        msg.data = self._nav_client.server_is_ready()
        self._nav_ready_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SentinelCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
