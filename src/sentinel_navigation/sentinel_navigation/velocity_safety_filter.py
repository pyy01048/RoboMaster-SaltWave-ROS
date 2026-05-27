from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sentinel_msgs.msg import RobotState, SentinelMode


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class VelocitySafetyFilter(Node):
    """Clamp and gate Nav2 velocity before it reaches the chassis driver."""

    def __init__(self) -> None:
        super().__init__('velocity_safety_filter')
        self.declare_parameter('max_vx', 1.5)
        self.declare_parameter('max_vy', 1.5)
        self.declare_parameter('max_wz', 3.5)
        self.declare_parameter('power_limit_scale', 0.55)
        self.declare_parameter('cmd_timeout_sec', 0.25)
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', 'cmd_vel_safe')

        self._max_vx = float(self.get_parameter('max_vx').value)
        self._max_vy = float(self.get_parameter('max_vy').value)
        self._max_wz = float(self.get_parameter('max_wz').value)
        self._power_limit_scale = float(self.get_parameter('power_limit_scale').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._last_cmd_time = self.get_clock().now()
        self._latest_cmd = Twist()
        self._mode = SentinelMode.IDLE
        self._robot_state = RobotState()
        self._robot_state.chassis_online = True
        self._robot_state.localization_ok = True

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._cmd_cb, 10)
        self.create_subscription(SentinelMode, 'sentinel/mode', self._mode_cb, 10)
        self.create_subscription(RobotState, 'robot_state', self._state_cb, 10)
        self.create_timer(0.02, self._tick)

    def _cmd_cb(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _mode_cb(self, msg: SentinelMode) -> None:
        self._mode = msg.mode

    def _state_cb(self, msg: RobotState) -> None:
        self._robot_state = msg

    def _tick(self) -> None:
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self._cmd_timeout:
            self._pub.publish(Twist())
            return

        if self._mode == SentinelMode.EMERGENCY_STOP:
            self._pub.publish(Twist())
            return

        if not self._robot_state.chassis_online or not self._robot_state.localization_ok:
            self._pub.publish(Twist())
            return

        scale = self._compute_power_scale()
        cmd = Twist()
        cmd.linear.x = clamp(self._latest_cmd.linear.x, self._max_vx * scale)
        cmd.linear.y = clamp(self._latest_cmd.linear.y, self._max_vy * scale)
        cmd.angular.z = clamp(self._latest_cmd.angular.z, self._max_wz * scale)
        self._pub.publish(cmd)

    def _compute_power_scale(self) -> float:
        limit = self._robot_state.chassis_power_limit
        if math.isclose(limit, 0.0):
            return 1.0
        ratio = self._robot_state.chassis_power / max(limit, 1.0)
        if ratio < 0.8:
            return 1.0
        if ratio > 1.0:
            return self._power_limit_scale
        return 1.0 - (ratio - 0.8) * (1.0 - self._power_limit_scale) / 0.2


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocitySafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
