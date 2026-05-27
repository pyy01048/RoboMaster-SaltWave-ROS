#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus

class PatrolWaypoints(Node):
    def __init__(self):
        super().__init__('patrol_waypoints')
        
        self.declare_parameter('waypoints', [])
        self.declare_parameter('loop_patrol', True)
        self.declare_parameter('goal_tolerance', 0.3)
        
        waypoints_param = self.get_parameter('waypoints').value
        self.waypoints = self.parse_waypoints(waypoints_param)
        self.loop_patrol = self.get_parameter('loop_patrol').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        self.current_waypoint_idx = 0
        self.is_patrolling = False
        
        self.timer = self.create_timer(1.0, self.start_patrol)
        
        self.get_logger().info(f'Patrol Manager initialized with {len(self.waypoints)} waypoints')
    
    def parse_waypoints(self, waypoints_param):
        waypoints = []
        for wp in waypoints_param:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)
        
        if len(waypoints) == 0:
            default_wps = [
                [2.0, 0.0],
                [2.0, 2.0],
                [0.0, 2.0],
                [0.0, 0.0]
            ]
            for wp in default_wps:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = wp[0]
                pose.pose.position.y = wp[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                waypoints.append(pose)
        
        return waypoints
    
    def start_patrol(self):
        if self.is_patrolling or len(self.waypoints) == 0:
            return
        
        self.timer.cancel()
        self.is_patrolling = True
        
        self.get_logger().info('Starting patrol...')
        self.send_waypoints()
    
    def send_waypoints(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints action server not available!')
            return
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints
        
        self.get_logger().info(f'Sending {len(self.waypoints)} waypoints...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Current waypoint index: {feedback.current_waypoint}')
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Patrol completed successfully!')
        else:
            self.get_logger().warn(f'Patrol failed with status: {status}')
        
        if self.loop_patrol:
            self.get_logger().info('Restarting patrol loop...')
            self.send_waypoints()
        else:
            self.is_patrolling = False

def main(args=None):
    rclpy.init(args=args)
    node = PatrolWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
