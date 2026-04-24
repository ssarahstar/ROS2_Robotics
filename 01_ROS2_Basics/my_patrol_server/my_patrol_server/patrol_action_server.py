import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_action_interfaces.action import PatrolTest

class PatrolActionServer(Node):
    def __init__(self):
        super().__init__('turtle_patrol_server')
        self.cb = ReentrantCallbackGroup()

        # 상태 변수
        self.pose = None
        self.goal_handle = None
        self.waypoints = []
        self.idx = 0
        self.tolerance = 0.0
        self.phase = 'IDLE'

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        self.server = ActionServer(
            self,
            PatrolTest,
            'patrol',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb
        )

        self.timer = self.create_timer(0.05, self.timer_cb, callback_group=self.cb)
        self.get_logger().info('Timer-based Patrol Action Server ready')

    def pose_cb(self, msg):
        self.pose = msg

    def execute_callback(self, goal_handle):
        # 실제 로직은 타이머나 별도 루프에서 처리하며, 여기서는 결과 객체를 반환합니다.
        return PatrolTest.Result()

    def goal_callback(self, goal_request):
        self.get_logger().info('Goal request received')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle
        self.waypoints = list(goal_handle.request.waypoints)
        self.tolerance = goal_handle.request.tolerance
        self.idx = 0
        self.phase = 'ROTATE'

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Cancel requested')
        return CancelResponse.ACCEPT
    
    def timer_cb(self):
        if self.goal_handle is None or self.pose is None:
            return

        if self.goal_handle.is_cancel_requested:
            self.stop()
            self.goal_handle.canceled()
            self.reset()
            return

        if self.idx >= len(self.waypoints):
            self.stop()
            self.goal_handle.succeed()
            self.reset()
            return

        target = self.waypoints[self.idx]
        dx = target.x - self.pose.x
        dy = target.y - self.pose.y
        dist = math.hypot(dx, dy)

        fb = PatrolTest.Feedback()
        fb.current_index = self.idx
        fb.remaining_distance = dist
        self.goal_handle.publish_feedback(fb)

        target_angle = math.atan2(dy, dx)
        ang_err = math.atan2(
            math.sin(target_angle - self.pose.theta),
            math.cos(target_angle - self.pose.theta)
        )

        cmd = Twist()

        if self.phase == 'ROTATE':
            if abs(ang_err) > 0.2:
                cmd.angular.z = 2.0 * ang_err
            else:
                self.stop()
                self.phase = 'MOVE'
                return

        elif self.phase == 'MOVE':
            if dist < self.tolerance:
                self.get_logger().info(
                    f'Waypoint {self.idx} reached '
                    f'(x={target.x:.2f}, y={target.y:.2f})'
                )
                self.stop()
                self.idx += 1
                self.phase = 'ROTATE'
                return

            cmd.linear.x = min(1.0, dist)
            cmd.angular.z = 1.5 * ang_err

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def reset(self):
        self.goal_handle = None
        self.waypoints.clear()
        self.idx = 0
        self.phase = 'IDLE'

def main():
    rclpy.init()
    node = PatrolActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()