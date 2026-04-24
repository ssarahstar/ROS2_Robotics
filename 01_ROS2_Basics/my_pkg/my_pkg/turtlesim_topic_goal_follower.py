import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point

# 각도를 -pi에서 pi 사이로 정규화하는 함수 [cite: 415, 481]
def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class TopicGoalFollower(Node):
    def __init__(self):
        super().__init__("topic_goal_follower")
        
        # 구독 및 발행 설정 [cite: 439, 440, 442, 490, 491]
        self.sub_pose = self.create_subscription(Pose, "/turtle1/pose", self.cb_pose, 10)
        self.sub_goal = self.create_subscription(Point, "/goal", self.cb_goal, 10)
        self.pub_cmd = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # 상태 변수 및 제어 파라미터 초기화 [cite: 443, 444, 510, 511, 516]
        self.pose = None
        self.goal = None
        self.k_lin = 1.0
        self.k_ang = 3.0
        self.max_lin = 2.0
        self.max_ang = 4.0
        self.stop_dist = 0.2
        
        # 0.05초마다 tick 함수 실행 [cite: 461, 518]
        self.create_timer(0.05, self.tick)

    def cb_pose(self, msg): 
        self.pose = msg 

    def cb_goal(self, msg): 
        self.goal = msg 
        self.get_logger().info(f"New goal: x={msg.x:.2f}, y={msg.y:.2f}") 

    def clamp(self, x, lo, hi): 
        return max(lo, min(hi, x)) 

    def tick(self):
        if self.pose is None or self.goal is None:
            return 
        
        # 거리 및 오차 계산 [cite: 446, 447, 449, 539, 542]
        dx = self.goal.x - self.pose.x
        dy = self.goal.y - self.pose.y
        dist = math.hypot(dx, dy)
        cmd = Twist()

        # 목표 도착 시 정지 [cite: 451, 452, 543, 544]
        if dist < self.stop_dist:
            self.pub_cmd.publish(cmd)
            return

        # 각도 오차 계산 및 제어 [cite: 453, 454, 553, 554]
        target_theta = math.atan2(dy, dx)
        err_theta = wrap_to_pi(target_theta - self.pose.theta)

        if abs(err_theta) > 0.7: # 회전 우선 [cite: 464, 551]
            cmd.angular.z = self.clamp(self.k_ang * err_theta, -self.max_ang, self.max_ang) 
        else: # 전진 및 회전 [cite: 467, 557]
            cmd.linear.x = self.clamp(self.k_lin * dist, 0.0, self.max_lin) [cite: 469, 558]
            cmd.angular.z = self.clamp(self.k_ang * err_theta, -self.max_ang, self.max_ang) 
        
        self.pub_cmd.publish(cmd) 

def main():
    rclpy.init() 
    node = TopicGoalFollower() 
    try:
        rclpy.spin(node) 
    finally:
        node.destroy_node() 
        rclpy.shutdown() 

if __name__ == '__main__':
    main()