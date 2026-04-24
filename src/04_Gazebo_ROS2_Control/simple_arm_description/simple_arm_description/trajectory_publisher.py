import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  
from builtin_interfaces.msg import Duration

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(
            JointTrajectory,                                        
            '/joint_trajectory_controller/joint_trajectory',        
            10
        )
        
        self.timer = self.create_timer(1.0, self.publish_once)  

    def publish_once(self):
        self.timer.cancel()  

        msg = JointTrajectory()           
        msg.joint_names = ['joint1_z', 'joint1_y', 'joint2', 'joint3']

        pt = JointTrajectoryPoint()
        pt.positions = [0.5, 0.3, -0.5, 0.2]    
        pt.time_from_start = Duration(sec=2, nanosec=0)  

        msg.points = [pt]
        self.pub.publish(msg)                  
        
        self.get_logger().info('[trajectory_publisher] 목표 자세 발행 완료!')
        raise SystemExit

def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
