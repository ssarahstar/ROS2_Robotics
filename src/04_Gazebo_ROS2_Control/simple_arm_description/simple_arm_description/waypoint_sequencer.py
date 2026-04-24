import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['joint1_z', 'joint1_y', 'joint2', 'joint3']


WAYPOINTS = [
    ([0.0,  0.0,  0.0, 0.0], 3),   
    ([0.5,  0.0,  0.0, 0.0], 3),   
    ([0.5,  0.3,  0.0, 0.0], 3),   
    ([0.5,  0.3, -0.5, 0.2], 2),   
]

class WaypointSequencer(Node):
    def __init__(self):
        super().__init__('waypoint_sequencer')
        self.pub = self.create_publisher(
            JointTrajectory,                              
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.timer  = self.create_timer(1.0, self.tick)      
        
        self.wp_idx = 0      
        self.ticks  = 0       
        self.done   = False 

    def publish_wp(self, idx):
        positions, _ = WAYPOINTS[idx]
        
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        
        pt = JointTrajectoryPoint()
        pt.positions       = positions                       
        pt.time_from_start = Duration(sec=2, nanosec=0)      
        
        msg.points = [pt]
        self.pub.publish(msg)                                
        
        names = dict(zip(JOINT_NAMES, positions))
        self.get_logger().info(
            f'[WP {idx}/{len(WAYPOINTS)-1}]'
            f' joint1_z={names["joint1_z"]}, joint1_y={names["joint1_y"]},'
            f' joint2={names["joint2"]}, joint3={names["joint3"]}'
        )

    def tick(self):
        if self.done:
            return
            
       
        if self.wp_idx == 0 and self.ticks == 0:
            self.publish_wp(0)
            self.ticks += 1
            return
            
        _, wait = WAYPOINTS[self.wp_idx]
        self.ticks += 1
        
      
        if self.ticks > wait:
            self.wp_idx += 1
            self.ticks  = 0
            
            if self.wp_idx >= len(WAYPOINTS):                
                self.get_logger().info('모든 waypoint 완료. 노드 종료.')
                self.done = True
                raise SystemExit
                
            self.publish_wp(self.wp_idx)                     

def main():
    rclpy.init()
    node = WaypointSequencer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()