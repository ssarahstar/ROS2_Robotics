import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  

TARGETS = ['joint1_z', 'joint1_y', 'joint2', 'joint3']
PRINT_HZ = 2.0  

class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')
        self.data = {}  

        self.create_subscription(
            JointState,       
            '/joint_states', 
            self.callback,
            10
        )

        
        self.create_timer(1.0 / PRINT_HZ, self.print_state)  

    def callback(self, msg):
        
        self.data = {n: (p, v) for n, p, v in
                     zip(msg.name, msg.position, msg.velocity)}

    def print_state(self):
        if not self.data:    
            return

        for name in TARGETS:
            if name not in self.data:
                continue
            pos, vel = self.data[name]   
            self.get_logger().info(
                f'[state_monitor] {name:<10} pos: {pos:.3f}  vel: {vel:.3f}'
            )

def main():
    rclpy.init()
    node = StateMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()