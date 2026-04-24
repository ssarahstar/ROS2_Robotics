import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState   #sensor_msgs 패키지의 JointState 메시지 사용

WARN_THRESHOLD = 1.0             # 경고 기준 각도 (rad)

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.create_subscription(
            JointState,          # 메시지 타입
            '/joint_states',     # 구독할 토픽 이름
            self.callback,
            10
        )

    def callback(self, msg):
        # msg.name  : 관절 이름 리스트  
        # msg.position : 각 관절 position 리스트 
        
        # name과 position을 묶어 딕셔너리로 생성
        pairs = dict(zip(msg.name, msg.position))              
        
        line = '[joint_monitor]'
        for name, pos in pairs.items():
            line += f'  {name}: {pos:.3f}'
        self.get_logger().info(line)

        # 경고 조건: 절댓값이 WARN_THRESHOLD(1.0) 이상인 관절 출력
        for name, pos in pairs.items():
            if abs(pos) >= WARN_THRESHOLD:
                self.get_logger().warn(
                    f'[WARNING]  {name} 이(가) {WARN_THRESHOLD} rad 를 초과! (현재: {pos:.3f})'
                )

def main():
    rclpy.init()
    node = JointMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()