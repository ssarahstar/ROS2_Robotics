import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

STOP_THRESHOLD = 0.8   

class AutoStop(Node):
    def __init__(self):
        super().__init__('auto_stop')
        
        # /joint_states 구독자 생성 (콜백: self.joint_cb)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # /effort_controller/commands 퍼블리셔 생성
        self.pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        # 0.5초마다 현재 joint3 position 출력용 타이머 생성
        self.timer = self.create_timer(0.5, self.log_cb)
        
        self.joint3_pos = 0.0
        self.stopped = False

    def joint_cb(self, msg):
        # joint3 position 추출 (name 리스트에서 인덱스를 찾아 position에 적용)
        try:
            idx = msg.name.index('joint3')
            self.joint3_pos = msg.position[idx]
        except ValueError:
            return

        # 임계값 초과 시 토크 해제 후 노드 종료
        if not self.stopped and abs(self.joint3_pos) > STOP_THRESHOLD:
            self.stopped = True
            
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0, 0.0, 0.0]
            
            # 퍼블리시
            self.pub.publish(stop_msg)
            
            self.get_logger().info(
                f'[AUTO STOP]  joint3 = {self.joint3_pos:.3f} rad 도달 → 전체 토크 해제'
            )
            
            # 노드 종료 (spin을 멈추기 위해 SystemExit 예외 발생)
            raise SystemExit

    def log_cb(self):
        if not self.stopped:
            self.get_logger().info(f'joint3 현재 위치: {self.joint3_pos:.3f} rad')

def main():
    rclpy.init()
    node = AutoStop()
    try:
        rclpy.spin(node)
    except SystemExit:                 # raise SystemExit 발생 시 안전하게 종료되도록 예외 처리
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
