import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

PHASES = [
    (5.0,  [0.0, 0.0, 0.0, 0.0]),   # Phase 0: 대기 (5초)
    (5.0,  [8.0, 0.0, 0.0, 0.0]),   # Phase 1: joint1_z 회전
    (5.0,  [0.0, 8.0, 0.0, 0.0]),   # Phase 2: joint1_y 들어올리기
    (5.0,  [0.0, 0.0, 8.0, 0.0]),   # Phase 3: joint2 구부리기
    (None, [0.0, 0.0, 0.0, 0.0]),   # Phase 4: 정지 (무한)
]

class ArmSequencer(Node):
    def __init__(self):
        super().__init__('arm_sequencer')
        self.pub = self.create_publisher(
            Float64MultiArray,          # ① 메시지 타입
            '/effort_controller/commands', # ② 토픽 이름
            10
        )
        self.timer = self.create_timer(0.1, self.callback)  # ③ 주기(초): 10Hz이므로 0.1
        self.start_time = self.get_clock().now()
        self.current_phase = 0
        self.phase_start   = self.get_clock().now()

    def callback(self):
        duration, data = PHASES[self.current_phase]
        
        # ④ 현재 Phase가 끝났는지 확인하고 다음 Phase로 전환하는 로직
        if duration is not None:
            elapsed = (self.get_clock().now() - self.phase_start).nanoseconds / 1e9
            if elapsed >= duration:     # ④ 지정된 지속 시간이 지났는지 확인
                self.current_phase += 1
                self.phase_start = self.get_clock().now()
                duration, data = PHASES[self.current_phase]

        msg = Float64MultiArray()
        msg.data = data
        self.pub.publish(msg)           # ⑤ 퍼블리시
        
        self.get_logger().info(
            f'[Phase {self.current_phase}]  data={data}'
        )

def main():
    rclpy.init()
    node = ArmSequencer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()