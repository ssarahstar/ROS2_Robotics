import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')

        #파라미터 선언 (기본값 설정)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 0.0)

        #현재 파라미터 값 읽기
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        #Publisher 생성
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        #타이머 생성
        self.timer = self.create_timer(0.5, self.timer_callback)

        #동적 파라미터 콜백 등록
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        #파라미터가 변경될 때마다 호출되는 함수
        for param in params:
            if param.name == 'linear_speed':
                self.linear_speed = float(param.value)
            elif param.name == 'angular_speed':
                self.angular_speed = float(param.value)
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        #0.5초마다 현재 속도값으로 Twist 메시지 발행
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SpeedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()