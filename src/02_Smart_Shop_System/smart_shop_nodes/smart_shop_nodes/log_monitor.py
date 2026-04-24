import rclpy
from rclpy.node import Node
from smart_shop_interfaces.msg import OrderLog

class LogMonitor(Node):
    def __init__(self):
        super().__init__('log_monitor')
        
        # /order_log 토픽 Subscriber 생성
        self.subscription = self.create_subscription(
            OrderLog,
            'order_log',
            self.listener_callback,
            10
        )
        self.subscription 
        
        # 성공/실패 누적 카운트를 멤버 변수로 초기화
        self.success_count = 0
        self.fail_count = 0
        
        self.get_logger().info("Log Monitor started. Waiting for order logs...")

    def listener_callback(self, msg):
        # 성공 여부에 따른 상태 문자열 지정 및 누적 카운트 증가
        if msg.success:
            status_str = "SUCCESS"
            self.success_count += 1
        else:
            status_str = "FAILED "  
            self.fail_count += 1
            
        # 요구된 형식으로 로그 포맷팅
        log_output = (
            f"[{msg.timestamp}] {msg.order_id} | {msg.item_id} x{msg.quantity} | "
            f"{msg.amount} KRW | {status_str} | {msg.detail}"
        )
        
        # 터미널 출력
        self.get_logger().info(log_output)
        
        # 성공/실패 누적 카운트 출력
        self.get_logger().info(
            f" => [Total Counts] SUCCESS: {self.success_count} | FAILED: {self.fail_count}\n"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LogMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()