import time
import random
import rclpy
from rclpy.node import Node
from smart_shop_interfaces.srv import AuthorizePayment

class PaymentServer(Node):
    def __init__(self):
        super().__init__('payment_server')
        
        self.srv = self.create_service(
            AuthorizePayment,
            'authorize_payment',
            self.cb_authorize
        )
        
        self.get_logger().info("PaymentServer ready.")

    def cb_authorize(self, request, response):
        self.get_logger().info(
            f"Payment request: order_id={request.order_id}, "
            f"amount={request.amount} {request.currency}"
        )
        
        # 실제 승인 지연 상황 흉내
        time.sleep(0.5)
        
        if request.amount > 0:
            response.approved = True
            response.auth_code = f"AUTH-{random.randint(100000, 999999)}"
            response.reason = "ok"
            self.get_logger().info(
                f"Payment approved: auth={response.auth_code}"
            )
        else:
            response.approved = False
            response.auth_code = ""
            response.reason = "invalid_amount"
            self.get_logger().warn("Payment rejected: invalid amount")
            
        return response

def main():
    rclpy.init()
    node = PaymentServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()