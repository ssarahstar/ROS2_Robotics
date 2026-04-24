import sys
import rclpy
from rclpy.node import Node
from smart_shop_interfaces.srv import PlaceOrder

class OrderClient(Node):
    def __init__(self):
        super().__init__('order_client')
        self.cli = self.create_client(PlaceOrder, 'place_order')
        
    def send(self, order_id, item_id, qty, amount, currency):
        # 서비스 대기
        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service not available")
            return None
            
        # 요청 생성
        req = PlaceOrder.Request()
        req.order_id = order_id
        req.item_id = item_id
        req.quantity = qty
        req.amount = amount
        req.currency = currency
        
        self.get_logger().info(
            f"Sending order: id={order_id}, item={item_id}, "
            f"qty={qty}, amount={amount} {currency}"
        )
        
        # 비동기 호출 + 완료까지 대기 (동기처럼 사용)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        # 결과 확인
        if not future.done():
            self.get_logger().error("Order request timed out")
            return None
            
        if future.exception() is not None:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return None
            
        return future.result()

def main():
    rclpy.init()
    node = OrderClient()
    
    # 실행 예:
    # ros2 run smart_shop_nodes order_client ORD-1 cup 2 1200 KRW
    if len(sys.argv) != 6:
        print(
            "\nUsage:\n"
            "  ros2 run smart_shop_nodes order_client "
            "<order_id> <item_id> <qty> <amount> <currency>\n\n"
            "Example:\n"
            "  ros2 run smart_shop_nodes order_client ORD-1 cup 2 1200 KRW\n"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
        
    order_id = sys.argv[1]
    item_id = sys.argv[2]
    qty = int(sys.argv[3])
    amount = int(sys.argv[4])
    currency = sys.argv[5]
    
    res = node.send(order_id, item_id, qty, amount, currency)
    
    print("\n====== ORDER RESULT ======")
    if res is None:
        print("Order failed (no response)")
    else:
        print(f"success          : {res.success}")
        print(f"status           : {res.status}")
        print(f"detail           : {res.detail}")
        print(f"remaining_stock  : {res.remaining_stock}")
        
        # 필드 존재 안전 처리
        if hasattr(res, "payment_auth_code"):
            print(f"payment_auth_code: {res.payment_auth_code}")
        else:
            print("payment_auth_code: (field not available)")
            
    print("==========================\n")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()