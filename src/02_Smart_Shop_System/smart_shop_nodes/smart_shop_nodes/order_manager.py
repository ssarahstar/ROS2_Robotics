import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime

# DiscountApply 인터페이스 추가 임포트 
from smart_shop_interfaces.srv import (
    PlaceOrder,
    CheckStock,
    AuthorizePayment,
    DiscountApply
)
from smart_shop_interfaces.msg import OrderLog


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')

        # 재진입 허용 콜백 그룹 (동기 서비스 체인 데드락 방지) 
        self.cb_group = ReentrantCallbackGroup()

        # 1. PlaceOrder 서비스 서버 
        self.srv = self.create_service(
            PlaceOrder,
            'place_order',
            self.cb_place_order,
            callback_group=self.cb_group
        )

        # 2. 내부 서비스 클라이언트들 생성
        self.stock_cli = self.create_client(
            CheckStock,
            'check_stock',
            callback_group=self.cb_group
        )
        self.pay_cli = self.create_client(
            AuthorizePayment,
            'authorize_payment',
            callback_group=self.cb_group
        )
        # 할인 서비스 클라이언트 추가 (과제 1-2-C) 
        self.disc_cli = self.create_client(
            DiscountApply,
            'apply_discount',
            callback_group=self.cb_group
        )
        self.log_pub = self.create_publisher(
            OrderLog, 'order_log', 10)

        self.get_logger().info("OrderManager ready (with Discount Service and OrderLog.")

    def wait_service_or_fail(self, client, name, timeout_sec=3.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"Service not available: {name}")
            return False
        return True
    
    def publish_order_log(self, request, response):
        log_msg = OrderLog()
        log_msg.order_id = request.order_id
        log_msg.item_id = request.item_id
        log_msg.quantity = request.quantity
        log_msg.amount = request.amount
        log_msg.success = response.success
        log_msg.status = response.status
        log_msg.detail = response.detail
        log_msg.timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.log_pub.publish(log_msg)
        
    
    def cb_place_order(self, request, response):
        self.get_logger().info(
            f"Order received: id={request.order_id}, item={request.item_id}, qty={request.quantity}"
        )

        # 모든 서비스 가용성 확인 
        if not all([
            self.wait_service_or_fail(self.stock_cli, 'check_stock'),
            self.wait_service_or_fail(self.disc_cli, 'apply_discount'),
            self.wait_service_or_fail(self.pay_cli, 'authorize_payment')
        ]):
            response.success = False
            response.status = "dependency_unavailable"
            self.publish_order_log(request, response)
            return response

        # --- STEP 1: 재고 확인 (기존) ---
        stock_req = CheckStock.Request()
        stock_req.item_id = request.item_id
        stock_req.quantity = request.quantity
        stock_res = self.stock_cli.call(stock_req)

        if not stock_res.available:
            response.success = False
            response.status = "rejected"
            response.detail = f"stock: {stock_res.reason}"
            response.remaining_stock = stock_res.remaining
            self.publish_order_log(request, response)
            return response

        # --- STEP 2: 할인 계산 (신규 과제)  ---
        disc_req = DiscountApply.Request()
        disc_req.item_id = request.item_id
        disc_req.original_amount = request.amount
        
        disc_res = self.disc_cli.call(disc_req)
        # 할인 결과 로그 기록
        self.get_logger().info(f"Discount: {disc_res.discount_rate}%, Final Price: {disc_res.discounted_amount}")

        # --- STEP 3: 결제 승인 (할인된 금액 적용) ---
        pay_req = AuthorizePayment.Request()
        pay_req.order_id = request.order_id
        pay_req.amount = disc_res.discounted_amount  # 할인 서버에서 받은 금액 사용
        pay_req.currency = request.currency
        pay_res = self.pay_cli.call(pay_req)

        if not pay_res.approved:
            response.success = False
            response.status = "rejected"
            response.detail = f"payment: {pay_res.reason}"
            response.remaining_stock = stock_res.remaining
            self.publish_order_log(request, response)
            return response

        # --- 성공 응답 구성 (할인 정보 포함) ---
        response.success = True
        response.status = "success"
        # 요구사항에 따른 detail 포맷 설정
        response.detail = f"order accepted (discount: {disc_res.discount_rate}%, final: {disc_res.discounted_amount} {request.currency})"
        response.remaining_stock = stock_res.remaining
        response.payment_auth_code = pay_res.auth_code
        self.publish_order_log(request, response)

        return response

def main():
    rclpy.init()
    node = OrderManager()
    # 멀티스레드 실행기 필수 
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()