import rclpy
from rclpy.node import Node
from smart_shop_interfaces.srv import DiscountApply

class DiscountServer(Node):
    def __init__(self):
        super().__init__('discount_server')
        self.srv = self.create_service(
            DiscountApply,
            'apply_discount',
            self.cb_apply_discount
        )
        self.get_logger().info("DiscountServer ready.")

    def cb_apply_discount(self, request, response):
        item = request.item_id
        amount = request.original_amount
        rate = 0
        
        # 할인 정책 적용
        if item == 'cup':
            rate = 10
        elif item == 'snack':
            rate = 20
        else:
            rate = 0
            
        discounted = int(amount * (100 - rate) / 100)
        
        response.discounted_amount = discounted
        response.discount_rate = rate
        response.reason = "discount applied"
        
        self.get_logger().info(
            f"[cb_apply_discount] item={item}, original={amount}, "
            f"rate={rate}%, discounted={discounted}"
        )
        return response

def main():
    rclpy.init()
    node = DiscountServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
