import rclpy
from rclpy.node import Node
from smart_shop_interfaces.srv import CheckStock

class StockServer(Node):
    def __init__(self):
        super().__init__('stock_server')
        self.srv = self.create_service(
            CheckStock,
            'check_stock',
            self.cb_check_stock
        )
        
        # 간단한 재고 DB (메모리)
        self.stock = {
            "cup": 12,
            "bottle": 3,
            "snack": 25
        }
        
        self.get_logger().info("StockServer ready.")

    def cb_check_stock(self, request, response):
        item = request.item_id
        qty = request.quantity
        
        self.get_logger().info(
            f"[cb_check_stock] item={item}, qty={qty}"
        )
        
        if item not in self.stock:
            response.available = False
            response.remaining = 0
            response.reason = "unknown_item"
            self.get_logger().warning(f"Item not found: {item}")
            return response
            
        remaining = self.stock[item]
        
        if remaining >= qty:
            response.available = True
            self.stock[item] -= qty  # 실제 재고 차감
            response.remaining = self.stock[item]
            response.reason = "ok"
            self.get_logger().info(
                f"Stock approved: {item}, remaining={self.stock[item]}"
            )
        else:
            response.available = False
            response.remaining = remaining
            response.reason = "insufficient_stock"
            self.get_logger().warning(
                f"Stock insufficient: {item}, requested={qty}, available={remaining}"
            )
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()