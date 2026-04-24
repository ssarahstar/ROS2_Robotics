#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw"
COLOR_INFO_TOPIC  = "/camera/camera/color/camera_info"


class ImageViewerNode(Node):
    def __init__(self):
        super().__init__("image_viewer")

        self.bridge = CvBridge()
        self.camera_info = None

        self.create_subscription(
            CameraInfo,
            COLOR_INFO_TOPIC,
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            Image,
            COLOR_IMAGE_TOPIC,
            self.image_callback,
            10,
        )

        self.get_logger().info("depth_pkg image viewer started")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]

            self.get_logger().info(
                f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
            )

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Color", frame)

        if cv2.waitKey(1) == 27:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()