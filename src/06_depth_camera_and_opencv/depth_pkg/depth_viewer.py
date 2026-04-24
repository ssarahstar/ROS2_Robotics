#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

# === 토픽 이름 (ros2 topic list로 확인한 걸 그대로 넣기) ===
COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEPTH_IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw"
CAMERA_INFO_TOPIC = "/camera/camera/depth/camera_info"
# =========================================================


class DepthCenterViewer(Node):
    """
    1) 컬러 이미지 구독 → 화면 중앙에 점 표시
    2) depth 이미지 구독 → 중앙 픽셀 depth 읽기
    3) depth용 camera_info 구독 → fx, fy, cx, cy 추출
    4) 중앙 픽셀 (X, Y, Z) 카메라 좌표 계산
    """

    def __init__(self):
        super().__init__("depth_center_viewer")

        self.bridge = CvBridge()

        # camera intrinsics (depth 기준)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # 가장 최근 depth 프레임
        self.depth_frame = None

        # 로그 출력 간격 제어
        self.print_counter = 0

        # --- 구독 설정 ---
        self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self.depth_image_callback,
            10,
        )

        self.create_subscription(
            Image,
            COLOR_IMAGE_TOPIC,
            self.color_image_callback,
            10,
        )

        self.get_logger().info("depth_pkg: DepthCenterViewer node started")

    # ---------- 콜백들 ----------

    def camera_info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

            self.get_logger().info(
                f"[CameraInfo(depth)] fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.cx:.2f}, cy={self.cy:.2f}"
            )

    def depth_image_callback(self, msg: Image):
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth_frame = depth_img

        # 디버그용: 첫 프레임에서 한 번만 찍기
        h, w = depth_img.shape[:2]
        if self.print_counter == 0:
            self.get_logger().info(f"[Depth] first frame shape w={w}, h={h}")

    def color_image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h_c, w_c, _ = frame.shape
        u = w_c // 2
        v = h_c // 2

        if (
            self.depth_frame is not None
            and self.fx is not None
            and self.fy is not None
        ):
            # 해상도 848x480으로 같다고 가정
            h_d, w_d = self.depth_frame.shape[:2]

            # 방어적 체크
            if (h_c, w_c) != (h_d, w_d):
                # 그래도 혹시 다르면 최소한 범위 안에서만 인덱싱
                u_depth = min(u, w_d - 1)
                v_depth = min(v, h_d - 1)
            else:
                u_depth = u
                v_depth = v

            depth_raw = self.depth_frame[v_depth, u_depth]

            if depth_raw > 0:
                Z = float(depth_raw) / 1000.0  # mm → m
                X = (u_depth - self.cx) * Z / self.fx
                Y = (v_depth - self.cy) * Z / self.fy

                self.print_counter += 1
                if self.print_counter % 30 == 0:
                    self.get_logger().info(
                        f"Center pixel (u={u_depth}, v={v_depth}), "
                        f"depth_raw={depth_raw} -> "
                        f"X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m"
                    )

                # 유효 depth → 빨간 점
                cv2.circle(frame, (u, v), 6, (0, 0, 255), -1)
            else:
                # depth=0 → 노란 점
                cv2.circle(frame, (u, v), 6, (0, 255, 255), -1)
        else:
            # depth/camera_info 준비 전 → 파란 점
            cv2.circle(frame, (u, v), 6, (255, 0, 0), -1)

        cv2.imshow("Color (center pixel depth)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DepthCenterViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()