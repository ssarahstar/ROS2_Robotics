#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

# === 토픽 이름 (ros2 topic list로 확인한 걸 그대로 넣기) ===
COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEPTH_IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw"
CAMERA_INFO_TOPIC = "/camera/camera/depth/camera_info"
# =========================================================

WINDOW_NAME = "Color (click pixel depth)"

# 마우스 클릭 좌표를 저장할 전역 변수
g_click_u = None
g_click_v = None


def mouse_callback(event, x, y, flags, param):
    """
    마우스로 클릭한 픽셀 좌표(u, v)를 전역 변수에 저장.
    """
    global g_click_u, g_click_v
    if event == cv2.EVENT_LBUTTONDOWN:
        g_click_u = x
        g_click_v = y
        print(f"[Mouse] 클릭한 픽셀: u={g_click_u}, v={g_click_v}")


class DepthClickViewer(Node):
    """
    1) 컬러 이미지 구독 → 클릭한 점(없으면 화면 중앙)에 동그라미 표시
    2) depth 이미지 구독 → 해당 픽셀의 depth 읽기
    3) depth용 camera_info 구독 → fx, fy, cx, cy 추출
    4) 클릭한 픽셀 (X, Y, Z) 카메라 좌표 계산
    """

    def __init__(self):
        super().__init__("depth_click_viewer")

        self.bridge = CvBridge()

        # camera intrinsics (depth 기준)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.depth_frame = None  # 최신 depth 이미지 (numpy)
        self.print_counter = 0   # 로그 출력 주기 조절용

        # --- Subscriber 설정 ---
        self.color_sub = self.create_subscription(
            Image,
            COLOR_IMAGE_TOPIC,
            self.color_image_callback,
            10,
        )
        self.depth_sub = self.create_subscription(
            Image,
            DEPTH_IMAGE_TOPIC,
            self.depth_image_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10,
        )

        # OpenCV 창 및 마우스 콜백 등록
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

        self.get_logger().info("DepthClickViewer 노드가 시작되었습니다.")

    # -----------------------------
    # 콜백들
    # -----------------------------
    def camera_info_callback(self, msg: CameraInfo):
        # K = [fx, 0, cx,
        #      0, fy, cy,
        #      0,  0,  1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        # 한 번만 로깅해도 충분하므로, 다른 값에서 None → 값으로 바뀌는 시점에만 출력
        self.camera_info_sub.destroy()  # 더 이상 받을 필요 없으면 끊어도 됨
        self.get_logger().info(
            f"Camera intrinsics 수신: fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}"
        )

    def depth_image_callback(self, msg: Image):
        # 16UC1(depth in mm) 가정
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth_frame = depth_image

    def color_image_callback(self, msg: Image):
        global g_click_u, g_click_v

        # BGR8 컬러 이미지
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h_c, w_c, _ = frame.shape

        # 클릭한 점이 있으면 그 점 사용, 없으면 화면 중앙 사용
        if g_click_u is not None and g_click_v is not None:
            u = int(np.clip(g_click_u, 0, w_c - 1))
            v = int(np.clip(g_click_v, 0, h_c - 1))
            clicked = True
        else:
            u = w_c // 2
            v = h_c // 2
            clicked = False

        # depth / camera_info가 준비된 상태인지 확인
        if (
            self.depth_frame is not None
            and self.fx is not None
            and self.fy is not None
        ):
            h_d, w_d = self.depth_frame.shape[:2]

            # 컬러/깊이 해상도가 다를 수 있으므로 방어적 인덱싱
            u_depth = int(np.clip(u, 0, w_d - 1))
            v_depth = int(np.clip(v, 0, h_d - 1))

            depth_raw = self.depth_frame[v_depth, u_depth]

            if depth_raw > 0:
                Z = float(depth_raw) / 1000.0  # mm → m
                X = (u_depth - self.cx) * Z / self.fx
                Y = (v_depth - self.cy) * Z / self.fy

                self.print_counter += 1
                # 30프레임마다 한 번씩만 출력 (너무 많이 찍히지 않도록)
                if self.print_counter % 30 == 0:
                    where = "클릭한 픽셀" if clicked else "중심 픽셀"
                    self.get_logger().info(
                        f"{where} (u={u_depth}, v={v_depth}), "
                        f"depth_raw={depth_raw} -> "
                        f"X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m"
                    )

                # 유효 depth → 빨간 점 (선택점이라는 의미로)
                color = (0, 0, 255)
            else:
                # depth=0 → 노란 점
                color = (0, 255, 255)
        else:
            # depth/camera_info 준비 전 → 파란 점
            color = (255, 0, 0)

        # 선택된 픽셀(또는 중앙 픽셀)에 동그라미 표시
        cv2.circle(frame, (u, v), 6, color, -1)

        # 안내 텍스트
        text = "Click pixel to see depth & XYZ"
        cv2.putText(
            frame,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DepthClickViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()