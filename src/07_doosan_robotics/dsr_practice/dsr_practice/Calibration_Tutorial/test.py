import cv2
import rclpy
import time
import numpy as np
import threading
from scipy.spatial.transform import Rotation

from realsense import ImgNode
from onrobot import RG
import DR_init

# ======================
# 로봇 / 그리퍼 설정
# ======================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502  # 정수

# ======================
# Z 관련 파라미터
# ======================
Z_OFFSET     = 220.0   # 클릭한 지점 z에 더해 줄 오프셋 (mm)
SAFE_Z       = 400.0   # 집은 뒤 올라갈 안전 높이 (mm) – 환경 보고 조정


class TestNode:
    def __init__(self):
        # RealSense 노드
        self.img_node = ImgNode()

        # Intrinsic 수신될 때까지 대기
        while rclpy.ok() and self.img_node.get_camera_intrinsic() is None:
            rclpy.spin_once(self.img_node, timeout_sec=0.1)

        self.intrinsics = self.img_node.get_camera_intrinsic()

        # Hand-eye 결과 (그리퍼 → 카메라)
        self.gripper2cam = np.load("T_gripper2camera.npy")

        # 그리퍼
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        # 준비자세(Joint)
        self.JReady = posj([0, 0, 90, 0, 90, 90])

        # 홈 XY (초기 자세에서 저장)
        self.home_pose = None  # [x, y, z, rx, ry, rz]

    # =========================
    # 카메라 → 베이스 좌표 변환
    # =========================
    def transform_to_base(self, camera_coords):
        """
        camera_coords: (Xc, Yc, Zc) in mm (카메라 기준)
        반환: (Xb, Yb, Zb) in mm (base_link 기준)
        """
        coord = np.append(np.array(camera_coords, dtype=float), 1.0)  # [x,y,z,1]

        # 현재 베이스→그리퍼
        base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])

        # 베이스→카메라 = 베이스→그리퍼 · 그리퍼→카메라
        base2cam = base2gripper @ self.gripper2cam

        td_coord = base2cam @ coord

        return td_coord[:3]

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    # =========================
    # Pick 시퀀스
    # =========================
    def pick_and_place(self, x, y, z):

        print("\n========== PICK SEQUENCE ==========")
        print(f"Base coord raw : x={x:.2f}, y={y:.2f}, z={z:.2f}")
        print(f"Z_OFFSET       : {Z_OFFSET}")
        print(f"SAFE_Z         : {SAFE_Z}")
        print("===================================\n")

        # 현재 포즈
        cur = get_current_posx()[0]
        cur_x, cur_y, cur_z, rx, ry, rz = cur

        # home pose (run()에서 설정)
        if self.home_pose is None:
            self.home_pose = cur
        home_x, home_y, home_z, hrx, hry, hrz = self.home_pose

        # 0) 안전용으로 한 번 더 open (혹시 이전에 안 열려 있으면)
        self.gripper.open_gripper()
        time.sleep(0.5)

        # 1) 클릭한 위치 x,y로 이동 (z는 현재 값 유지)
        target_xy = posx([x, y, cur_z, rx, ry, rz])
        print("[1] Move to XY only:", target_xy)
        movel(target_xy, VELOCITY, ACC)
        wait(0.5)

        # 2) z_correct 계산 및 이동
        z_correct = z + Z_OFFSET
        target_xyz = posx([x, y, z_correct, rx, ry, rz])
        print(f"[2] Move down to z_correct={z_correct:.2f}:", target_xyz)
        movel(target_xyz, VELOCITY, ACC)
        wait(0.3)

        # 3) gripper close
        print("[3] Gripper Close")
        self.gripper.close_gripper()
        time.sleep(1.0)

        # 4) z 상승: SAFE_Z까지 (x,y는 그대로)
        up_pose = posx([x, y, SAFE_Z, rx, ry, rz])
        print(f"[4] Move up to SAFE_Z={SAFE_Z:.2f}:", up_pose)
        movel(up_pose, VELOCITY, ACC)
        wait(0.5)

        # 5) home xy 이동 (z는 SAFE_Z 유지)
        home_xy_pose = posx([home_x, home_y, SAFE_Z, hrx, hry, hrz])
        print("[5] Move to home XY:", home_xy_pose)
        movel(home_xy_pose, VELOCITY, ACC)
        wait(0.5)

        # 6) z = 270 까지 이동
        z_final = 280.0
        home_xy_z280 = posx([home_x, home_y, z_final, hrx, hry, hrz])
        print(f"[6] Move to home XY, z={z_final}:", home_xy_z280)
        movel(home_xy_z280, VELOCITY, ACC)
        wait(0.3)

        # 7) gripper open
        print("[7] Gripper Open")
        self.gripper.open_gripper()
        time.sleep(1.0)

        # 8) 다시 SAFE_Z로 올려두고 끝낼지 여부 (원하면 유지)
        back_up = posx([home_x, home_y, SAFE_Z, hrx, hry, hrz])
        print("[8] Back up to SAFE_Z:", back_up)
        movel(back_up, VELOCITY, ACC)
        wait(0.5)

        print("========== PICK END ==========\n")

    # =========================
    # 마우스 콜백
    # =========================
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not hasattr(self, '_pick_thread_running'):
            depth_frame = self.img_node.get_depth_frame()
            if depth_frame is None:
                print("No depth frame")
                return

            # 픽셀 범위 체크
            h, w = depth_frame.shape
            if not (0 <= x < w and 0 <= y < h):
                print("Click out of range")
                return

            z = depth_frame[y, x]
            if z == 0:
                print("Depth invalid at clicked point")
                return

            # 카메라 좌표 (mm) 계산
            fx = self.intrinsics["fx"]
            fy = self.intrinsics["fy"]
            ppx = self.intrinsics["ppx"]
            ppy = self.intrinsics["ppy"]

            X = (x - ppx) * z / fx
            Y = (y - ppy) * z / fy
            Z = z

            cam_coord = (X, Y, Z)
            base_coord = self.transform_to_base(cam_coord)

            print("Camera:", cam_coord)
            print("Base  :", base_coord)

            def run_pick():
                self._pick_thread_running = True
                self.pick_and_place(*base_coord)
                del self._pick_thread_running

            threading.Thread(target=run_pick, daemon=True).start()

    # =========================
    # 메인 루프
    # =========================
    def run(self):
        cv2.namedWindow("Webcam")
        cv2.setMouseCallback("Webcam", self.mouse_callback)

        # rclpy spin을 별도 스레드로 분리 (pick 스레드와 충돌 방지)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.img_node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 초기 자세로 이동
        print("[Init] movej JReady")
        movej(self.JReady, VELOCITY, ACC)
        wait(1.0)

        # 현재 자세를 home_pose로 저장
        self.home_pose = get_current_posx()[0]

        # 초기 gripper open
        print("[Init] Gripper Open")
        self.gripper.open_gripper()
        time.sleep(1.0)

        while True:
            img = self.img_node.get_color_frame()
            if img is None:
                time.sleep(0.01)
                continue

            cv2.imshow("Webcam", img)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break

        executor.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import get_current_posx, movej, movel, wait
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        rclpy.shutdown()
        raise SystemExit(1)

    test = TestNode()
    test.run()

    rclpy.shutdown()