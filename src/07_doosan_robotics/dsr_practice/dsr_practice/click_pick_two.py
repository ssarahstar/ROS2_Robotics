#!/usr/bin/env python3
"""
click_pick_two.py  –  두 번 클릭으로 Pick & Place 위치 지정

1st click : pick 위치 (픽셀 + depth → base 좌표 저장)
2nd click : place 위치 (픽셀 + depth → XY만 사용, Z는 pick bz 재사용)
"""

import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from .onrobot import RG


# ═══════════════════════════════════════════
#  설정
# ═══════════════════════════════════════════
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK    = "link_6"

HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}

# 안전 작업 영역 (m, base_link 기준)
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.30
SAFE_Y_MAX =  0.30
SAFE_Z_MIN =  0.25

# Pick/Place 파라미터 (m)
Z_OFFSET        = 0.20   # base_z에 더할 오프셋
SAFE_Z          = 0.40   # 안전 이동 높이
APPROACH_OFFSET = 0.05   # pick/place 위에서 접근 거리

# 그리퍼
GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502

# TCP 아래로 향한 자세
DOWN_ORI = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}


# ═══════════════════════════════════════════
#  유틸 함수 (click_pick_node와 동일)
# ═══════════════════════════════════════════
def clamp_to_safe_workspace(x, y, z, logger):
    if x < SAFE_X_MIN:
        logger.warning(f"x={x:.3f} → {SAFE_X_MIN}")
        x = SAFE_X_MIN
    if y < SAFE_Y_MIN:
        logger.warning(f"y={y:.3f} → {SAFE_Y_MIN}")
        y = SAFE_Y_MIN
    elif y > SAFE_Y_MAX:
        logger.warning(f"y={y:.3f} → {SAFE_Y_MAX}")
        y = SAFE_Y_MAX
    if z < SAFE_Z_MIN:
        logger.warning(f"z={z:.3f} → {SAFE_Z_MIN}")
        z = SAFE_Z_MIN
    return x, y, z


def plan_and_execute(robot, arm, logger, pose_goal=None,
                     state_goal=None, params=None):
    arm.set_start_state_to_current_state()

    if pose_goal is not None:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)
    elif state_goal is not None:
        arm.set_goal_state(robot_state=state_goal)
    else:
        logger.error("pose/state 없음")
        return False

    plan_result = (arm.plan(parameters=params)
                   if params is not None else arm.plan())
    if not plan_result:
        logger.error("Planning 실패")
        return False

    robot.execute(group_name=GROUP_NAME,
                  robot_trajectory=plan_result.trajectory,
                  blocking=True)
    return True


def make_pose(x, y, z, ori=None):
    if ori is None:
        ori = DOWN_ORI
    p = PoseStamped()
    p.header.frame_id = BASE_FRAME
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    p.pose.orientation.x = ori["x"]
    p.pose.orientation.y = ori["y"]
    p.pose.orientation.z = ori["z"]
    p.pose.orientation.w = ori["w"]
    return p


def get_ee_matrix(moveit_robot):
    psm = moveit_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        T = scene.current_state.get_global_link_transform(EE_LINK)
    return np.asarray(T, dtype=float)


# ═══════════════════════════════════════════
#  ClickPickTwoNode
# ═══════════════════════════════════════════
class ClickPickTwoNode(Node):
    def __init__(self):
        super().__init__("click_pick_two_node")
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics  = None

        self.picking    = False   # pick 실행 중 중복 방지

        # ① 두 번 클릭 상태 관리 변수 추가
        self.pick_point = None     # None: 첫 클릭 대기 / (bx,by,bz): 두 번째 클릭 대기

        # Hand-Eye 변환행렬 로드
        calib_file = (
            Path(get_package_share_directory("dsr_practice"))
            / "config" / "T_gripper2camera.npy"
        )
        self.gripper2cam = np.load(str(calib_file)).astype(float)
        self.gripper2cam[:3, 3] /= 1000.0  # mm → m
        self.get_logger().info(f"Hand-Eye 로드: {calib_file}")

        # 그리퍼
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        # MoveIt
        self.get_logger().info("MoveItPy 초기화 중…")
        self.robot = MoveItPy(node_name="click_pick_two_moveit_py")
        self.arm   = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.get_logger().info("MoveItPy 초기화 완료")

        # Plan 파라미터
        self.ompl_params = PlanRequestParameters(self.robot)
        self.ompl_params.planning_pipeline = "ompl"
        self.ompl_params.planner_id = "RRTConnect"
        self.ompl_params.max_velocity_scaling_factor = 0.2
        self.ompl_params.max_acceleration_scaling_factor = 0.1
        self.ompl_params.planning_time = 2.0

        self.pilz_params = PlanRequestParameters(self.robot)
        self.pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.pilz_params.planner_id = "PTP"
        self.pilz_params.max_velocity_scaling_factor = 0.15
        self.pilz_params.max_acceleration_scaling_factor = 0.1
        self.pilz_params.planning_time = 2.0

        self.home_xyz = None
        self.home_ori = None

        # 구독
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            self._cam_info_cb, 10)
        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self._color_cb, 10)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_cb, 10)

    # ── 콜백 ──
    def _cam_info_cb(self, msg):
        self.intrinsics = {
            "fx": msg.k[0], "fy": msg.k[4],
            "ppx": msg.k[2], "ppy": msg.k[5],
        }

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # ── 좌표 변환 ──
    def transform_to_base(self, cam_xyz_m):
        coord = np.append(np.array(cam_xyz_m, dtype=float), 1.0)
        base2ee  = get_ee_matrix(self.robot)
        base2cam = base2ee @ self.gripper2cam
        return (base2cam @ coord)[:3]

    # ── 픽셀 → base 좌표 공통 변환 ──
    def pixel_to_base(self, x, y):
        h, w = self.depth_image.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            self.get_logger().warn("클릭 범위 초과")
            return None

        z_raw = self.depth_image[y, x]
        if z_raw == 0:
            self.get_logger().warn("depth=0 — 다시 클릭")
            return None

        z_m  = float(z_raw) / 1000.0
        fx   = self.intrinsics["fx"]
        fy   = self.intrinsics["fy"]
        ppx  = self.intrinsics["ppx"]
        ppy  = self.intrinsics["ppy"]

        cam_x = (x - ppx) * z_m / fx
        cam_y = (y - ppy) * z_m / fy
        base  = self.transform_to_base((cam_x, cam_y, z_m))
        if base is None:
            self.get_logger().error("좌표 변환 실패")
            return None

        return float(base[0]), float(base[1]), float(base[2])

    # ── 마우스 콜백 ──
    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.picking:
            self.get_logger().warn("pick 동작 중 — 무시")
            return
        if self.color_image is None or self.depth_image is None or self.intrinsics is None:
            self.get_logger().warn("프레임/내참 아직 준비 안 됨")
            return

        result = self.pixel_to_base(x, y)
        if result is None:
            return
        bx, by, bz = result

        # ② 첫 번째 / 두 번째 클릭 분기
        if self.pick_point is None:         
            # 첫 번째 클릭 → pick 좌표 저장
            self.pick_point = (bx, by, bz)
            self.get_logger().info(
                f"[1st click] Pick 좌표 저장: ({bx:.3f}, {by:.3f}, {bz:.3f}) m")
            self.get_logger().info("이제 Place할 위치를 클릭하세요.")

        else:
            # 두 번째 클릭 → place 좌표 계산 후 실행
            px, py = bx, by
            _, _, pick_bz = self.pick_point

            # ③ place z는 pick 때 측정한 bz 재사용
            pz = pick_bz                   

            self.get_logger().info(
                f"[2nd click] Place 좌표: ({px:.3f}, {py:.3f}, {pz:.3f}) m")

            ppx_r, ppy_r, ppz_r = self.pick_point
            self.pick_point = None  # 상태 초기화

            self.picking = True
            try:
                self.pick_and_place(ppx_r, ppy_r, ppz_r, px, py, pz)
            finally:
                self.picking = False

    # ── Pick & Place ──
    def pick_and_place(self, bx, by, bz, px, py, pz):
        """
        bx, by, bz : pick 베이스 좌표 (m)
        px, py, pz : place 베이스 좌표 (m)
        """
        log = self.get_logger()
        ori = self.home_ori or DOWN_ORI

        pick_z  = bz + Z_OFFSET
        place_z = pz + Z_OFFSET

        log.info(f"Pick  base: ({bx:.3f}, {by:.3f}, {bz:.3f})")
        log.info(f"Place base: ({px:.3f}, {py:.3f}, {pz:.3f})")
        log.info(f"pick_z={pick_z:.3f}, place_z={place_z:.3f}")

        # 0) gripper open
        self.gripper.open_gripper()
        time.sleep(0.5)

        cur_z = get_ee_matrix(self.robot)[2, 3]

        # 1) 현재 z 유지하며 pick XY로 이동
        log.info(f"[1] pick XY → ({bx:.3f}, {by:.3f}) @ cur_z={cur_z:.3f}")
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(bx, by, cur_z, ori),
                         params=self.pilz_params)

        # 2) pick_z로 하강
        log.info(f"[2] down to pick_z={pick_z:.3f}")
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(bx, by, pick_z, ori),
                         params=self.pilz_params)

        # 3) gripper close
        log.info("[3] Gripper CLOSE")
        self.gripper.close_gripper()
        time.sleep(1.0)

        # 4) SAFE_Z로 상승
        log.info(f"[4] up to SAFE_Z={SAFE_Z}")
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(bx, by, SAFE_Z, ori),
                         params=self.pilz_params)

        # 5) place XY로 이동 (SAFE_Z 유지)
        log.info(f"[5] place XY → ({px:.3f}, {py:.3f}) @ SAFE_Z")
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(px, py, SAFE_Z, ori),
                         params=self.pilz_params)

        # 6) place_z로 하강
        log.info(f"[6] down to place_z={place_z:.3f}")
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(px, py, place_z, ori),
                         params=self.pilz_params)

        # 7) gripper open
        log.info("[7] Gripper OPEN")
        self.gripper.open_gripper()
        time.sleep(1.0)

        # 8) SAFE_Z로 상승
        log.info(f"[8] up to SAFE_Z={SAFE_Z}")
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(px, py, SAFE_Z, ori),
                         params=self.pilz_params)

        log.info("========== PICK & PLACE END ==========")

    # ── 메인 루프 ──
    def run(self):
        log = self.get_logger()
        window = "ClickToPick Two (MoveIt)"
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, self.mouse_callback)

        log.info("[Init] Home 이동")
        home_state = RobotState(self.robot_model)
        home_state.joint_positions = HOME_JOINTS
        home_state.update()
        if not plan_and_execute(self.robot, self.arm, log,
                                state_goal=home_state,
                                params=self.ompl_params):
            log.error("Home 이동 실패 — 종료")
            return

        time.sleep(0.5)

        T = get_ee_matrix(self.robot)
        self.home_xyz = (T[0, 3], T[1, 3], T[2, 3])
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        self.home_ori = {"x": float(qx), "y": float(qy),
                         "z": float(qz), "w": float(qw)}
        log.info(f"[Init] Home = ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}) m")

        self.gripper.open_gripper()
        time.sleep(1.0)

        log.info("준비 완료 — Pick할 위치를 클릭하세요.")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.color_image is None:
                continue

            display = self.color_image.copy()
            if self.pick_point is not None:
                cv2.putText(display, "Pick selected! Click Place position.",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display, "Click Pick position.",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (255, 255, 255), 2)

            cv2.imshow(window, display)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ClickPickTwoNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
