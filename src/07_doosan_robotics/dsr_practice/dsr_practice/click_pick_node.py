#!/usr/bin/env python3
"""
click_pick_node.py  –  카메라 클릭 → Pick & Place (MoveIt 기반)

gear_assembly.py 구조 + test.py 시퀀스를 MoveIt/m 단위로 통합.
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
Z_OFFSET        = 0.20   # base_z에 더할 오프셋 (test.py의 200mm와 동일)
SAFE_Z          = 0.40   # 안전 이동 높이
APPROACH_OFFSET = 0.05   # pick/place 위에서 접근 거리

# 그리퍼
GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502

# TCP 아래로 향한 자세
DOWN_ORI = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}


# ═══════════════════════════════════════════
#  유틸 함수 (gear_assembly 스타일)
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
    """plan 후 execute. 실패 시 False 반환."""
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
    """MoveIt FK로 base_link → EE_LINK 4×4 행렬 (m)."""
    psm = moveit_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        T = scene.current_state.get_global_link_transform(EE_LINK)
    return np.asarray(T, dtype=float)


# ═══════════════════════════════════════════
#  ClickPickNode
# ═══════════════════════════════════════════
class ClickPickNode(Node):
    def __init__(self):
        super().__init__("click_pick_moveit_node")
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics  = None
        self.picking = False  # pick 중복 방지

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
        self.robot = MoveItPy(node_name="click_pick_moveit_py")
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

        # Home pose (run에서 설정)
        self.home_xyz = None  # (x, y, z) in m
        self.home_ori = None  # dict {x, y, z, w}

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
        """카메라 좌표 (m) → 베이스 좌표 (m)."""
        coord = np.append(np.array(cam_xyz_m, dtype=float), 1.0)
        base2ee = get_ee_matrix(self.robot)
        base2cam = base2ee @ self.gripper2cam
        return (base2cam @ coord)[:3]

    # ── Pick & Place (test.py 시퀀스를 MoveIt으로) ──
    def pick_and_place(self, bx, by, bz):
        """
        1) 현재 z 유지하며 XY 이동
        2) pick_z (= bz + Z_OFFSET) 로 하강
        3) gripper close
        4) SAFE_Z로 상승
        5) home XY로 이동 (SAFE_Z 유지)
        6) place_z로 하강 (pick_z 이상 보장)
        7) gripper open
        8) SAFE_Z로 상승
        """
        log = self.get_logger()
        ori = self.home_ori or DOWN_ORI

        pick_z = bz + Z_OFFSET
        place_z = max(pick_z, 0.25)  # pick 높이 이상 보장

        log.info(f"Base raw: ({bx:.3f}, {by:.3f}, {bz:.3f}) m")
        log.info(f"Z_OFFSET={Z_OFFSET}, pick_z={pick_z:.3f}, place_z={place_z:.3f}")

        # 현재 EE 위치
        cur_ee = get_ee_matrix(self.robot)
        cur_z = cur_ee[2, 3]

        hx, hy, hz = self.home_xyz

        # 0) gripper open
        self.gripper.open_gripper()
        time.sleep(0.5)

        # 1) 현재 z 유지하며 클릭 XY로 이동
        log.info(f"[1] XY → ({bx:.3f}, {by:.3f}) @ cur_z={cur_z:.3f}")
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(bx, by, cur_z, ori),
                                params=self.pilz_params):
            log.error("pick 중단: [1] plan 실패"); return

        # 2) pick_z로 하강
        log.info(f"[2] down to pick_z={pick_z:.3f}")
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(bx, by, pick_z, ori),
                                params=self.pilz_params):
            log.error("pick 중단: [2] plan 실패"); return

        # 3) gripper close
        log.info("[3] Gripper CLOSE")
        self.gripper.close_gripper()
        time.sleep(1.0)

        # 4) SAFE_Z로 상승
        log.info(f"[4] up to SAFE_Z={SAFE_Z:.3f}")
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(bx, by, SAFE_Z, ori),
                                params=self.pilz_params):
            log.error("pick 중단: [4] plan 실패"); return

        # 5) home XY로 이동 (SAFE_Z 유지)
        log.info(f"[5] home XY → ({hx:.3f}, {hy:.3f}) @ SAFE_Z")
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(hx, hy, SAFE_Z, ori),
                                params=self.pilz_params):
            log.error("pick 중단: [5] plan 실패"); return

        # 6) place_z로 하강
        log.info(f"[6] down to place_z={place_z:.3f}")
        if not plan_and_execute(self.robot, self.arm, log,
                                pose_goal=make_pose(hx, hy, place_z, ori),
                                params=self.pilz_params):
            log.error("pick 중단: [6] plan 실패"); return

        # 7) gripper open
        log.info("[7] Gripper OPEN")
        self.gripper.open_gripper()
        time.sleep(1.0)

        # 8) SAFE_Z로 상승
        log.info(f"[8] up to SAFE_Z={SAFE_Z:.3f}")
        plan_and_execute(self.robot, self.arm, log,
                         pose_goal=make_pose(hx, hy, SAFE_Z, ori),
                         params=self.pilz_params)

        log.info("========== PICK END ==========")

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

        h, w = self.depth_image.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            self.get_logger().warn("클릭 범위 초과")
            return

        z_raw = self.depth_image[y, x]
        if z_raw == 0:
            self.get_logger().warn("해당 픽셀 depth=0")
            return

        # depth → m
        z_m = float(z_raw) / 1000.0 if self.depth_image.dtype == np.uint16 else float(z_raw)

        fx, fy   = self.intrinsics["fx"],  self.intrinsics["fy"]
        ppx, ppy = self.intrinsics["ppx"], self.intrinsics["ppy"]

        cam_x = (x - ppx) * z_m / fx
        cam_y = (y - ppy) * z_m / fy
        cam_z = z_m

        base = self.transform_to_base((cam_x, cam_y, cam_z))
        if base is None:
            self.get_logger().error("좌표 변환 실패")
            return

        bx, by, bz = float(base[0]), float(base[1]), float(base[2])
        self.get_logger().info(
            f"Camera: ({cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f}) m → "
            f"Base: ({bx:.3f}, {by:.3f}, {bz:.3f}) m"
        )

        self.picking = True
        try:
            self.pick_and_place(bx, by, bz)
        finally:
            self.picking = False

    # ── 메인 루프 ──
    def run(self):
        log = self.get_logger()
        window = "ClickToPick (MoveIt)"
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, self.mouse_callback)

        # Home 이동
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

        # Home pose 저장
        T = get_ee_matrix(self.robot)
        self.home_xyz = (T[0, 3], T[1, 3], T[2, 3])
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        self.home_ori = {"x": float(qx), "y": float(qy),
                         "z": float(qz), "w": float(qw)}
        log.info(f"[Init] Home = ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}) m")

        self.gripper.open_gripper()
        time.sleep(1.0)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.color_image is None:
                continue
            cv2.imshow(window, self.color_image)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ClickPickNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
