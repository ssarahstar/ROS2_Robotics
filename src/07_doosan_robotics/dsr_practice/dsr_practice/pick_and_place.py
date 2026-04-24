#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

# ====== OnRobot RG2 설정 ======
from .onrobot import RG  # 같은 패키지 내부의 onrobot.py 사용

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502

# 그리퍼 폭 (raw 단위: 1/10 mm)
GRIPPER_OPEN_WIDTH = 500   # 50.0 mm
GRIPPER_CLOSE_WIDTH = 150  # 20.0 mm
GRIPPER_FORCE = 300        # 약 20 N

# ================================
# 기본 설정
# ================================
GROUP_NAME = "manipulator"   # SRDF에 정의된 planning group 이름
BASE_FRAME = "base_link"     # 로봇 베이스 프레임
EE_LINK = "link_6"           # 엔드이펙터 링크 이름 (SRDF/URDF 기준)

HOME_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
HOME_JOINTS_RAD = [math.radians(d) for d in HOME_JOINTS_DEG]

# ====== 안전 작업 영역 정의 (base_link 기준) ======
SAFE_X_MIN = 0.0      # x는 0 이상
SAFE_Y_MIN = -0.3     # y 하한
SAFE_Y_MAX = 0.3      # y 상한
SAFE_Z_MIN = 0.27    # z는 이 값보다 낮아지면 안 됨
# ==================================================

# ====== 기어 Pick/Place 포즈 (base_link 기준) ======
GEAR_TASK = {
    "pick": {
        "pos": {"x": 0.427, "y":  0.148, "z": 0.280},
        "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
    },
    "place": {
        "pos": {"x": 0.426, "y": -0.153, "z": 0.280},
        "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
    },
}

APPROACH_OFFSET = 0.05  # 위에서 접근/후퇴할 거리 [m]


def clamp_to_safe_workspace(x: float, y: float, z: float, logger):
    """안전 작업 영역으로 (x, y, z) 클램핑"""
    safe_x = x
    safe_y = y
    safe_z = z

    if safe_x < SAFE_X_MIN:
        logger.warning(
            f"Requested x ({safe_x:.3f} m) is below safety limit "
            f"({SAFE_X_MIN:.3f} m). Clamping to SAFE_X_MIN."
        )
        safe_x = SAFE_X_MIN

    if safe_y < SAFE_Y_MIN:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is below safety limit "
            f"({SAFE_Y_MIN:.3f} m). Clamping to SAFE_Y_MIN."
        )
        safe_y = SAFE_Y_MIN
    elif safe_y > SAFE_Y_MAX:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is above safety limit "
            f"({SAFE_Y_MAX:.3f} m). Clamping to SAFE_Y_MAX."
        )
        safe_y = SAFE_Y_MAX

    if safe_z < SAFE_Z_MIN:
        logger.warning(
            f"Requested z ({safe_z:.3f} m) is below safety limit "
            f"({SAFE_Z_MIN:.3f} m). Clamping to SAFE_Z_MIN."
        )
        safe_z = SAFE_Z_MIN

    return safe_x, safe_y, safe_z


def plan_and_execute(
    robot: MoveItPy,
    planning_component,
    logger,
    pose_goal: PoseStamped = None,
    plan_parameters=None,
):
    """
    공식 문서 스타일 helper: 계획 후 곧바로 실행

    - pose_goal이 주어지면:
      · 안전 영역 클램핑
      · start_state = current
      · pose 기반 goal 설정 (EE_LINK)
    - 그 다음 plan_parameters 유무에 따라 plan() 호출 후 execute
    """
    if pose_goal is not None:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z

        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz

        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=EE_LINK,
        )

    logger.info("Planning trajectory")

    if plan_parameters is not None:
        plan_result = planning_component.plan(
            parameters=plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed")
        return False

    logger.info("Executing plan")
    robot_trajectory = plan_result.trajectory
    robot.execute(
        group_name=GROUP_NAME,
        robot_trajectory=robot_trajectory,
        blocking=True,
    )
    logger.info("Execution finished")
    return True


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("gear_pick_place_simple")

    # ---- Gripper ----
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
    time.sleep(0.5)
    gripper.move_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_FORCE)

    # ---- MoveIt ----
    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component(GROUP_NAME)
    robot_model = robot.get_robot_model()

    # ---- PlanRequestParameters (HOME / Pilz) ----
    home_params = PlanRequestParameters(robot)
    home_params.planning_pipeline = "ompl"
    home_params.planner_id = "RRTConnectkConfigDefault"
    home_params.max_velocity_scaling_factor = 0.2
    home_params.max_acceleration_scaling_factor = 0.1
    home_params.planning_time = 2.0

    pilz_params = PlanRequestParameters(robot)
    pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
    pilz_params.planner_id = "PTP"
    pilz_params.max_velocity_scaling_factor = 0.15
    pilz_params.max_acceleration_scaling_factor = 0.1
    pilz_params.planning_time = 2.0

    # ---- HOME 자세로 이동 (joint goal) ----
    logger.info("=== HOME 자세로 이동 ===")
    home_state = RobotState(robot_model)
    home_state.set_joint_group_positions(GROUP_NAME, HOME_JOINTS_RAD)
    home_state.update()

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=home_state)
    plan_and_execute(robot, arm, logger, plan_parameters=home_params)

    logger.info("=== Gear Pick & Place 시작 ===")

    # ---- PoseStamped 공용 객체 준비 ----
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = BASE_FRAME

    pick = GEAR_TASK["pick"]
    place = GEAR_TASK["place"]

    # ============================
    #          PICK
    # ============================
    pos = pick["pos"]
    ori = pick["ori"]

    # 1) pick 위에서 접근
    pose_goal.pose.position.x = pos["x"]
    pose_goal.pose.position.y = pos["y"]
    pose_goal.pose.position.z = pos["z"] + APPROACH_OFFSET

    pose_goal.pose.orientation.x = ori["x"]
    pose_goal.pose.orientation.y = ori["y"]
    pose_goal.pose.orientation.z = ori["z"]
    pose_goal.pose.orientation.w = ori["w"]

    plan_and_execute(
        robot,
        arm,
        logger,
        pose_goal=pose_goal,
        plan_parameters=pilz_params,
    )

    # 2) pick 높이까지 내려가기
    pose_goal.pose.position.z = pos["z"]
    plan_and_execute(
        robot,
        arm,
        logger,
        pose_goal=pose_goal,
        plan_parameters=pilz_params,
    )

    # 3) 그리퍼 닫기 (집기)
    logger.info("Gripper CLOSE – gear pick")
    gripper.move_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_FORCE)
    time.sleep(1.0)

    # 4) 다시 위로
    pose_goal.pose.position.z = pos["z"] + APPROACH_OFFSET
    plan_and_execute(
        robot,
        arm,
        logger,
        pose_goal=pose_goal,
        plan_parameters=pilz_params,
    )

    # ============================
    #          PLACE
    # ============================
    pos = place["pos"]
    ori = place["ori"]

    # 5) place 위에서 접근
    pose_goal.pose.position.x = pos["x"]
    pose_goal.pose.position.y = pos["y"]
    pose_goal.pose.position.z = pos["z"] + APPROACH_OFFSET

    pose_goal.pose.orientation.x = ori["x"]
    pose_goal.pose.orientation.y = ori["y"]
    pose_goal.pose.orientation.z = ori["z"]
    pose_goal.pose.orientation.w = ori["w"]

    plan_and_execute(
        robot,
        arm,
        logger,
        pose_goal=pose_goal,
        plan_parameters=pilz_params,
    )

    # 6) place 높이까지 내려가기
    pose_goal.pose.position.z = pos["z"]
    plan_and_execute(
        robot,
        arm,
        logger,
        pose_goal=pose_goal,
        plan_parameters=pilz_params,
    )

    # 7) 그리퍼 열기 (놓기)
    logger.info("Gripper OPEN – gear place")
    gripper.move_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_FORCE)
    time.sleep(1.0)

    # 8) 다시 위로
    pose_goal.pose.position.z = pos["z"] + APPROACH_OFFSET
    plan_and_execute(
        robot,
        arm,
        logger,
        pose_goal=pose_goal,
        plan_parameters=pilz_params,
    )

    # 마지막으로 HOME으로 복귀 (선택)
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=home_state)
    plan_and_execute(robot, arm, logger, plan_parameters=home_params)

    logger.info("=== Gear Pick & Place 노드 종료 ===")
    rclpy.shutdown()


if __name__ == "__main__":
    main()