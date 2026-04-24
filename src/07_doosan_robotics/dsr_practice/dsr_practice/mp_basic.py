#!/usr/bin/env python3
import math

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

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
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """공식 문서 스타일 helper: 계획 후 곧바로 실행"""
    logger.info("Planning trajectory")

    # multi / single / default 순서대로 선택
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # 계획 결과 확인 및 실행
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(group_name=GROUP_NAME, robot_trajectory=robot_trajectory, blocking=True)
        logger.info("Execution finished")
    else:
        logger.error("Planning failed")


def main():
    # ================================
    # Instantiating moveit_py and planning component
    # ================================
    rclpy.init()
    logger = get_logger("m0609.moveit_py.basic")

    # MoveItPy 인스턴스 및 planning component 생성
    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component(GROUP_NAME)
    logger.info("MoveItPy instance created")

    # 로봇 모델 / RobotState 준비
    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)


    home_state = RobotState(robot_model)
    home_state.set_joint_group_positions(GROUP_NAME, HOME_JOINTS_RAD)
    home_state.update()

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=home_state)

    plan_and_execute(robot, arm, logger)


    # 시작 상태: 현재 상태
    arm.set_start_state_to_current_state()

    # EE pose 목표 설정
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = BASE_FRAME

    # ★ 실제 M0609 작업 공간에 맞게 숫자 조정해서 사용 ★
    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = 0.2
    pose_goal.pose.position.z = 0.5

    # 예시 orientation (Y축 180도 뒤집힌 자세) – 필요하면 수정
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 1.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0

    # EE 링크 이름은 SRDF/URDF와 일치해야 함 (여기선 "link_6")
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)

    plan_and_execute(robot, arm, logger)

    # 종료
    rclpy.shutdown()


if __name__ == "__main__":
    main()