#!/usr/bin/env python3
"""
STT 기반 Pick & Place 제어 노드

/stt_result (std_msgs/String) 구독
  -> 키워드 매핑 -> 명령 큐 -> 워커 스레드에서 MoveItPy + Gripper 실행
"""
import math
import os
import queue
import tempfile
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import Constraints, JointConstraint

try:
    from gtts import gTTS
    import pygame
    _TTS_OK = True
except ImportError:
    _TTS_OK = False

from .onrobot import RG

# ----- Gripper -----
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502
GRIPPER_OPEN_WIDTH = 500
GRIPPER_CLOSE_WIDTH = 150
GRIPPER_FORCE = 300

# ----- MoveIt / Robot -----
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"

HOME_JOINTS_RAD = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(0.0),
}

# ----- Safety bounds -----
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX = 0.3
SAFE_Z_MIN = 0.27

# ----- Task poses -----
TASK_POSES = {
    "pick": {
        "pos": {"x": 0.427, "y": 0.148, "z": 0.280},
        "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
    },
    "place": {
        "pos": {"x": 0.426, "y": -0.153, "z": 0.280},
        "ori": {"x": 0.000, "y": 1.000, "z": 0.000, "w": 0.000},
    },
}
APPROACH_OFFSET = 0.05


TTS_PHRASES = {
    "home":      "홈 위치로 이동합니다",
    "pick":      "물체를 집습니다",
    "place":     "물체를 내려놓습니다",
    "pickplace": "픽 앤 플레이스를 시작합니다",
    "stop":      "정지합니다",
    "done":      "완료되었습니다",
    "fail":      "실행에 실패했습니다",
}


class _TtsPlayer:
    """gTTS + pygame 기반 비동기 음성 재생 (캐시 사용)."""

    def __init__(self, lang: str = "ko", enabled: bool = True):
        self._enabled = enabled and _TTS_OK
        self._lang = lang
        self._cache: dict[str, str] = {}
        if self._enabled:
            try:
                pygame.mixer.init()
            except Exception:
                self._enabled = False

    def speak(self, text: str):
        if not self._enabled or not text:
            return
        try:
            path = self._cache.get(text)
            if not path or not os.path.exists(path):
                fd, path = tempfile.mkstemp(suffix=".mp3", prefix="dsr_tts_")
                os.close(fd)
                gTTS(text=text, lang=self._lang).save(path)
                self._cache[text] = path
            pygame.mixer.music.load(path)
            pygame.mixer.music.play()
        except Exception:
            pass


KEYWORD_MAP: dict[str, str] = {
    "홈": "home",
    "home": "home",
    "홈으로": "home",
    "픽": "pick",
    "집어": "pick",
    "잡아": "pick",
    "pick": "pick",
    "플레이스": "place",
    "놓아": "place",
    "내려놔": "place",
    "place": "place",
    "픽앤플레이스": "pickplace",
    "픽플레이스": "pickplace",
    "pickandplace": "pickplace",
    "pickplace": "pickplace",
    "정지": "stop",
    "멈춰": "stop",
    "스톱": "stop",
    "stop": "stop",
}


def _text_to_cmd(text: str) -> str | None:
    normalized = text.lower().replace(" ", "")
    for kw, cmd in KEYWORD_MAP.items():
        if kw in normalized:
            return cmd
    return None


def _clamp_to_safe_workspace(x: float, y: float, z: float, logger):
    sx, sy, sz = x, y, z

    if sx < SAFE_X_MIN:
        logger.warning(f"x safety clamp: {sx:.3f} -> {SAFE_X_MIN:.3f}")
        sx = SAFE_X_MIN
    if sy < SAFE_Y_MIN:
        logger.warning(f"y safety clamp: {sy:.3f} -> {SAFE_Y_MIN:.3f}")
        sy = SAFE_Y_MIN
    elif sy > SAFE_Y_MAX:
        logger.warning(f"y safety clamp: {sy:.3f} -> {SAFE_Y_MAX:.3f}")
        sy = SAFE_Y_MAX
    if sz < SAFE_Z_MIN:
        logger.warning(f"z safety clamp: {sz:.3f} -> {SAFE_Z_MIN:.3f}")
        sz = SAFE_Z_MIN

    return sx, sy, sz


def _build_home_joint_constraints() -> list[Constraints]:
    constraints = Constraints()
    joint_names = [f"joint_{i}" for i in range(1, 7)]

    for joint_name, position in zip(joint_names, HOME_JOINTS_RAD.values()):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = position
        joint_constraint.tolerance_above = 0.001
        joint_constraint.tolerance_below = 0.001
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

    return [constraints]


def _plan_and_execute(robot, arm, logger, plan_params=None) -> bool:
    result = arm.plan(parameters=plan_params) if plan_params else arm.plan()
    if not result:
        logger.error("Planning failed")
        return False
    robot.execute(group_name=GROUP_NAME, robot_trajectory=result.trajectory, blocking=True)
    return True


def _move_home(robot, arm, logger, home_params) -> bool:
    arm.set_start_state_to_current_state()
    arm.set_goal_state(motion_plan_constraints=_build_home_joint_constraints())
    return _plan_and_execute(robot, arm, logger, plan_params=home_params)


def _move_pose(robot, arm, logger, pose_goal: PoseStamped, pilz_params) -> bool:
    x = pose_goal.pose.position.x
    y = pose_goal.pose.position.y
    z = pose_goal.pose.position.z
    sx, sy, sz = _clamp_to_safe_workspace(x, y, z, logger)
    pose_goal.pose.position.x = sx
    pose_goal.pose.position.y = sy
    pose_goal.pose.position.z = sz

    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)
    return _plan_and_execute(robot, arm, logger, plan_params=pilz_params)


class SttPickAndPlaceNode(Node):
    def __init__(self, robot: MoveItPy, arm, home_params, pilz_params, gripper):
        super().__init__("stt_pick_and_place")

        self.declare_parameter("use_tts", True)
        use_tts = self.get_parameter("use_tts").get_parameter_value().bool_value

        self._robot = robot
        self._arm = arm
        self._home = home_params
        self._pilz = pilz_params
        self._cmd_q: queue.Queue[str] = queue.Queue()
        self._holding = False
        self._tts = _TtsPlayer(enabled=use_tts)
        self._gripper = gripper

        self.create_subscription(String, "/stt_result", self._stt_cb, 10)
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(
            f"준비 완료  명령어: home / pick / place / pickplace / stop  (tts={'on' if self._tts._enabled else 'off'})"
        )

    def _stt_cb(self, msg: String):
        cmd = _text_to_cmd(msg.data)
        if cmd is None:
            self.get_logger().debug(f"매핑 없음: '{msg.data}'")
            return

        if cmd == "stop":
            n = 0
            while not self._cmd_q.empty():
                try:
                    self._cmd_q.get_nowait()
                    n += 1
                except queue.Empty:
                    break
            self.get_logger().info(f"[STOP] 큐 {n}개 취소")
            self._tts.speak(TTS_PHRASES["stop"])
            return

        self._cmd_q.put(cmd)
        self.get_logger().info(f"[CMD] '{cmd}' 큐 추가 (크기={self._cmd_q.qsize()})")

    def _build_pose_goal(self, task_key: str, z_value: float) -> PoseStamped:
        task = TASK_POSES[task_key]
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = BASE_FRAME
        pose_goal.pose.position.x = task["pos"]["x"]
        pose_goal.pose.position.y = task["pos"]["y"]
        pose_goal.pose.position.z = z_value
        pose_goal.pose.orientation.x = task["ori"]["x"]
        pose_goal.pose.orientation.y = task["ori"]["y"]
        pose_goal.pose.orientation.z = task["ori"]["z"]
        pose_goal.pose.orientation.w = task["ori"]["w"]
        return pose_goal

    def _set_gripper(self, logger, width: int):
        try:
            self._gripper.move_gripper(width, GRIPPER_FORCE)
            time.sleep(1.0)
        except Exception as e:
            logger.error(f"Gripper error: {e}")

    def _run_pick(self, logger) -> bool:
        pick = TASK_POSES["pick"]["pos"]
        approach = self._build_pose_goal("pick", pick["z"] + APPROACH_OFFSET)
        target = self._build_pose_goal("pick", pick["z"])

        if not _move_pose(self._robot, self._arm, logger, approach, self._pilz):
            return False
        if not _move_pose(self._robot, self._arm, logger, target, self._pilz):
            return False

        logger.info("Gripper CLOSE")
        self._set_gripper(logger, GRIPPER_CLOSE_WIDTH)
        self._holding = True

        if not _move_pose(self._robot, self._arm, logger, approach, self._pilz):
            return False
        return True

    def _run_place(self, logger) -> bool:
        place = TASK_POSES["place"]["pos"]
        approach = self._build_pose_goal("place", place["z"] + APPROACH_OFFSET)
        target = self._build_pose_goal("place", place["z"])

        if not _move_pose(self._robot, self._arm, logger, approach, self._pilz):
            return False
        if not _move_pose(self._robot, self._arm, logger, target, self._pilz):
            return False

        logger.info("Gripper OPEN")
        self._set_gripper(logger, GRIPPER_OPEN_WIDTH)
        self._holding = False

        if not _move_pose(self._robot, self._arm, logger, approach, self._pilz):
            return False
        return True

    def _worker(self):
        logger = get_logger("stt_pick_and_place.worker")
        while True:
            try:
                cmd = self._cmd_q.get(timeout=1.0)
            except queue.Empty:
                continue

            logger.info(f"===== '{cmd}' 실행 =====")
            self._tts.speak(TTS_PHRASES.get(cmd, ""))
            ok = True

            if cmd == "home":
                ok = _move_home(self._robot, self._arm, logger, self._home)
            elif cmd == "pick":
                ok = self._run_pick(logger)
            elif cmd == "place":
                if not self._holding:
                    logger.warning("현재 물체를 쥐고 있지 않습니다. place를 계속 진행합니다.")
                ok = self._run_place(logger)
            elif cmd == "pickplace":
                ok = self._run_pick(logger)
                if ok:
                    ok = self._run_place(logger)

            if ok:
                logger.info(f"===== '{cmd}' 완료 =====")
            else:
                logger.error(f"===== '{cmd}' 실패 =====")
            self._tts.speak(TTS_PHRASES["done" if ok else "fail"])


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("stt_pick_and_place.main")

    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
    time.sleep(0.5)
    gripper.move_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_FORCE)

    robot = MoveItPy(node_name="moveit_py")
    arm = robot.get_planning_component(GROUP_NAME)
    logger.info("MoveItPy 초기화 완료")

    home = PlanRequestParameters(robot)
    home.planning_pipeline = "ompl"
    home.planner_id = "RRTConnect"
    home.max_velocity_scaling_factor = 0.2
    home.max_acceleration_scaling_factor = 0.1
    home.planning_time = 3.0

    pilz = PlanRequestParameters(robot)
    pilz.planning_pipeline = "pilz_industrial_motion_planner"
    pilz.planner_id = "PTP"
    pilz.max_velocity_scaling_factor = 0.15
    pilz.max_acceleration_scaling_factor = 0.1
    pilz.planning_time = 3.0

    logger.info("시작 자세를 home으로 이동합니다")
    _move_home(robot, arm, logger, home)

    node = SttPickAndPlaceNode(robot, arm, home, pilz, gripper)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    logger.info("음성 명령 대기 중 ... (Ctrl+C 종료)")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        robot.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
