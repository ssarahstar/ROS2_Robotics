#!/usr/bin/env python3
"""
STT 음성 명령 로봇 제어 노드 (오프셋 이동 방식)

/stt_result (std_msgs/String) 구독
  → 키워드 매핑 → 명령 큐 → 워커 스레드에서 MoveItPy 실행

명령어:
  home                 → HOME 조인트 자세 (OMPL RRTConnect)
  앞/뒤/왼쪽/오른쪽/위/아래 → 현재 EE 위치 기준 5cm 오프셋 이동 (Pilz PTP)
  stop                 → 명령 큐 비우기
"""
import math
import os
import queue
import tempfile
import threading

import numpy as np
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

# ── 로봇 설정 ──────────────────────────────────────────────
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK    = "link_6"
HOME_JOINTS_RAD = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(0.0),
}

# ── 안전 작업 영역 (base_link 기준) ───────────────────────
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX =  0.3
SAFE_Z_MIN =  0.27

# ── 방향 → (dx, dy, dz) 오프셋 ───────────────────────────
JOG_OFFSET = 0.05  # m
DIRECTIONS = {
    "forward":  ( JOG_OFFSET,  0.0,         0.0),
    "backward": (-JOG_OFFSET,  0.0,         0.0),
    "left":     ( 0.0,          JOG_OFFSET,  0.0),
    "right":    ( 0.0,         -JOG_OFFSET,  0.0),
    "up":       ( 0.0,          0.0,         JOG_OFFSET),
    "down":     ( 0.0,          0.0,        -JOG_OFFSET),
}

# ── 명령별 음성 안내 ──────────────────────────────────────
TTS_PHRASES = {
    "home":     "홈 위치로 이동합니다",
    "forward":  "앞으로 이동합니다",
    "backward": "뒤로 이동합니다",
    "left":     "왼쪽으로 이동합니다",
    "right":    "오른쪽으로 이동합니다",
    "up":       "위로 이동합니다",
    "down":     "아래로 이동합니다",
    "stop":     "정지합니다",
    "done":     "완료되었습니다",
    "fail":     "실행에 실패했습니다",
}

# ── 키워드 → 명령 매핑 ────────────────────────────────────
KEYWORD_MAP: dict[str, str] = {
    "홈": "home", "home": "home", "홈으로": "home",
    "왼쪽": "left", "왼": "left", "left": "left",
    "오른쪽": "right", "오른": "right", "right": "right",
    "앞": "forward", "앞으로": "forward", "전방": "forward",
    "front": "forward", "forward": "forward",
    "뒤": "backward", "뒤로": "backward", "후방": "backward",
    "back": "backward", "backward": "backward",
    "위": "up", "위로": "up", "올려": "up", "올라가": "up", "up": "up",
    "아래": "down", "아래로": "down", "내려": "down", "내려가": "down", "down": "down",
    "정지": "stop", "멈춰": "stop", "스톱": "stop", "stop": "stop",
}

VALID_CMDS = {"home"} | set(DIRECTIONS.keys())


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


def _text_to_cmd(text: str) -> str | None:
    normalized = text.lower().replace(" ", "")
    for kw, cmd in KEYWORD_MAP.items():
        if kw in normalized:
            return cmd
    return None


def _clamp_to_safe(x: float, y: float, z: float, logger) -> tuple[float, float, float]:
    if x < SAFE_X_MIN:
        logger.warning(f"x 클램핑: {x:.3f} → {SAFE_X_MIN:.3f}")
        x = SAFE_X_MIN
    if y < SAFE_Y_MIN:
        logger.warning(f"y 클램핑: {y:.3f} → {SAFE_Y_MIN:.3f}")
        y = SAFE_Y_MIN
    elif y > SAFE_Y_MAX:
        logger.warning(f"y 클램핑: {y:.3f} → {SAFE_Y_MAX:.3f}")
        y = SAFE_Y_MAX
    if z < SAFE_Z_MIN:
        logger.warning(f"z 클램핑: {z:.3f} → {SAFE_Z_MIN:.3f}")
        z = SAFE_Z_MIN
    return x, y, z


def _rot_matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 회전행렬 → (x, y, z, w) 쿼터니언."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _plan_and_execute(robot, arm, logger, plan_params=None) -> bool:
    logger.info("Planning ...")
    result = arm.plan(parameters=plan_params) if plan_params else arm.plan()
    if not result:
        logger.error("Planning failed")
        return False
    logger.info("Executing ...")
    robot.execute(group_name=GROUP_NAME, robot_trajectory=result.trajectory, blocking=True)
    logger.info("Done")
    return True


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


def _move_home(robot, arm, logger, plan_params=None) -> bool:
    arm.set_start_state_to_current_state()
    arm.set_goal_state(motion_plan_constraints=_build_home_joint_constraints())
    return _plan_and_execute(robot, arm, logger, plan_params=plan_params)


# ══════════════════════════════════════════════════════════
class SttRobotControlNode(Node):

    def __init__(
        self,
        robot: MoveItPy,
        arm,
        home_params: PlanRequestParameters,
        pilz_params: PlanRequestParameters,
    ):
        super().__init__("stt_robot_control")

        self.declare_parameter("use_tts", True)
        use_tts = self.get_parameter("use_tts").get_parameter_value().bool_value

        self._robot      = robot
        self._arm        = arm
        self._home       = home_params
        self._pilz       = pilz_params
        self._cmd_q: queue.Queue[str] = queue.Queue()
        self._tts        = _TtsPlayer(enabled=use_tts)

        threading.Thread(target=self._worker, daemon=True).start()

        self.create_subscription(String, "/stt_result", self._stt_cb, 10)
        self.get_logger().info(
            f"준비 완료  명령어: home / {' / '.join(DIRECTIONS)} / stop  "
            f"(jog={JOG_OFFSET*100:.0f}cm, tts={'on' if self._tts._enabled else 'off'})"
        )

    # ── ROS 콜백 ───────────────────────────────────────────
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
        else:
            self._cmd_q.put(cmd)
            self.get_logger().info(f"[CMD] '{cmd}' 큐 추가 (크기={self._cmd_q.qsize()})")

    # ── 워커 스레드 ────────────────────────────────────────
    def _worker(self):
        logger = get_logger("stt_robot_control.worker")

        while True:
            try:
                cmd = self._cmd_q.get(timeout=1.0)
            except queue.Empty:
                continue

            logger.info(f"===== '{cmd}' 실행 =====")
            self._tts.speak(TTS_PHRASES.get(cmd, ""))
            ok = True

            if cmd == "home":
                ok = _move_home(
                    self._robot,
                    self._arm,
                    logger,
                    plan_params=self._home,
                )

            elif cmd in DIRECTIONS:
                ok = self._move_offset(logger, cmd)

            logger.info(f"===== '{cmd}' 완료 =====")
            self._tts.speak(TTS_PHRASES["done" if ok else "fail"])

    # ── 현재 EE 위치 기준 오프셋 이동 ──────────────────────
    def _move_offset(self, logger, direction: str) -> bool:
        try:
            with self._robot.get_planning_scene_monitor().read_only() as scene:
                state = scene.current_state
                state.update()
                tf = state.get_frame_transform(EE_LINK)
        except Exception as e:
            logger.error(f"현재 EE 상태 조회 실패: {e}")
            return False

        cx = float(tf[0, 3])
        cy = float(tf[1, 3])
        cz = float(tf[2, 3])
        qx, qy, qz, qw = _rot_matrix_to_quat(tf[:3, :3])

        dx, dy, dz = DIRECTIONS[direction]
        tx, ty, tz = _clamp_to_safe(cx + dx, cy + dy, cz + dz, logger)

        logger.info(
            f"JOG dir={direction} offset={JOG_OFFSET*100:.1f}cm  "
            f"({cx:.3f},{cy:.3f},{cz:.3f}) → ({tx:.3f},{ty:.3f},{tz:.3f})"
        )

        ps = PoseStamped()
        ps.header.frame_id    = BASE_FRAME
        ps.pose.position.x    = tx
        ps.pose.position.y    = ty
        ps.pose.position.z    = tz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=ps, pose_link=EE_LINK)
        return _plan_and_execute(self._robot, self._arm, logger, plan_params=self._pilz)


# ══════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("stt_robot_control.main")

    robot = MoveItPy(node_name="moveit_py")
    arm   = robot.get_planning_component(GROUP_NAME)
    logger.info("MoveItPy 초기화 완료")

    home = PlanRequestParameters(robot)
    home.planning_pipeline               = "ompl"
    home.planner_id                      = "RRTConnect"
    home.max_velocity_scaling_factor     = 0.2
    home.max_acceleration_scaling_factor = 0.1
    home.planning_time                   = 3.0

    pilz = PlanRequestParameters(robot)
    pilz.planning_pipeline               = "pilz_industrial_motion_planner"
    pilz.planner_id                      = "PTP"
    pilz.max_velocity_scaling_factor     = 0.2
    pilz.max_acceleration_scaling_factor = 0.1
    pilz.planning_time                   = 3.0

    logger.info("시작 자세를 home으로 이동합니다")
    _move_home(robot, arm, logger, plan_params=home)

    node = SttRobotControlNode(robot, arm, home, pilz)

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
