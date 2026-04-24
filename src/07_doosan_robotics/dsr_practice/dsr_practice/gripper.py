#!/usr/bin/env python3
# rg2_gripper_node.py

import time

import rclpy

from .onrobot import RG  # 같은 패키지 내부의 onrobot.py 사용


GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502

# 그리퍼 폭 (raw 단위: 1/10 mm)
GRIPPER_OPEN_WIDTH = 500   # 50.0 mm
GRIPPER_CLOSE_WIDTH = 200  # 20.0 mm
GRIPPER_FORCE = 200

# 최대 재시도 횟수 (필요하면 숫자만 바꿔서 실습)
MAX_GRIP_RETRY = 3


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rg2_gripper_node")

    logger = node.get_logger()
    logger.info("=== RG2 Gripper Control Node 시작 ===")

    # ---- 그리퍼 연결 ----
    gripper = RG(
        gripper=GRIPPER_NAME,
        ip=TOOLCHARGER_IP,
        port=TOOLCHARGER_PORT,
    )
    time.sleep(0.5)  # 연결 안정화 대기
    logger.info("그리퍼와 연결됨")

    # 1) 초기 그리퍼 OPEN (예: 50mm)
    logger.info(f"초기 그리퍼 OPEN (폭={GRIPPER_OPEN_WIDTH})")
    gripper.move_gripper(width_val=GRIPPER_OPEN_WIDTH,
                         force_val=GRIPPER_FORCE)
    time.sleep(1.0)  # 초기 위치로 이동 대기

    # 2) grip_detected가 될 때까지 CLOSE 재시도
    grip_detected = False

    for attempt in range(1, MAX_GRIP_RETRY + 1):
        logger.info(
            f"Gripper CLOSE 시도 {attempt}/{MAX_GRIP_RETRY} "
            f"(폭={GRIPPER_CLOSE_WIDTH}, 힘={GRIPPER_FORCE})"
        )
        gripper.move_gripper(width_val=GRIPPER_CLOSE_WIDTH,
                             force_val=GRIPPER_FORCE)
        time.sleep(1.0)  # 동작 완료 대기

        # onrobot.RG.get_status() → [busy, grip_detected, S1_pushed, ...]
        status = gripper.get_status()
        busy = bool(status[0])
        grip_detected = bool(status[1])

        logger.info(f"그리퍼 상태: busy={busy}, grip_detected={grip_detected}")

        if grip_detected:
            logger.info("Grip 성공 – 물체를 정상적으로 잡았습니다.")
            break
        else:
            logger.warning(
                "Grip 미검출 – 물체를 못 잡았을 수 있습니다. "
                "그리퍼를 다시 OPEN 후 재시도합니다."
            )
            # 다시 열고 잠깐 대기 후 재시도
            gripper.move_gripper(width_val=GRIPPER_OPEN_WIDTH,
                                 force_val=GRIPPER_FORCE)
            time.sleep(0.5)

    if not grip_detected:
        logger.error(
            f"최대 {MAX_GRIP_RETRY}번 시도 후에도 grip_detected가 True가 되지 않았습니다."
        )

    logger.info("=== RG2 Gripper Control Node 종료 ===")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()