#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState


class ControllerStateMonitor(Node):
    def __init__(self):
        super().__init__("controller_state_monitor")

        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("print_period", 0.5)

        self.controller_name = self.get_parameter("controller_name").value
        self.print_period = float(self.get_parameter("print_period").value)

        self.topic = f"/{self.controller_name}/controller_state"
        self._last = None

        self.create_subscription(
            JointTrajectoryControllerState,
            self.topic,
            self._cb,
            10,
        )
        self.create_timer(self.print_period, self._print)

        self.get_logger().info(f"Subscribe: {self.topic}")

    def _cb(self, msg: JointTrajectoryControllerState):
        self._last = msg

    def _print(self):
        if self._last is None:
            self.get_logger().info("No state received yet...")
            return

        names = list(self._last.joint_names)
        actual = list(self._last.actual.positions)
        desired = list(self._last.desired.positions)
        error = list(self._last.error.positions)

        # 길이 짧은 경우도 있어서 zip으로 안전하게 출력
        pairs = []
        for n, a, d, e in zip(names, actual, desired, error):
            pairs.append(f"{n}: act={a:.3f} des={d:.3f} err={e:.3f}")

        self.get_logger().info(" | ".join(pairs))


def main():
    rclpy.init()
    node = ControllerStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
