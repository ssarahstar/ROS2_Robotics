#!/usr/bin/env python3
from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SendWaypoint(Node):
    def __init__(self):
        super().__init__("send_waypoint")

        # ---- params ----
        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("timeout_sec", 3.0)
        self.declare_parameter("duration_sec", 2.0)

        # 4-DOF 기준 기본 목표(필요시 실행 시 override)
        self.declare_parameter("target_positions", [0.0, 0.3, 0.0, 0.0])

        self.controller_name = self.get_parameter("controller_name").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.target_positions = list(self.get_parameter("target_positions").value)

        self.state_topic = f"/{self.controller_name}/state"
        self.action_name = f"/{self.controller_name}/follow_joint_trajectory"

        # ---- state cache ----
        self._last_state: Optional[JointTrajectoryControllerState] = None

        self.create_subscription(
            JointTrajectoryControllerState,
            self.state_topic,
            self._state_cb,
            10,
        )

        self._ac = ActionClient(self, FollowJointTrajectory, self.action_name)

        self.get_logger().info(f"State topic: {self.state_topic}")
        self.get_logger().info(f"Action: {self.action_name}")

    def _state_cb(self, msg: JointTrajectoryControllerState):
        self._last_state = msg

    def _wait_for_state(self) -> JointTrajectoryControllerState:
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=self.timeout_sec)
        while rclpy.ok():
            if self._last_state is not None:
                return self._last_state
            if self.get_clock().now() > end_time:
                raise TimeoutError("No controller state received.")
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("rclpy shutdown")

    def _wait_for_action_server(self):
        if not self._ac.wait_for_server(timeout_sec=self.timeout_sec):
            raise TimeoutError("Action server not available.")

    def send(self):
        # 1) 상태 확보
        state = self._wait_for_state()
        joint_names = list(state.joint_names)
        current = list(state.actual.positions)

        if len(joint_names) == 0 or len(current) == 0:
            raise RuntimeError("Empty joint state received.")
        if len(self.target_positions) != len(joint_names):
            raise ValueError(
                f"target_positions length({len(self.target_positions)}) "
                f"!= joints({len(joint_names)}): {joint_names}"
            )

        # 2) goal 구성
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        p0 = JointTrajectoryPoint()
        p0.positions = current
        p0.time_from_start = Duration(sec=0, nanosec=0)

        p1 = JointTrajectoryPoint()
        p1.positions = self.target_positions
        # duration
        sec = int(self.duration_sec)
        nsec = int((self.duration_sec - sec) * 1e9)
        p1.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [p0, p1]

        # 3) 송신
        self._wait_for_action_server()
        self.get_logger().info(f"Send joints: {joint_names}")
        self.get_logger().info(f"Current: {['%.3f' % x for x in current]}")
        self.get_logger().info(f"Target : {['%.3f' % x for x in self.target_positions]}")

        send_future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Goal rejected by controller.")

        self.get_logger().info("Goal accepted. Waiting result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec + self.duration_sec + 2.0)
        result = result_future.result()
        if result is None:
            raise TimeoutError("No result received.")
        self.get_logger().info(f"Result status: {result.status}")


def main():
    rclpy.init()
    node = SendWaypoint()
    try:
        node.send()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
