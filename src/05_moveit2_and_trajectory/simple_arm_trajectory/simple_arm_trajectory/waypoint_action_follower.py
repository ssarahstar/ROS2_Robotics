#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class WaypointActionFollower(Node):
    def __init__(self):
        super().__init__("waypoint_action_follower")

        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("segment_time", 2.0)
        self.declare_parameter("hold_time", 0.5)

        self.controller_name = self.get_parameter("controller_name").value
        self.segment_time = float(self.get_parameter("segment_time").value)
        self.hold_time = float(self.get_parameter("hold_time").value)

        self.joint_names = ["joint1_z", "joint1_y", "joint2", "joint3"]


        self.waypoints = [
            [0.0, 0.0, 0.0, 0.0],  
            [0.5, 0.0, 0.0, 0.0],  
            [0.5, 0.5, 0.0, 0.0],  
            [0.0, 0.5, 0.0, 0.0],  
            [0.0, 0.0, 0.0, 0.0],  
        ]

        self.action_name = f"/{self.controller_name}/follow_joint_trajectory"
        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)

        self._retry_timer = None
        self._hold_timer = None
        self._in_flight = False  # 실행 중인 goal이 있으면 True

        self.get_logger().info(f"Action: {self.action_name}")
        self._try_send()

    def _build_goal(self) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        t = 0.0
        for pos in self.waypoints:
            t += self.segment_time
            pt = JointTrajectoryPoint()
            pt.positions = pos
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            goal.trajectory.points.append(pt)

        return goal

    def _try_send(self):
        # 이미 실행 중이면 절대 새 goal을 보내지 않음
        if self._in_flight:
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for action server...")
            if self._retry_timer is None:
                self._retry_timer = self.create_timer(0.5, self._retry_cb)
            return

        # 서버가 뜨면 retry 타이머 정리
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

        goal = self._build_goal()
        self.get_logger().info(
            f"Send trajectory: points={len(goal.trajectory.points)}, segment_time={self.segment_time}s"
        )

        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _retry_cb(self):
        self._try_send()

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            self._schedule_next(1.0)
            return

        self._in_flight = True
        self.get_logger().info("Goal accepted. Waiting result...")
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        self._in_flight = False

        if result is None:
            self.get_logger().error("No result received.")
            self._schedule_next(1.0)
            return

        self.get_logger().info(f"Trajectory done. status={result.status}. Repeat after {self.hold_time}s")
        self._schedule_next(self.hold_time)

    def _schedule_next(self, delay_sec: float):
        # 한 번만 실행되는 hold 타이머를 구현 (기존 타이머 있으면 제거)
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self._hold_timer = None

        def _cb():
            # callback 첫 줄에서 자기 타이머를 끊어 “one-shot”으로 만듦
            if self._hold_timer is not None:
                self._hold_timer.cancel()
                self._hold_timer = None
            self._try_send()

        self._hold_timer = self.create_timer(delay_sec, _cb)


def main():
    rclpy.init()
    node = WaypointActionFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
