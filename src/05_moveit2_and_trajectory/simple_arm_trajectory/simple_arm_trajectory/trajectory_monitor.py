#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['joint1_z', 'joint1_y', 'joint2', 'joint3']
WAYPOINTS   = [[0.0,0.0,0.0,0.0],[0.2,0.1,0.0,0.0],
               [0.0,0.3,-0.2,0.0],[0.2,0.3,-0.2,0.0]]
THRESHOLD   = 0.01
TIMEOUT_SEC = 5.0

class TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        
        self._ac = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self.create_subscription(
            JointTrajectoryControllerState,       
            '/arm_controller/controller_state',
            self._state_cb, 10)

        self._error       = None
        self._wp_idx      = 0
        self._elapsed     = 0.0
        self._check_timer = None

        self._start_timer = self.create_timer(1.0, self._start)

    def _state_cb(self, msg: JointTrajectoryControllerState):
        self._error = list(msg.error.positions)   

    def _start(self):
        self._start_timer.cancel()
        self._start_timer = None
        self._send_wp(self._wp_idx)

    def _send_wp(self, idx: int):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = WAYPOINTS[idx]
        pt.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [pt]

        self.get_logger().info(f'WP {idx} 전송: {WAYPOINTS[idx]}')

        self._ac.wait_for_server()
        fut = self._ac.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._goal_cb(f, idx))

    def _goal_cb(self, future, idx: int):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'WP {idx} 거부됨')
            return

        res_fut = handle.get_result_async()
        res_fut.add_done_callback(
            lambda f: self._on_trajectory_done(f, idx))

    def _on_trajectory_done(self, future, idx: int):
        self._elapsed = 0.0
        self._check_timer = self.create_timer(
            0.2, lambda: self._check_convergence(idx))

    def _check_convergence(self, idx: int):
        self._elapsed += 0.2
        if self._error is None:
            return

        err_str = ' '.join(
            f'{n}={abs(e):.3f}' for n, e in zip(JOINT_NAMES, self._error))
        self.get_logger().info(f'[모니터] WP {idx} 오차: {err_str}')

        converged = all(abs(e) < THRESHOLD for e in self._error)
        timed_out = self._elapsed > TIMEOUT_SEC  

        if converged or timed_out:
            self._check_timer.cancel()
            self._check_timer = None

            if timed_out and not converged:
                self.get_logger().warn(
                    f'[경고] WP {idx} 수렴 실패, 다음으로 진행')

            self._wp_idx += 1
            if self._wp_idx >= len(WAYPOINTS):   
                self.get_logger().info('모든 궤적 완료!')
                raise SystemExit

            self._send_wp(self._wp_idx)           

def main():
    rclpy.init()
    node = TrajectoryMonitor()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()