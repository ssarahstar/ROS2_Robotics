import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = "simple_arm_description"
    pkg_share = get_package_share_directory(pkg)

    urdf_path = os.path.join(pkg_share, "urdf", "simple_arm.urdf.xacro")

    # Gazebo Classic 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    # robot_description에 URDF(xacro) 문자열 넣기
    robot_desc = xacro.process_file(urdf_path).toxml()

    # gazebo_ros2_control 파라미터 override 파싱 문제 방지용(기존 코드 스타일 유지)
    robot_desc = re.sub(r"<!--.*?-->", "", robot_desc, flags=re.DOTALL)
    robot_desc = " ".join(robot_desc.split())

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": True,
        }],
    )

    # Gazebo에 스폰
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "simple_arm",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
    )

    # 컨트롤러 스폰 (이름 통일: arm_controller)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    # Gazebo + gazebo_ros2_control이 올라올 시간을 조금 주고 스폰(안정화)
    spawn_controllers_delayed = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        spawn_controllers_delayed,
    ])