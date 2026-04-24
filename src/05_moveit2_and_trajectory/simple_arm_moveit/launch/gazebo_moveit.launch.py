import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt config (URDF/SRDF/kinematics/joint_limits/controllers 등)
    moveit_config = (
        MoveItConfigsBuilder("simple_arm", package_name="simple_arm_moveit")
        .to_moveit_configs()
    )

    # move_group만 실행 (Gazebo가 ros2_control/controller_manager를 이미 제공하므로
    # 여기서는 ros2_control_node/spawner를 띄우지 않음)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    # RViz (MoveIt 플러그인용)
    rviz_config = os.path.join(
        get_package_share_directory("simple_arm_moveit"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    return LaunchDescription([move_group_node, rviz_node])