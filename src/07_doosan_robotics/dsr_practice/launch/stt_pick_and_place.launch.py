from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("dsr_practice"), "config", "moveit_py.yaml"]
    )

    pick_place_node = Node(
        package="dsr_practice",
        executable="stt_pick_and_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_params,
            {"use_tts": True},
        ],
    )

    stt_node = Node(
        package="dsr_practice",
        executable="stt_node",
        output="screen",
        parameters=[
            {"language": "ko-KR"},
            {"device_index": -1},
            {"energy_threshold": 300.0},
            {"pause_threshold": 0.8},
            {"phrase_time_limit": 5.0},
            {"dynamic_energy": True},
            {"ambient_duration": 1.0},
        ],
    )

    return LaunchDescription([pick_place_node, stt_node])
