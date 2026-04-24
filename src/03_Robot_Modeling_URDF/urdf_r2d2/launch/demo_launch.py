import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    pkg_share = get_package_share_directory("urdf_r2d2")

    default_model_path = os.path.join(pkg_share, "urdf", "r2d2.urdf.xacro")
    default_rviz_path = os.path.join(pkg_share, "rviz", "r2d2.rviz")

    model = LaunchConfiguration("model")
    rvizconfig = LaunchConfiguration("rvizconfig")

    # Xacro -> URDF 문자열로 변환되어 robot_description 파라미터로 들어감
    robot_desc = Command(["xacro ", model])

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=default_model_path,
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=default_rviz_path,
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
            }],
        ),

        Node(
            package="urdf_r2d2",
            executable="state_publisher",
            name="state_publisher",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rvizconfig],
        ),
    ])