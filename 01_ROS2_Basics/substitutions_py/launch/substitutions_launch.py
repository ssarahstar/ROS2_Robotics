from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1'
    )

    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='True'
    )

    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle_late = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    PythonExpression(["'/", turtlesim_ns, "/spawn'"]),
                    'turtlesim/srv/Spawn',
                    '{x: 2, y: 2, theta: 0.2}'
                ]
            )
        ]
    )

    change_background_late = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression([
                        "'", use_provided_red, "' == 'True'"
                    ])
                ),
                cmd=[
                    'ros2', 'param', 'set',
                    PythonExpression(["'/", turtlesim_ns, "/sim'"]),
                    'background_r',
                    new_background_r
                ]
            )
        ]
    )

    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        spawn_turtle_late,
        change_background_late,
    ])