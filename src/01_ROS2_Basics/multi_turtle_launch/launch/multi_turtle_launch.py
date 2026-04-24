from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction,ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration,FindExecutable, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():


    #Launch Argument 선언 
    bg_r_arg = DeclareLaunchArgument(
        'bg_r',
        default_value=TextSubstitution(text='69'),
        description='Background Red value for turtle_a'
    )


    bg_r_val = LaunchConfiguration('bg_r')


    #turtlesim_node 2개 실행
    turtle_a = Node(
        package='turtlesim',
        executable='turtlesim_node',
        namespace='turtle_a',
        name='sim'
    )


    turtle_b = Node(
        package='turtlesim',
        executable='turtlesim_node',
        namespace='turtle_b',
        name='sim'
    )


    #turtle_a의 background_r 파라미터 변경
    change_bg = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name='ros2'),
                    ' param set /turtle_a/sim',
                    ' background_r ', bg_r_val
                ],
                shell=True
            )
        ]
    )


    #실행 완료 메시지
    log_msg = LogInfo(
        msg='거북이 노드 실행 완료!'
    )


    return LaunchDescription([
        bg_r_arg,
        turtle_a,
        turtle_b,
        change_bg,
        log_msg,
    ])
