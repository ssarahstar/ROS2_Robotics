from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    
    #Turtlesim 노드
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )

    #동적 속도 제어 노드 
    speed_ctrl_node = Node(
        package='multi_turtle_launch',
        executable='speed_controller',
        name='speed_ctrl'
    )

    #OnProcessStart 이벤트 핸들러: 제어 노드가 켜지면 로그 출력
    on_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=speed_ctrl_node,
            on_start=[
                LogInfo(msg="속도 제어 노드 시작!")
            ]
        )
    )

    #OnProcessExit 이벤트 핸들러: Turtlesim 창이 꺼지면 전체 시스템 종료
    on_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim_node,
            on_exit=[
                LogInfo(msg="터틀심 종료 → 시스템 종료"),
                EmitEvent(event=Shutdown())
            ]
        )
    )

    return LaunchDescription([
        turtlesim_node,
        speed_ctrl_node,
        on_start_handler,
        on_exit_handler
    ])