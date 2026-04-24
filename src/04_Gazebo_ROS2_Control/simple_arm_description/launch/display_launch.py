import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 공유 디렉토리 경로 설정
    pkg = get_package_share_directory('simple_arm_description')
    
    # URDF 및 RViz 설정 파일 경로 조인
    urdf_path = os.path.join(pkg, 'urdf', 'simple_arm.urdf')
    rviz_path = os.path.join(pkg, 'rviz', 'urdf.rviz')
    
    # URDF 파일 읽기
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
        
    return LaunchDescription([
        # 1. joint_state_publisher_gui: 슬라이더로 관절 값 조정
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # 2. robot_state_publisher: URDF와 관절 값으로 TF 계산
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 3. rviz2: 로봇 모델 시각화
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        ),
    ])