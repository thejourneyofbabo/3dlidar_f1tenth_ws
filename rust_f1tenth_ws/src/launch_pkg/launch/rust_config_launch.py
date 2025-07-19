import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # config.toml의 상대 경로 (워크스페이스 루트 기준)
    config_path = './src/rust_ws/config.toml'
    
    return LaunchDescription([
        # 환경변수 설정
        SetEnvironmentVariable(
            name='CONFIG_PATH',
            value=config_path
        ),
        
        # 노드 실행
        Node(
            package='rust_ws',
            executable='toml_config',
            name='rust_run_toml_config',
            output='screen',
        )
    ])
