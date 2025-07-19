import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = './src/rust_ws/config.toml'
    vehicle_param = './src/toml_param/vehicle_param.toml'
    
    return LaunchDescription([
        # 환경변수 설정
        # SetEnvironmentVariable(
        #     name='CONFIG_PATH',
        #     value=config_path
        # ),

        # Node(
        #     package='rust_ws',
        #     executable='toml_config',
        #     name='rust_ws_config_node',
        #     output='screen',
        #     env={'CONFIG_PATH': config_path}
        # ),
        Node(
            package='toml_param',
            executable='toml_param', 
            name='toml_param_node',
            output='screen',
            env={'CONFIG_PATH': vehicle_param}
        ),
    ])

