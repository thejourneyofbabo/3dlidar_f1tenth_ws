import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    env_vars = os.environ.copy()
    env_vars['CONFIG1_PATH'] = './src/rust_ws/config.toml'
    # env_vars['CONFIG_PATH'] = './src/toml_param/vehicle_param.toml'
    env_vars['CONFIG_PATH'] = './src/follow_the_gap_param/src/vehicle_setup/vehicle_param.toml'
    # config_path = './src/rust_ws/config.toml'
    
    return LaunchDescription([
        Node(
            package='follow_the_gap_param',
            # executable='follow_the_gap_param', 
            executable='hybrid_reactive', 
            name='reactive_follow',
            output='screen',
            env=env_vars
        ),
        # Node(
        #     package='rust_ws',
        #     executable='toml_config',
        #     name='rust_ws_config_node',
        #     output='screen',
        #     env=env_vars
        #     # env={'CONFIG_PATH': config_path}
        # ),
        # Node(
        #     package='toml_param',
        #     executable='toml_reactive', 
        #     name='toml_reactive_node',
        #     output='screen',
        #     env=env_vars
        # ),
        # Node(
        #     package='toml_param',
        #     executable='hybrid_reactive', 
        #     name='hybrid_reactive',
        #     output='screen',
        #     env=env_vars
        # ),
    ])
