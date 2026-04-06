import os
import time

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_udp_bridge = get_package_share_directory("capra_udp_bridge")
    
    demo_config_file = os.path.join(pkg_udp_bridge, "config", "demo_config.yaml")
    dir_dec = DeclareLaunchArgument("dir", choices=["rx", "tx"], default_value="rx")
    dir = LaunchConfiguration("dir")
    
    return LaunchDescription([
        dir_dec,
        Node(
            package='capra_udp_bridge',
            executable='udp_bridge_node',
            name=['udp_bridge_node_', dir],
            parameters=[demo_config_file],
            ros_arguments=["--log-level", "my_node:=debug"],
        ),
    ])