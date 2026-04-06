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
    config_dec = DeclareLaunchArgument("config", default_value=demo_config_file)
    config = LaunchConfiguration("config")
    
    return LaunchDescription([
        config_dec,
        Node(
            package='capra_udp_bridge',
            executable='udp_bridge_node',
            name=f"udp_{int(time.time())}",
            parameters=[config],
        ),
    ])