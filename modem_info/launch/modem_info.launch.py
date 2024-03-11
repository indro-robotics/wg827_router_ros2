from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node( package='modem_info', 
            executable='modem_info_node', 
            name='modem_info_node',
            output='both',
            parameters=[{'modem_ip': '192.168.42.1'},
                        {'modem_user': 'root'},
                        {'modem_pass': 'indr0.com'}])
    ])
