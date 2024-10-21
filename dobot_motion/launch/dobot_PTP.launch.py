import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('dobot_motion'),
        'config',
        'PTP_motion_params.yaml'
        )

    return LaunchDescription([
        LogInfo(msg='Starting PointToPoint action server.'),
        LogInfo(msg='Setting speed and acceleration values.'),
        Node(
            package='dobot_motion',
            executable='PTP_server',
            output='screen',
        ),
    ExecuteProcess(
        cmd=[[
            'sleep 5;', 'ros2 param load /dobot_PTP_server ', config
        ]],
        shell=True
    )
    ])