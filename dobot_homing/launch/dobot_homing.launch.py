import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('dobot_homing'),
        'config',
        'homing_params.yaml'
        )

    return LaunchDescription([
        LogInfo(msg='Starting homing service.'),
        LogInfo(msg='Loading homing parameters.'),
        Node(
            package='dobot_homing',
            executable='homing_server',
            output='screen',
        ),
    ExecuteProcess(
        cmd=[[
            'sleep 5;', 'ros2 ' , 'param ',  'load ', '/dobot_homing_srv ', config
        ]],
        shell=True
    )
    ])

