"""Launch analyzer loader with parameters from yaml."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.actions import LogInfo, RegisterEventHandler, EmitEvent, SetLaunchConfiguration
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    diagnosticsc_package_path = get_package_share_path('dobot_diagnostics')
    analyzer_params_filepath = diagnosticsc_package_path / 'config/analyzer_params.yaml'


    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[analyzer_params_filepath],
        arguments=['--ros-args', '--log-level', 'warn']
        )
    diag_publisher = Node(
        package='dobot_diagnostics',
        executable='alarms_parser')
    alarm_clear_srv = Node(
            package='dobot_diagnostics',
            executable='alarm_clear',
            output='screen')
    usb_monitor = Node(
        package='dobot_diagnostics',
        executable='usb_monitor'
    )
    return LaunchDescription([
        LogInfo(msg='Starting the diagnostics module.'),
        SetLaunchConfiguration('already_shutdown', 'false'),
        aggregator,
        diag_publisher,
        alarm_clear_srv,
        usb_monitor,    
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=aggregator,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='Diagnostic aggregator has been shutdown!'), condition=IfCondition(PythonExpression(["'", LaunchConfiguration('already_shutdown'), "' == 'false'"]))), SetLaunchConfiguration('already_shutdown', 'true')],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=usb_monitor,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='USB has been unplugged!'), condition=IfCondition(PythonExpression(["'", LaunchConfiguration('already_shutdown'), "' == 'false'"]))), SetLaunchConfiguration('already_shutdown', 'true')]
            )
        )
    ])
