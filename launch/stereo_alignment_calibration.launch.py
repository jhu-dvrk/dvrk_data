"""Start stereo_source, then the interactive stereo_alignment_calibration tool.

Run from a directory with stereo_source.json and stereo_alignment.json::

    ros2 launch dvrk_data stereo_alignment_calibration.launch.py
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_SOURCE_CONFIG = "stereo_source.json"
DEFAULT_ALIGNMENT_CONFIG = "stereo_alignment.json"


def _config_path(base_dir, value):
    value = os.path.expanduser(value.strip())
    return value if os.path.isabs(value) else os.path.join(base_dir, value)


def _launch_processes(context, *args, **kwargs):
    config_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration("config_dir").perform(context)))
    source_path = _config_path(
        config_dir, LaunchConfiguration("source_config").perform(context))
    alignment_path = _config_path(
        config_dir, LaunchConfiguration("alignment_config").perform(context))

    if not os.path.isfile(source_path):
        raise RuntimeError("Missing source configuration file: " + source_path)
    # The calibration tool creates stereo_alignment.json when it is absent.

    calibration_executable = os.path.join(
        get_package_prefix("dvrk_data"), "lib", "dvrk_data",
        "stereo_alignment_calibration")
    calibration = ExecuteProcess(
        cmd=[calibration_executable, "-c", alignment_path],
        output="screen",
    )
    return [
        Node(
            package="dvrk_data",
            executable="stereo_source",
            name="stereo_source",
            output="screen",
            arguments=["-c", source_path],
        ),
        TimerAction(period=2.0, actions=[calibration]),
        RegisterEventHandler(OnProcessExit(
            target_action=calibration,
            on_exit=[Shutdown(reason="Calibration finished")],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_dir",
            default_value=os.getcwd(),
            description="Directory containing the source config and optional alignment config; defaults to the current working directory.",
        ),
        DeclareLaunchArgument(
            "source_config",
            default_value=DEFAULT_SOURCE_CONFIG,
            description="Source config filename relative to config_dir, or absolute path.",
        ),
        DeclareLaunchArgument(
            "alignment_config",
            default_value=DEFAULT_ALIGNMENT_CONFIG,
            description="Alignment config filename relative to config_dir, or absolute path.",
        ),
        OpaqueFunction(function=_launch_processes),
    ])
