"""Start the stereo source and the uniquely selected geometry processor."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from dvrk_data.stereo_pipeline_selection import config_path, select_processor, output_socket, color_config


def _launch(context):
    directory = os.path.abspath(os.path.expanduser(LaunchConfiguration("config_dir").perform(context)))
    source_path = config_path(directory, LaunchConfiguration("source_config").perform(context))
    if not source_path.is_file():
        raise RuntimeError("Missing source configuration: " + str(source_path))
    executable, processor_path = select_processor(directory,
        LaunchConfiguration("processor").perform(context),
        LaunchConfiguration("alignment_config").perform(context),
        LaunchConfiguration("rectify_config").perform(context))
    colors = color_config(directory, LaunchConfiguration("color_config").perform(context))
    arguments = ["-c", str(processor_path), "-i", output_socket(source_path, "@dvrk:stereo_source:stereo")]
    if colors:
        arguments += ["--color-adjustment", str(colors)]
    source = Node(package="dvrk_data", executable="stereo_source", output="screen",
                  arguments=["-c", str(source_path)])
    processing = Node(package="dvrk_data", executable=executable, output="screen", arguments=arguments)
    return [source, processing, RegisterEventHandler(OnProcessExit(target_action=processing,
        on_exit=[Shutdown(reason="Stereo processor exited")]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_dir", default_value=os.getcwd()),
        DeclareLaunchArgument("source_config", default_value="stereo_source.json"),
        DeclareLaunchArgument("processor", default_value="auto", choices=["auto", "alignment", "rectify"]),
        DeclareLaunchArgument("alignment_config", default_value="stereo_alignment.json"),
        DeclareLaunchArgument("rectify_config", default_value="stereo_rectify.json"),
        DeclareLaunchArgument("color_config", default_value="", description="Optional shared color JSON path"),
        OpaqueFunction(function=_launch),
    ])
