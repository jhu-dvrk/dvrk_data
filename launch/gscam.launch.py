"""
gscam.launch.py — launch gscam_node from a dvrk_gst abstract Unix socket.

Intended to be called via the ``gscam_socket`` helper script, which handles
socket discovery and validation before delegating here.  Can also be invoked
directly if the fully-qualified socket name is already known:

  ros2 launch dvrk_data gscam.launch.py socket:=@dvrk_gst:stereo_source:left

Arguments
---------
socket      Required.  Fully-qualified abstract socket name, e.g.
            ``@dvrk_gst:stereo_source:left``.
namespace   ROS namespace for the gscam node (default: <role>/<name>).
frame_id    TF frame id stamped on published images (default: <role>_<name>_frame).
camera_name camera_name in CameraInfo (default: <role>_<name>).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PREFIX = "@dvrk_gst:"


def _parse_socket(fqn: str):
    """Return (role, name) from a fully-qualified ``@dvrk_gst:role:name`` string."""
    if not fqn.startswith(_PREFIX):
        raise ValueError(
            f"Expected a fully-qualified socket name starting with '{_PREFIX}', got: {fqn!r}"
        )
    rest = fqn[len(_PREFIX):]
    parts = rest.split(":", 1)
    if len(parts) != 2 or not parts[0] or not parts[1]:
        raise ValueError(
            f"Socket name must be '@dvrk_gst:<role>:<name>', got: {fqn!r}"
        )
    return parts[0], parts[1]


def _launch_gscam(context, *args, **kwargs):
    fqn = LaunchConfiguration("socket").perform(context)
    role, name = _parse_socket(fqn)

    def _resolve(cfg_key: str, derived: str) -> str:
        val = LaunchConfiguration(cfg_key).perform(context)
        return val if val else derived

    namespace   = _resolve("namespace",    f"{role}/{name}")
    frame_id    = _resolve("frame_id",     f"{role}_{name}_frame")
    camera_name = _resolve("camera_name",  f"{role}_{name}")

    # Strip leading '@' for GStreamer's socket-path= property
    gst_path = fqn[1:]

    gscam_config = (
        f"unixfdsrc socket-path={gst_path} socket-type=abstract do-timestamp=true"
        " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream"
        " ! videoconvert"
    )

    return [Node(
        package="gscam",
        executable="gscam_node",
        name=camera_name,
        namespace=namespace,
        output="screen",
        parameters=[{
            "gscam_config": gscam_config,
            "camera_name": camera_name,
            "frame_id": frame_id,
            "use_gst_timestamps": False,
            "sync_sink": False,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "socket",
            description=(
                "Fully-qualified @dvrk_gst abstract socket name, "
                "e.g. '@dvrk_gst:stereo_source:left'.  "
                "Use the gscam_socket script for discovery and validation."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace (default: <role>/<name>).",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="",
            description="TF frame_id (default: <role>_<name>_frame).",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="",
            description="camera_name in CameraInfo (default: <role>_<name>).",
        ),
        OpaqueFunction(function=_launch_gscam),
    ])
