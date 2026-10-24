"""
gscam.launch.py — launch gscam_node from a dVRK abstract Unix socket.

Intended to be called via the ``gscam_socket`` helper script, which handles
socket discovery and validation before delegating here.  Can also be invoked
directly if the fully-qualified socket name is already known:

  ros2 launch dvrk_data gscam.launch.py
  ros2 launch dvrk_data gscam.launch.py socket:=@dvrk:stereo_source:stereo channel:=left

Arguments
---------
socket      Optional.  Fully-qualified abstract socket name, e.g.
            ``@dvrk:stereo_source:left``.  If omitted, active sockets
            are listed and no gscam node is launched.
namespace   ROS namespace for the gscam node (default: <role>/<name>).
frame_id    TF frame id stamped on published images (default: <role>_<name>_frame).
camera_name camera_name in CameraInfo (default: <role>_<name>).
channel     Optional left or right eye from a stereo socket.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dvrk_data import dvrk_gst_socket


_PREFIX = "@dvrk:"


def _parse_socket(fqn: str):
    """Return (role, name) from a fully-qualified ``@dvrk:role:name`` string."""
    if not fqn.startswith(_PREFIX):
        raise ValueError(
            f"Expected a fully-qualified dVRK socket starting with '{_PREFIX}', got: {fqn!r}"
        )
    rest = fqn[len(_PREFIX):]
    parts = rest.split(":", 1)
    if len(parts) != 2 or not parts[0] or not parts[1]:
        raise ValueError(
            f"Socket name must be '@dvrk:<role>:<name>', got: {fqn!r}"
        )
    return parts[0], parts[1]


def _stereo_crop(gst_path: str, channel: str) -> str:
    """Inspect a frame before choosing the crop; reject non-stereo input."""
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
    if Gst.Meta.get_info("DataCollectionFrameTimestamps") is None:
        Gst.Meta.register_custom_simple("DataCollectionFrameTimestamps")
    pipeline = Gst.parse_launch(
        f"unixfdsrc socket-path={gst_path} socket-type=abstract "
        "! appsink name=probe sync=false max-buffers=1 drop=true"
    )
    try:
        pipeline.set_state(Gst.State.PLAYING)
        sample = pipeline.get_by_name("probe").emit("try-pull-sample", 3 * Gst.SECOND)
        if sample is None:
            message = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
            detail = message.parse_error()[1] if message else "no frame received within 3 seconds"
            raise RuntimeError(f"Cannot inspect stereo socket: {detail}")
        caps = sample.get_caps().get_structure(0)
        valid, width = caps.get_int("width")
        meta = sample.get_buffer().get_custom_meta("DataCollectionFrameTimestamps")
        layout = meta.get_structure().get_string("channel") if meta else None
        if layout not in ("stereo-LR", "stereo-RL"):
            raise ValueError(
                f"Requested {channel} channel, but socket is not stereo "
                f"(frame channel: {layout or 'unknown'})."
            )
        if not valid or width < 2 or width % 2:
            reported_width = width if valid else "unknown"
            raise ValueError(f"Stereo frame has invalid width: {reported_width}")
        eye_width = width // 2
        first_eye = "left" if layout == "stereo-LR" else "right"
        if channel == first_eye:
            return f"videocrop right={eye_width} ! video/x-raw,width={eye_width}"
        return f"videocrop left={eye_width} ! video/x-raw,width={eye_width}"
    finally:
        pipeline.set_state(Gst.State.NULL)


def _launch_gscam(context, *args, **kwargs):
    fqn = LaunchConfiguration("socket").perform(context)
    if not fqn:
        sockets = dvrk_gst_socket.list_sockets()
        if not sockets:
            return [LogInfo(msg="No @dvrk abstract sockets are currently active.")]
        lines = ["Available @dvrk sockets:"]
        lines.extend(f"  {socket}" for socket in sockets)
        lines.append("")
        lines.append("Launch one with:")
        lines.append(
            f"  ros2 launch dvrk_data gscam.launch.py socket:={sockets[0]}"
        )
        return [LogInfo(msg="\n".join(lines))]

    role, name = _parse_socket(fqn)
    channel = LaunchConfiguration("channel").perform(context)
    if channel not in ("", "left", "right"):
        raise ValueError("channel must be left or right")

    def _resolve(cfg_key: str, derived: str) -> str:
        val = LaunchConfiguration(cfg_key).perform(context)
        return val if val else derived

    suffix = f"_{channel}" if channel else ""
    default_namespace = f"{role}/{name}" + (f"/{channel}" if channel else "")
    namespace = _resolve("namespace", default_namespace)
    frame_id = _resolve("frame_id", f"{role}_{name}{suffix}_frame")
    camera_name = _resolve("camera_name", f"{role}_{name}{suffix}")

    # Strip leading '@' for GStreamer's socket-path= property
    gst_path = fqn[1:]

    crop = _stereo_crop(gst_path, channel) if channel else ""

    gscam_config = (
        f"unixfdsrc socket-path={gst_path} socket-type=abstract do-timestamp=true"
        " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream"
        + (f" ! {crop}" if crop else "")
        + " ! videoconvert"
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
            default_value="",
            description=(
                "Optional fully-qualified @dvrk abstract socket name, "
                "e.g. '@dvrk:stereo_source:left'.  "
                "If omitted, list active sockets and exit."
            ),
        ),
        DeclareLaunchArgument(
            "channel",
            default_value="",
            description="Optional left or right eye; requires stereo frame metadata.",
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
