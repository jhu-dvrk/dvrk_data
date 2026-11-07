"""Read/write the native dVRK frame metadata on PyGObject Gst.Buffer objects.

Import this module before starting unixfdsrc so it can deserialize the metadata.
Timestamp values use CLOCK_REALTIME nanoseconds; zero marks an unavailable stage.
"""

from ._video_timestamps import (
    FrameTimestamps,
    ReferenceTimestamp,
    clock_realtime_ns,
    get_frame_timestamps,
    register_frame_timestamps,
    set_frame_timestamps,
)

__all__ = [
    "FrameTimestamps", "ReferenceTimestamp", "clock_realtime_ns",
    "get_frame_timestamps", "register_frame_timestamps", "set_frame_timestamps",
]
