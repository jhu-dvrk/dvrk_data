"""Exercise the native binding with real PyGObject buffers."""

import importlib.util
import sys
import unittest

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)
if len(sys.argv) > 1 and sys.argv[1].endswith(".so"):
    spec = importlib.util.spec_from_file_location("_video_timestamps", sys.argv.pop(1))
    timestamps = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(timestamps)
else:
    from dvrk_data import video_timestamps as timestamps


class VideoTimestampsTest(unittest.TestCase):
    def test_defaults_and_missing_metadata(self):
        buffer = Gst.Buffer.new()
        self.assertIsNone(timestamps.get_frame_timestamps(buffer))
        value = timestamps.FrameTimestamps()
        self.assertEqual(value.channel, "")
        self.assertEqual(value.left_source_ts, 0)
        self.assertEqual(value.right_source_ts, 0)

    def test_native_roundtrip_and_deep_copy(self):
        buffer = Gst.Buffer.new()
        value = timestamps.FrameTimestamps()
        value.channel = "stereo-LR"
        value.left_source_ts = 1_794_000_000_000_000_001
        value.right_source_ts = value.left_source_ts + 123
        value.source_stereo_output_ts = value.right_source_ts + 456
        value.alignment_output_ts = value.source_stereo_output_ts + 789
        value.stereo_output_ts = value.alignment_output_ts
        value.overlay_output_ts = value.alignment_output_ts + 456
        reference = timestamps.ReferenceTimestamp()
        reference.reference = "timestamp/x-test"
        reference.timestamp = 1234
        reference.duration = 5678
        value.left_reference_timestamps = [reference]
        timestamps.set_frame_timestamps(buffer, value)
        for received in (buffer, buffer.copy_deep()):
            read = timestamps.get_frame_timestamps(received)
            for field in ("channel", "left_source_ts", "right_source_ts",
                          "source_stereo_output_ts", "alignment_output_ts",
                          "stereo_output_ts", "overlay_output_ts"):
                self.assertEqual(getattr(read, field), getattr(value, field))
            self.assertEqual(read.left_reference_timestamps[0].reference, reference.reference)
            self.assertEqual(read.left_reference_timestamps[0].timestamp, 1234)
            self.assertEqual(read.left_reference_timestamps[0].duration, 5678)
            self.assertEqual(read.right_reference_timestamps, [])
        # Reading returns an independent value, not a writable metadata pointer.
        read.left_source_ts = 0
        self.assertEqual(timestamps.get_frame_timestamps(buffer).left_source_ts, value.left_source_ts)

    def test_mono_update(self):
        buffer = Gst.Buffer.new()
        value = timestamps.FrameTimestamps()
        value.channel = "mono"
        value.mono_source_ts = timestamps.clock_realtime_ns()
        timestamps.set_frame_timestamps(buffer, value)
        value.mono_source_ts += 1
        timestamps.set_frame_timestamps(buffer, value)
        read = timestamps.get_frame_timestamps(buffer)
        self.assertEqual(read.mono_source_ts, value.mono_source_ts)
        self.assertEqual(read.left_source_ts, 0)

    def test_invalid_objects_are_rejected(self):
        for object_ in (None, 1234, object(), Gst.Caps.from_string("video/x-raw")):
            with self.subTest(object=object_):
                with self.assertRaises(TypeError):
                    timestamps.set_frame_timestamps(object_, timestamps.FrameTimestamps())
                with self.assertRaises(TypeError):
                    timestamps.get_frame_timestamps(object_)

    def test_shared_buffer_is_rejected(self):
        buffer = Gst.Buffer.new()
        sample = Gst.Sample.new(buffer, None, None, None)
        with self.assertRaisesRegex(ValueError, "writable"):
            timestamps.set_frame_timestamps(buffer, timestamps.FrameTimestamps())
        self.assertIsNone(timestamps.get_frame_timestamps(buffer))
        del sample
        timestamps.set_frame_timestamps(buffer, timestamps.FrameTimestamps())

    def test_registration_is_idempotent(self):
        timestamps.register_frame_timestamps()
        timestamps.register_frame_timestamps()


if __name__ == "__main__":
    unittest.main()
