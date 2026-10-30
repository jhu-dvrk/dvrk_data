# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Johns Hopkins University, LCSR
"""Regression tests for sidecar observations that differ from encoded frames.

Run in a sourced ROS environment: python3 tests/extract_timestamps_test.py
"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dvrk_data import extract


class ExtractionTimestampTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.video = self.directory / 'stereo.mp4'
        writer = cv2.VideoWriter(str(self.video), cv2.VideoWriter_fourcc(*'mp4v'), 12, (64, 32))
        self.assertTrue(writer.isOpened())
        for i in range(36):
            frame = np.zeros((32, 64, 3), dtype=np.uint8)
            frame[:, :32] = (20 + i * 3, 30, 180)
            frame[:, 32:] = (160, 20 + i * 3, 40)
            writer.write(frame)
        writer.release()
        # 24 observed frames/sec, but only 12 encoded frames/sec. Late tags
        # carry frame_index values beyond the encoded video's last frame.
        self.epoch = 1_700_000_000_000_000_000
        self.frames = [
            {'frame_index': i, 'gst': {'pts_ns': i * 10**9 // 24},
             'cpu': {'realtime': {'mono_source_ns': self.epoch + i * 10**9 // 24}}}
            for i in range(72)]
        self.sidecar = self.directory / 'stereo.json'
        self.sidecar.write_text(json.dumps({
            'type': 'dvrk_data:video_sidecar@1.0.0', 'video_file': self.video.name,
            'fps': 30, 'side_by_side': 'LR', 'frames': self.frames}))
        extract.video_timeline.cache_clear()

    def run_extract(self, name, formats=('mp4',), split=True, start=48, end=60):
        output = self.directory / name
        output.mkdir()
        extract.extract_session_data(
            str(self.sidecar), str(output), formats, 1,
            None if start is None else extract.explicit_annotation_timestamp_ns(self.frames[start]),
            None if end is None else extract.explicit_annotation_timestamp_ns(self.frames[end]),
            split_stereo=split)
        return output

    def read_video(self, path):
        cap = cv2.VideoCapture(str(path))
        self.assertTrue(cap.isOpened())
        fps = cap.get(cv2.CAP_PROP_FPS)
        frames = []
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frames.append(frame)
        cap.release()
        return fps, frames

    def test_late_tags_split_video_and_images(self):
        output = self.run_extract('late', formats=('mp4', 'png'))
        left_fps, left = self.read_video(output / 'stereo_left.mp4')
        right_fps, right = self.read_video(output / 'stereo_right.mp4')
        self.assertEqual((left_fps, right_fps), (12, 12))
        self.assertEqual((len(left), len(right)), (7, 7))
        self.assertEqual(left[0].shape, (32, 32, 3))
        _, original = self.read_video(self.video)
        for output_frame, source_frame in ((0, 24), (-1, 30)):
            np.testing.assert_allclose(left[output_frame].mean(axis=(0, 1)),
                                       original[source_frame][:, :32].mean(axis=(0, 1)), atol=6)
            np.testing.assert_allclose(right[output_frame].mean(axis=(0, 1)),
                                       original[source_frame][:, 32:].mean(axis=(0, 1)), atol=6)
        self.assertEqual(len(list(output.glob('*_left_*.png'))), 7)
        self.assertEqual(len(list(output.glob('*_right_*.png'))), 7)

    def assert_sidecar(self, output, name, source_data, expected_eye):
        path = output / (name + '.json')
        self.assertTrue(path.exists())
        data = json.loads(path.read_text())
        video_pts, fps = extract.video_timeline(str(output / (name + '.mp4')))
        self.assertEqual(data['video_file'], name + '.mp4')
        self.assertEqual(data['frames_recorded'], len(video_pts))
        self.assertEqual(data['fps'], fps)
        self.assertEqual(data['extraction']['eye'], expected_eye)
        self.assertEqual(data['extraction']['source_side_by_side'], source_data['side_by_side'])
        self.assertEqual(data['extraction']['frame_mapping'], 'nearest_sidecar_pts')
        self.assertEqual(data['side_by_side'], source_data['side_by_side'] if expected_eye == 'unsplit' else 'undefined')
        self.assertEqual((path.parent / data['extraction']['source_video_file']).resolve(), self.video)
        self.assertEqual((path.parent / data['extraction']['source_sidecar_file']).resolve(), self.sidecar)
        self.assertEqual([f['frame_index'] for f in data['frames']], list(range(len(video_pts))))
        self.assertEqual([f['gst']['pts_ns'] for f in data['frames']], list(video_pts))
        self.assertEqual(video_pts[0], 0)
        for frame in data['frames']:
            source = source_data['frames'][frame['source']['sidecar_index']]
            self.assertEqual(frame['cpu'], source['cpu'])
            self.assertEqual(frame['source']['gst'], source['gst'])
            self.assertEqual(frame['source']['frame_index'], source['frame_index'])
            self.assertEqual(frame.get('reference_timestamps'), source.get('reference_timestamps'))
        return data

    def test_split_sidecars_preserve_both_eye_timestamps_for_lr_and_rl(self):
        for layout in ('LR', 'RL'):
            source = json.loads(self.sidecar.read_text())
            source['side_by_side'] = layout
            source['timestamp_statistics'] = {'from_unixfd': 72, 'at_reception': 0}
            for f in source['frames']:
                realtime = f['cpu']['realtime']
                t = extract.explicit_annotation_timestamp_ns(f)
                realtime['mono_source_ns'] = None
                realtime['left_source_ns'] = t - 1000
                realtime['right_source_ns'] = t + 1000
                realtime['recorder_reception_ns'] = t + 5000
                f['reference_timestamps'] = {'left': [{'timestamp_ns': t - 1000}], 'right': []}
            self.sidecar.write_text(json.dumps(source))
            output = self.run_extract('sidecars_' + layout)
            left = self.assert_sidecar(output, 'stereo_left', source, 'left')
            right = self.assert_sidecar(output, 'stereo_right', source, 'right')
            self.assertEqual(len(left['frames']), 7)
            for l, r in zip(left['frames'], right['frames']):
                self.assertEqual(l['cpu'], r['cpu'])
                self.assertIsNone(l['cpu']['realtime']['mono_source_ns'])
                self.assertNotEqual(l['cpu']['realtime']['left_source_ns'], l['cpu']['realtime']['right_source_ns'])
            self.assertEqual(left['frames'][0]['source']['video_frame_index'], 24)
            self.assertEqual(left['frames'][-1]['source']['video_frame_index'], 30)
            self.assertEqual(left['start_cpu_realtime_ns'], left['frames'][0]['cpu']['realtime']['recorder_reception_ns'])
            self.assertEqual(left['end_cpu_realtime_ns'], left['frames'][-1]['cpu']['realtime']['recorder_reception_ns'])
            self.assertEqual(json.loads(self.sidecar.read_text()), source)

    def test_nonzero_recorder_origins_match_the_zero_based_mp4(self):
        for origin in (20_311_938_412, 307_217_843_676):
            source = json.loads(self.sidecar.read_text())
            # Reset instead of accumulating the origin across subtests.
            for frame, original in zip(source['frames'], self.frames):
                frame['gst']['pts_ns'] = original['gst']['pts_ns'] + origin
            self.sidecar.write_text(json.dumps(source))
            output = self.run_extract('origin_' + str(origin), formats=('mp4', 'png'))
            data = self.assert_sidecar(output, 'stereo_left', source, 'left')
            self.assertEqual(len(data['frames']), 7)
            self.assertEqual(data['frames'][0]['source']['video_frame_index'], 24)
            self.assertEqual(data['frames'][-1]['source']['video_frame_index'], 30)
            self.assertEqual(data['extraction']['source_pts_origin_ns'], origin)
            self.assertEqual(len(list(output.glob('*_left_*.png'))), 7)
            self.assertEqual(json.loads(self.sidecar.read_text()), source)

    def test_sparse_observations_still_give_one_record_per_output_frame(self):
        source = json.loads(self.sidecar.read_text())
        source['frames'] = source['frames'][::4]
        self.sidecar.write_text(json.dumps(source))
        output = self.run_extract('sparse')
        data = self.assert_sidecar(output, 'stereo_left', source, 'left')
        self.assertEqual(len(data['frames']), 7)
        rows = [frame['source']['sidecar_index'] for frame in data['frames']]
        self.assertLess(len(set(rows)), len(rows))

    def test_full_session_uses_all_encoded_frames(self):
        output = self.run_extract('full', split=False, start=None, end=None)
        fps, frames = self.read_video(output / 'stereo.mp4')
        self.assertEqual(fps, 12)
        self.assertEqual(len(frames), 36)
        self.assertEqual(frames[0].shape, (32, 64, 3))
        self.assert_sidecar(output, 'stereo', json.loads(self.sidecar.read_text()), 'unsplit')

    def test_nonuniform_presentation_times_and_bounds(self):
        pts = (0, 40, 120, 200, 240)
        self.assertEqual(list(extract.video_frame_range(pts, 50, 180)), [2, 3])
        self.assertEqual(list(extract.video_frame_range(pts, 40, 240)), [1, 2, 3, 4])
        with self.assertRaises(ValueError):
            extract.video_frame_range(pts, 300, 400)
        with self.assertRaises(ValueError):
            extract.video_frame_range(pts, 180, 50)

    def test_reordered_packet_timestamps_use_presentation_order(self):
        data = {'streams': [{'codec_name': 'h264', 'nb_frames': '4',
                             'time_base': '1/1000', 'avg_frame_rate': '25/1'}],
                'packets': [{'pts': tick} for tick in (0, 120, 40, 80)]}
        with patch.object(extract.subprocess, 'run') as probe:
            probe.return_value.stdout = json.dumps(data)
            pts, fps = extract.video_timeline('reordered.mp4')
        self.assertEqual(pts, (0, 40_000_000, 80_000_000, 120_000_000))
        self.assertEqual(fps, 25)

    def test_fallback_reads_decoded_frame_timestamps(self):
        from types import SimpleNamespace
        stream = {'streams': [{'time_base': '1/1000', 'avg_frame_rate': '25/1'}]}
        frames = {'frames': [{'best_effort_timestamp': tick} for tick in (0, 40, 120)]}
        with patch.object(extract.subprocess, 'run', side_effect=[
                SimpleNamespace(stdout=json.dumps(stream)),
                SimpleNamespace(stdout=json.dumps(frames))]):
            pts, _ = extract.video_timeline('fallback.mp4')
        self.assertEqual(pts, (0, 40_000_000, 120_000_000))

    def test_probe_failure_does_not_fall_back_to_sidecar_indices(self):
        with patch.object(extract.subprocess, 'run', side_effect=FileNotFoundError):
            with self.assertRaisesRegex(RuntimeError, 'ffprobe'):
                extract.video_timeline('missing.mp4')

    def test_failed_extraction_does_not_write_sidecars(self):
        with patch.object(extract, 'extract_video_range', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'Video extraction failed'):
                self.run_extract('failed')
        self.assertEqual(list((self.directory / 'failed').glob('*.json')), [])

    def test_decode_failure_is_reported(self):
        self.assertFalse(extract.extract_video_range(
            str(self.video), str(self.directory / 'invalid.mp4'), 50, 51, 12))

    def test_main_keeps_cpu_boundaries_for_csv(self):
        index = {'type': 'dvrk_data:index@1.0.0', 'rosbags': 'rosbag',
                 'streams': [{'videos': [{'file': 'stereo.mp4'}]}]}
        (self.directory / 'index.json').write_text(json.dumps(index))
        (self.directory / 'rosbag').mkdir()
        start, end = self.epoch + 2_000_000_000, self.epoch + 2_500_000_000
        tags = {'type': 'dvrk_data:video_tags@1.0.0', 'video_file': 'stereo.mp4',
                'stages': [{'name': 'suture', 'start': {'cpu_ts': start, 'frame_relative': 0},
                            'end': {'cpu_ts': end, 'frame_relative': 1}}]}
        tags_path = self.directory / 'tags.json'
        tags_path.write_text(json.dumps(tags))
        with patch.object(sys, 'argv', ['extract', '-d', str(self.directory), '-t', str(tags_path)]), \
                patch.object(extract, 'extract_session_data') as video, \
                patch.object(extract, 'rosbag_to_csv') as csv:
            extract.main()
        self.assertEqual(video.call_args.args[4:6], (start, end))
        self.assertEqual(csv.call_args.args[2:], (start, end))


class CsvRangeTest(unittest.TestCase):
    def test_range_seek_preserves_boundaries_and_variable_columns(self):
        import csv
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from std_msgs.msg import Float64MultiArray

        for storage_id in ('mcap', 'sqlite3'):
            with self.subTest(storage_id=storage_id), tempfile.TemporaryDirectory() as temp:
                directory = Path(temp)
                bag = directory / 'bag'
                writer = rosbag2_py.SequentialWriter()
                writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
                            rosbag2_py.ConverterOptions('', ''))
                for i, topic in enumerate(('/first', '/second')):
                    writer.create_topic(rosbag2_py.TopicMetadata(
                        id=i, name=topic, type='std_msgs/msg/Float64MultiArray',
                        serialization_format='cdr'))
                # Intentionally write out of timestamp order: readers must
                # use received timestamps for the early stop to be correct.
                for topic, timestamp, values in [
                        ('/first', 100, [0.] * 7),
                        ('/first', 300, [3., 4., 5.]),
                        ('/first', 200, [2.]),
                        ('/second', 250, [6., 7.]),
                        ('/first', 400, [0.] * 9)]:
                    writer.write(topic, serialize_message(Float64MultiArray(data=values)), timestamp)
                del writer
                output = directory / 'csv'
                output.mkdir()
                with patch.object(extract, 'get_message', wraps=extract.get_message) as lookup:
                    extract.rosbag_to_csv(str(bag), str(output), 200, 300)
                self.assertEqual(lookup.call_count, 1)
                with (output / 'first.csv').open() as stream:
                    rows = list(csv.DictReader(line for line in stream if not line.startswith('#')))
                self.assertEqual([row['timestamp'] for row in rows], ['200', '300'])
                self.assertEqual(rows[0]['data.0'], '2.0')
                self.assertEqual(rows[0]['data.2'], '')
                self.assertEqual(rows[1]['data.2'], '5.0')
                self.assertNotIn('data.3', rows[0])
                self.assertTrue((output / 'second.csv').exists())


if __name__ == '__main__':
    unittest.main()
