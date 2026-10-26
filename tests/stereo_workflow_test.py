# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Johns Hopkins University, LCSR
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dvrk_data.stereo_pipeline_selection import select_processor, color_config, output_socket
from dvrk_data.stereo_color_adjustment import load_color_file, save_color_file, NEUTRAL_COLOR


class WorkflowTest(unittest.TestCase):
    def test_processor_selection(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            with self.assertRaisesRegex(RuntimeError, "No stereo processing"):
                select_processor(directory)
            alignment = directory / "stereo_alignment.json"
            alignment.write_text("{}")
            self.assertEqual(select_processor(directory)[0], "stereo_alignment")
            rectify = directory / "stereo_rectify.json"
            rectify.write_text("{}")
            with self.assertRaisesRegex(RuntimeError, "processor:=alignment or processor:=rectify"):
                select_processor(directory)
            self.assertEqual(select_processor(directory, "rectify"), ("stereo_rectify", rectify))
            alignment.unlink()
            self.assertEqual(select_processor(directory)[0], "stereo_rectify")
            with self.assertRaisesRegex(RuntimeError, "Missing selected"):
                select_processor(directory, "alignment")
            self.assertEqual(output_socket(rectify, "@dvrk:stereo_rectify:stereo"), "@dvrk:stereo_rectify:stereo")

    def test_color_file_independent_of_geometry(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "stereo_color_adjustment.json"
            self.assertIsNone(color_config(temp))
            with self.assertRaises(RuntimeError):
                color_config(temp, "missing.json")
            neutral = load_color_file(path)
            self.assertEqual(neutral["left"], NEUTRAL_COLOR)
            left = dict(NEUTRAL_COLOR, brightness=0.1)
            save_color_file(path, left, NEUTRAL_COLOR)
            self.assertEqual(load_color_file(path)["left"], left)
            self.assertEqual(color_config(temp), path)
            self.assertEqual(set(json.loads(path.read_text())), {"type", "left", "right"})
            with self.assertRaises(ValueError):
                save_color_file(path, dict(left, contrast=3), NEUTRAL_COLOR)

    def test_alignment_saves_only_geometry(self):
        functions = runpy.run_path(str(ROOT / "scripts" / "stereo_alignment_calibration"), run_name="alignment_calibration_test")
        config = {"camera": {"left": {"color": {"brightness": 0.1}},
                             "right": {"color": {"contrast": 0.9}}}}
        functions["update_config_from_runtime"](config, "@dvrk:stereo_source:stereo",
                                                 640, 480, 600, 460, 20, 4)
        self.assertNotIn("left", config["camera"])
        self.assertNotIn("right", config["camera"])
        self.assertEqual(config["camera"]["alignment"]["horizontal_shift_px"], 20)

    def test_named_cli_only(self):
        build = Path(os.environ["DVRK_DATA_TEST_BUILD"])
        environment = dict(os.environ, DISPLAY="", PYTHONPATH=str(ROOT))
        for name in ("charuco_stereo_calibrator", "verify_stereo_calibration", "video_tag", "video_latency", "video_configurator"):
            result = subprocess.run([str(build / name), "implicit_input.json"], env=environment,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0, name)
            self.assertIn("argument", result.stdout.lower(), name + ": " + result.stdout)
        for name in ("gscam_socket", "stereo_find_color_adjustment", "stereo_alignment_calibration"):
            result = subprocess.run([sys.executable, str(ROOT / "scripts" / name), "implicit_input.json"],
                                    env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=5)
            self.assertEqual(result.returncode, 2, name + ": " + result.stdout)
        help_text = subprocess.check_output([str(build / "verify_stereo_calibration"), "--help"], text=True)
        self.assertIn("--calibration", help_text)
        self.assertIn("--input", help_text)


if __name__ == "__main__":
    unittest.main()
