# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Johns Hopkins University, LCSR
"""Resolve pipeline configuration and explicit processor selection without ROS."""
import json
from pathlib import Path


def config_path(directory, value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else Path(directory) / path


def select_processor(directory, processor="auto", alignment="stereo_alignment.json",
                     rectify="stereo_rectify.json"):
    paths = {"alignment": config_path(directory, alignment),
             "rectify": config_path(directory, rectify)}
    if processor not in ("auto", "alignment", "rectify"):
        raise RuntimeError("processor must be auto, alignment or rectify")
    if processor == "auto":
        available = [name for name, path in paths.items() if path.is_file()]
        if len(available) == 2:
            raise RuntimeError("Both stereo_alignment.json and stereo_rectify.json are available. "
                               "Choose processor:=alignment or processor:=rectify explicitly.")
        if not available:
            raise RuntimeError("No stereo processing configuration found. Create stereo_alignment.json "
                               "or stereo_rectify.json, or specify its path using the named launch argument.")
        processor = available[0]
    if not paths[processor].is_file():
        raise RuntimeError("Missing selected processor configuration: " + str(paths[processor]))
    return "stereo_" + processor, paths[processor]


def output_socket(config, default):
    data = json.loads(Path(config).read_text())
    value = data.get("gst_output", default)
    if not isinstance(value, str) or not value.startswith("@dvrk:"):
        raise RuntimeError("Pipeline requires a stereo output socket in " + str(config))
    return value


def color_config(directory, explicit=""):
    path = config_path(directory, explicit or "stereo_color_adjustment.json")
    if explicit and not path.is_file():
        raise RuntimeError("Missing explicitly selected color adjustment: " + str(path))
    return path if path.is_file() else None
