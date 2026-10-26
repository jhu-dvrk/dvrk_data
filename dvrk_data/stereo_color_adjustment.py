# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Johns Hopkins University, LCSR
"""Color matching and the shared stereo_color_adjustment.json format."""
import json
import math
from pathlib import Path
import cv2
import numpy as np

NEUTRAL_COLOR = {"brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0}
COLOR_MODES = {"Left reference": "left_ref", "Right reference": "right_ref",
               "Average": "average", "Average (luminance only)": "average_luma"}
TYPE = "dvrk_data:stereo_color_adjustment@1.0.0"

def clip_float(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def compute_stats(yuv, mask):
    pixels = yuv[mask].astype(np.float32)
    luma = pixels[:, 0]
    chroma = pixels[:, 1:] - 128.0
    return {
        "mean_y": float(np.mean(luma)),
        "std_y": float(np.std(luma)),
        "chroma_rms": float(np.sqrt(np.mean(np.sum(chroma * chroma, axis=1)))),
    }


def calculate_match_from_stats(source, target, luma_only=False):
    # A nearly flat image does not contain enough information to fit contrast.
    contrast = (clip_float(target["std_y"] / source["std_y"], 0.7, 1.3)
                if source["std_y"] >= 8.0 and target["std_y"] >= 8.0 else 1.0)
    # GStreamer videobalance applies contrast around video black (Y=16).
    brightness = (target["mean_y"] - 16.0
                  - (source["mean_y"] - 16.0) * contrast) / 255.0
    brightness = clip_float(brightness, -0.25, 0.25)
    saturation = 1.0
    if not luma_only and min(source["chroma_rms"], target["chroma_rms"]) >= 8.0:
        saturation = clip_float(target["chroma_rms"] / source["chroma_rms"], 0.7, 1.3)
    return {"brightness": brightness, "contrast": contrast,
            "saturation": saturation, "hue": 0.0}


def calculate_calibration(mode, left_bgr, right_bgr):
    neutral = dict(NEUTRAL_COLOR)
    if mode == "none":
        return neutral, neutral
    left_yuv = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2YUV)
    right_yuv = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2YUV)
    # Compare corresponding visible pixels, excluding black borders and clipped highlights.
    mask = ((left_yuv[:, :, 0] >= 24) & (left_yuv[:, :, 0] <= 235)
            & (right_yuv[:, :, 0] >= 24) & (right_yuv[:, :, 0] <= 235))
    if np.count_nonzero(mask) < max(100, mask.size // 100):
        return None
    left_stats = compute_stats(left_yuv, mask)
    right_stats = compute_stats(right_yuv, mask)
    if mode == "left_ref":
        return neutral, calculate_match_from_stats(right_stats, left_stats)
    if mode == "right_ref":
        return calculate_match_from_stats(left_stats, right_stats), neutral
    if mode in ("average", "average_luma"):
        average = {key: (left_stats[key] + right_stats[key]) / 2.0 for key in left_stats}
        luma_only = mode == "average_luma"
        return (calculate_match_from_stats(left_stats, average, luma_only),
                calculate_match_from_stats(right_stats, average, luma_only))
    raise ValueError(f"Unknown color matching mode: {mode}")


def apply_color_calibration_preview(bgr, color_params):
    brightness = color_params.get("brightness", 0.0)
    contrast = color_params.get("contrast", 1.0)
    saturation = color_params.get("saturation", 1.0)
    hue = color_params.get("hue", 0.0)
    if brightness == 0.0 and contrast == 1.0 and saturation == 1.0 and hue == 0.0:
        return bgr

    # Approximate GStreamer videobalance in YUV rather than HSV.
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV).astype(np.float32)
    yuv[:, :, 0] = np.clip(16 + (yuv[:, :, 0] - 16) * contrast
                           + brightness * 255.0, 0, 255)
    u = yuv[:, :, 1] - 128.0
    v = yuv[:, :, 2] - 128.0
    angle = np.pi * hue
    yuv[:, :, 1] = np.clip(128 + saturation * (u * np.cos(angle) + v * np.sin(angle)), 0, 255)
    yuv[:, :, 2] = np.clip(128 + saturation * (-u * np.sin(angle) + v * np.cos(angle)), 0, 255)
    return cv2.cvtColor(yuv.astype(np.uint8), cv2.COLOR_YUV2BGR)


def validate_color_file(config):
    if not isinstance(config, dict) or config.get("type") != TYPE:
        raise ValueError("Expected " + TYPE)
    for eye in ("left", "right"):
        if not isinstance(config.get(eye), dict):
            raise ValueError("Color adjustment requires left and right objects")
        for key, neutral in NEUTRAL_COLOR.items():
            value = config[eye].get(key, neutral)
            low, high = (0, 2) if key in ("contrast", "saturation") else (-1, 1)
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"Invalid {eye} {key}: {value}")
    return config


def load_color_file(path):
    path = Path(path)
    if not path.exists():
        return {"type": TYPE, "left": dict(NEUTRAL_COLOR), "right": dict(NEUTRAL_COLOR)}
    return validate_color_file(json.loads(path.read_text()))


def save_color_file(path, left, right):
    config = validate_color_file({"type": TYPE, "left": left, "right": right})
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(config, indent=2, allow_nan=False) + "\n")
