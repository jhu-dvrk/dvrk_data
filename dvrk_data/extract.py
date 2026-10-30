#!/usr/bin/env python3
import cv2
import json
import os
import argparse
import sys
import threading
import csv
import copy
import rosbag2_py
import multiprocessing
import math
import numpy as np
import re
import subprocess
from bisect import bisect_left
from fractions import Fraction
from functools import lru_cache
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from .common import get_current_timestamp_iso8601


def parse_stage_timestamp(value):
    """Return the current video_tags stage timestamp object fields."""
    if not isinstance(value, dict):
        return None, None
    return value.get("cpu_ts"), value.get("frame_relative")

def _valid_timestamp(value):
    return isinstance(value, int) and value >= 0

def explicit_annotation_timestamp_ns(frame):
    """Return an explicit CPU realtime timestamp for annotation/extraction."""
    realtime = frame.get("cpu", {}).get("realtime", {})
    if not isinstance(realtime, dict):
        return None

    mono = realtime.get("mono_source_ns")
    if _valid_timestamp(mono):
        return mono

    left = realtime.get("left_source_ns")
    right = realtime.get("right_source_ns")
    has_left = _valid_timestamp(left)
    has_right = _valid_timestamp(right)
    if has_left and has_right:
        return left // 2 + right // 2 + ((left % 2) + (right % 2)) // 2
    if has_left:
        return left
    if has_right:
        return right

    recorder = realtime.get("recorder_reception_ns")
    if _valid_timestamp(recorder):
        return recorder
    return None


@lru_cache(maxsize=8)
def video_timeline(video_path):
    """Read encoded frames in presentation order, independently of sidecar rows."""
    try:
        def probe(entries):
            result = subprocess.run(
                ["ffprobe", "-v", "error", "-select_streams", "v:0",
                 "-show_entries", entries, "-of", "json", video_path],
                check=True, capture_output=True, text=True)
            return json.loads(result.stdout)

        # MP4 H.264/HEVC/MPEG-4 samples each hold one encoded picture. Reading
        # packet timestamps avoids decoding a long recording just to index it.
        data = probe("stream=codec_name,nb_frames,time_base,avg_frame_rate:packet=pts")
        stream = data["streams"][0]
        packets = data.get("packets", [])
        if (stream.get("codec_name") in ("h264", "hevc", "mpeg4")
                and str(len(packets)) == stream.get("nb_frames")
                and all("pts" in packet for packet in packets)):
            ticks = sorted(int(packet["pts"]) for packet in packets)
        else:
            data = probe("frame=best_effort_timestamp")
            ticks = [int(frame["best_effort_timestamp"]) for frame in data["frames"]]
        time_base = Fraction(stream["time_base"])
        fps = float(Fraction(stream["avg_frame_rate"]))
        pts = tuple(int(tick * time_base * 10**9) for tick in ticks)
        if not pts or fps <= 0 or any(a > b for a, b in zip(pts, pts[1:])):
            raise ValueError("missing or unordered frame timestamps / invalid frame rate")
        return pts, fps
    except (OSError, subprocess.CalledProcessError, ValueError, KeyError,
            IndexError, ZeroDivisionError) as exc:
        raise RuntimeError(
            f"Cannot read video presentation timestamps for {video_path}: {exc}. "
            "Ensure ffprobe (the ffmpeg package) is installed.") from exc


def video_frame_range(video_pts, start_pts, end_pts):
    """Match accurate timestamp seeks: the first frame at or after each boundary."""
    if start_pts > end_pts:
        raise ValueError("Video range ends before it starts")
    start = bisect_left(video_pts, start_pts)
    end = min(bisect_left(video_pts, end_pts), len(video_pts) - 1)
    if start >= len(video_pts) or end_pts < video_pts[0]:
        raise ValueError("Tagged timestamps are outside the encoded video")
    return range(start, end + 1)


def sidecar_video_pts(frames):
    """Translate recorder pipeline PTS to the MP4's zero-based media timeline."""
    origin = int(frames[0]["gst"]["pts_ns"])
    return [int(frame["gst"]["pts_ns"]) - origin for frame in frames]


def write_extracted_video_sidecars(data, json_path, video_path, output_dir,
                                   video_basename, video_indices, video_pts,
                                   side_by_side):
    """Write one frame record per encoded output frame, retaining source clocks."""
    observations = sorted((pts, i) for i, pts in enumerate(sidecar_video_pts(data["frames"])))
    observation_pts = [pts for pts, _ in observations]
    source_rows = []
    for video_idx in video_indices:
        pts = video_pts[video_idx]
        row = bisect_left(observation_pts, pts)
        if row == len(observations):
            row -= 1
        elif row > 0 and pts - observation_pts[row - 1] <= observation_pts[row] - pts:
            row -= 1
        source_rows.append(observations[row][1])

    outputs = [(video_basename, "unsplit")]
    if side_by_side in ("LR", "RL"):
        outputs = [(video_basename + "_left", "left"),
                   (video_basename + "_right", "right")]

    for name, eye in outputs:
        output_path = os.path.join(output_dir, name + ".mp4")
        output_pts, output_fps = video_timeline(os.path.abspath(output_path))
        if len(output_pts) != len(video_indices):
            raise RuntimeError(f"Frame count mismatch writing sidecar for {output_path}")
        result = copy.deepcopy({key: value for key, value in data.items() if key != "frames"})
        result["video_file"] = name + ".mp4"
        result["fps"] = output_fps
        result["frames_recorded"] = len(output_pts)
        result["frames_dropped"] = 0
        # A split output is a single-eye video. Both source-eye observations
        # stay in cpu.realtime, without replacing mono_source_ns or either eye.
        result["side_by_side"] = data.get("side_by_side", "undefined") if eye == "unsplit" else "undefined"
        result["extraction"] = {
            "source_video_file": os.path.relpath(video_path, output_dir),
            "source_sidecar_file": os.path.relpath(json_path, output_dir),
            "source_side_by_side": data.get("side_by_side", "undefined"),
            "source_pts_origin_ns": int(data["frames"][0]["gst"]["pts_ns"]),
            "source_timestamp_statistics": copy.deepcopy(data.get("timestamp_statistics", {})),
            "eye": eye,
            "frame_mapping": "nearest_sidecar_pts",
            "timestamp_statistics_basis": "source_timestamp_presence",
        }
        output_frames = []
        for i, (video_idx, source_row) in enumerate(zip(video_indices, source_rows)):
            frame = copy.deepcopy(data["frames"][source_row])
            frame["source"] = {
                "sidecar_index": source_row,
                "frame_index": frame["frame_index"],
                "video_frame_index": video_idx,
                "video_pts_ns": video_pts[video_idx],
                "gst": copy.deepcopy(frame["gst"]),
            }
            frame["frame_index"] = i
            # OpenCV re-encodes the clip. Read its actual PTS instead of
            # subtracting the clip start from the old GStreamer clock values.
            duration = (output_pts[i + 1] - output_pts[i] if i + 1 < len(output_pts)
                        else round(1e9 / output_fps))
            frame["gst"] = {
                "pts_ns": output_pts[i], "dts_ns": None, "duration_ns": duration,
                "running_time_ns": output_pts[i], "stream_time_ns": output_pts[i],
                "clock_time_ns": None,
            }
            output_frames.append(frame)
        result["frames"] = output_frames
        realtime = [frame["cpu"]["realtime"] for frame in output_frames]
        result["start_cpu_realtime_ns"] = realtime[0].get("recorder_reception_ns") or explicit_annotation_timestamp_ns(output_frames[0])
        result["end_cpu_realtime_ns"] = realtime[-1].get("recorder_reception_ns") or explicit_annotation_timestamp_ns(output_frames[-1])
        from_source = sum(any(_valid_timestamp(row.get(key)) for key in
                              ("mono_source_ns", "left_source_ns", "right_source_ns"))
                          for row in realtime)
        result["timestamp_statistics"] = {
            "from_unixfd": from_source, "at_reception": len(output_frames) - from_source}
        with open(os.path.join(output_dir, name + ".json"), "w") as f:
            json.dump(result, f, indent=2)
            f.write("\n")


def flatten_dict(d, parent_key='', sep='.'):
    items = []
    if isinstance(d, (dict,)) or (hasattr(d, 'items') and callable(getattr(d, 'items'))):
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            items.extend(flatten_dict(v, new_key, sep=sep).items())
    elif isinstance(d, (list, tuple, np.ndarray)):
        for i, v in enumerate(d):
            new_key = f"{parent_key}{sep}{i}"
            items.extend(flatten_dict(v, new_key, sep=sep).items())
    else:
        items.append((parent_key, d))
    return dict(items)

def natural_sort_key(s):
    """
    Key function for natural sorting (e.g., 'frame10' comes after 'frame2').
    """
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('([0-123456789]+)', str(s))]

def detect_rosbag_format(bag_path):
    if os.path.isdir(bag_path):
        mcap_files = [f for f in os.listdir(bag_path) if f.endswith('.mcap')]
        if mcap_files: return 'mcap'
        db3_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
        if db3_files: return 'sqlite3'
    
    reader = rosbag2_py.SequentialReader()
    for storage_id in ['mcap', 'sqlite3']:
        try:
            reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                        rosbag2_py.ConverterOptions('', ''))
            return storage_id
        except Exception: continue
    return None

def rosbag_to_csv(bag_path, output_dir, start_ns=None, end_ns=None):
    print(f"Converting ROS bag {bag_path} to CSV...")
    storage_id = detect_rosbag_format(bag_path)
    if not storage_id:
        print(f"Error: Unable to detect rosbag format for {bag_path}")
        return
    
    # Pass 1: Discover all fields and vector sizes
    print("Pass 1: Discovering columns and vector sizes...")
    topic_fields = {} # topic -> set of field names
    reader_p1 = rosbag2_py.SequentialReader()
    try:
        reader_p1.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                       rosbag2_py.ConverterOptions('', ''))
        if start_ns is not None:
            reader_p1.seek(start_ns)
    except Exception as e:
        print(f"Error opening rosbag for Pass 1: {e}")
        return

    topic_types = {topic.name: topic.type for topic in reader_p1.get_all_topics_and_types()}
    message_classes = {}
    
    # SequentialReader defaults to ascending received timestamps, so neither
    # pass needs to scan messages outside the requested interval.
    while reader_p1.has_next():
        (topic, data, t) = reader_p1.read_next()
        if start_ns is not None and t < start_ns: continue
        if end_ns is not None and t > end_ns: break
        
        if topic not in topic_fields:
            topic_fields[topic] = set(['timestamp'])
            
        type_name = topic_types[topic]
        if type_name not in message_classes:
            message_classes[type_name] = get_message(type_name)
        msg_type = message_classes[type_name]
        msg = deserialize_message(data, msg_type)
        msg_dict = flatten_dict(message_to_ordereddict(msg))
        topic_fields[topic].update(msg_dict.keys())
    
    del reader_p1 # Close reader for re-opening

    # Pass 2: Write data with unified header
    print("Pass 2: Writing CSV data...")
    reader_p2 = rosbag2_py.SequentialReader()
    try:
        reader_p2.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
                       rosbag2_py.ConverterOptions('', ''))
        if start_ns is not None:
            reader_p2.seek(start_ns)
    except Exception as e:
        print(f"Error opening rosbag for Pass 2: {e}")
        return
    
    writers = {}
    files = {}

    try:
        while reader_p2.has_next():
            (topic, data, t) = reader_p2.read_next()
            if start_ns is not None and t < start_ns: continue
            if end_ns is not None and t > end_ns: break
            
            if topic not in writers:
                csv_name = topic.replace('/', '_').strip('_') + ".csv"
                csv_path = os.path.join(output_dir, csv_name)
                f = open(csv_path, 'w', newline='')
                files[topic] = f
                
                f.write(f"# Topic: {topic}\n# Type: {topic_types[topic]}\n")
                
                # Natural sort field names
                fieldnames = sorted(list(topic_fields[topic]), key=natural_sort_key)
                if 'timestamp' in fieldnames:
                    fieldnames.remove('timestamp')
                    fieldnames = ['timestamp'] + fieldnames
                
                writers[topic] = csv.DictWriter(f, fieldnames=fieldnames)
                writers[topic].writeheader()
            
            msg_type = message_classes[topic_types[topic]]
            msg = deserialize_message(data, msg_type)
            row = {'timestamp': t}
            row.update(flatten_dict(message_to_ordereddict(msg)))
            writers[topic].writerow(row)
    except Exception as e:
        print(f"Error reading bag: {e}")
    finally:
        for f in files.values(): f.close()
    print("Finished CSV conversion.")

def process_video_chunk(args):
    video_path, output_dir, timestamps, frame_indices, output_format, video_basename, is_ns, is_ms, side_by_side = args
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened(): return 0

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    half_width = width // 2

    saved_count = 0
    current_pos = None
    for ts, frame_idx in zip(timestamps, frame_indices):
        if current_pos is None or frame_idx < current_pos or frame_idx - current_pos > 500:
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            current_pos = frame_idx
        while current_pos < frame_idx:
            if not cap.grab():
                break
            current_pos += 1
        if current_pos != frame_idx:
            break
        ret, frame = cap.read()
        if not ret: break
        current_pos += 1
        
        if is_ns: ts_str = f"{int(ts) / 1e9:.9f}"
        elif is_ms: ts_str = f"{float(ts) / 1000.0:.6f}"
        else: ts_str = f"{float(ts):.6f}"
        
        if side_by_side in ["", "LR", "RL"]:
            left = frame[:, :half_width]
            right = frame[:, half_width:]
            
            if side_by_side == "RL":
                left, right = right, left
                
            cv2.imwrite(os.path.join(output_dir, f"{video_basename}_left_{ts_str}.{output_format}"), left)
            cv2.imwrite(os.path.join(output_dir, f"{video_basename}_right_{ts_str}.{output_format}"), right)
        else:
            image_name = f"{video_basename}_{ts_str}.{output_format}"
            cv2.imwrite(os.path.join(output_dir, image_name), frame)
        saved_count += 1
    cap.release()
    return saved_count

def extract_video_range(video_path, output_path, start_frame, end_frame, fps, original_indices=None, side_by_side="undefined"):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return False
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # If using OpenCV loop, this fps setting is for the output
    t_fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0: fps = t_fps if t_fps > 0 else 30.0

    out = None
    out_l = None
    out_r = None

    if side_by_side in ["LR", "RL"]:
        base_path = os.path.splitext(output_path)[0]
        out_l_path = f"{base_path}_left.mp4"
        out_r_path = f"{base_path}_right.mp4"
        
        half_width = width // 2
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        print(f"Splitting video into: {out_l_path} and {out_r_path}")
        out_l = cv2.VideoWriter(out_l_path, fourcc, fps, (half_width, height))
        out_r = cv2.VideoWriter(out_r_path, fourcc, fps, (half_width, height))
        
        if not out_l.isOpened() or not out_r.isOpened():
             print("Error: Failed to open output video files.")
             cap.release()
             if out_l: out_l.release()
             if out_r: out_r.release()
             return False

    else:
        # Standard mono output using standard MP4 writer (or GStreamer equivalent)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        if not out.isOpened():
            cap.release()
            return False

    # Process frames
    success = True
    try:
        def process_frame(f):
            if out:
                out.write(f)
            else:
                # side_by_side processing
                curr_w = f.shape[1]
                hw = curr_w // 2
                left_img = f[:, :hw]
                right_img = f[:, hw:]
                
                if side_by_side == "RL":
                     out_l.write(right_img)
                     out_r.write(left_img)
                else:
                     out_l.write(left_img)
                     out_r.write(right_img)

        # Determine which frames to extract
        if original_indices is not None:
            sorted_indices = sorted(list(set(original_indices)))
        else:
            sorted_indices = list(range(start_frame, end_frame + 1))
            
        if not sorted_indices:
            print("No frames to extract.")
            return True

        # Initial seek to the first desired frame
        first_target = sorted_indices[0]
        cap.set(cv2.CAP_PROP_POS_FRAMES, first_target)
        
        # Verify seek: if we are at 0 but wanted something else, assume seek failure
        # Note: CAP_PROP_POS_FRAMES is not always accurate, but 0 usually means reset.
        current_pos = cap.get(cv2.CAP_PROP_POS_FRAMES)
        if current_pos == 0 and first_target > 0:
             print(f"Warning: Seek to frame {first_target} failed (pos={current_pos}). Fallback to manual skip.")
             cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
             current_pos = 0
             while current_pos < first_target:
                  if not cap.grab():
                       print(f"Error: Could not skip to frame {first_target}.")
                       break
                  current_pos += 1
        else:
             # Assume seek succeeded, or we are at 0 and wanted 0
             current_pos = first_target

        for target_idx in sorted_indices:
            # If we need to go forward
            if target_idx > current_pos:
                skip = target_idx - current_pos
                # If gap is small, skip. If large, try seek (unless we know seek is broken?)
                # For robustness, let's prefer skipping if it's reasonable (< 500 frames)
                # or if we suspect seek is broken.
                # Here we'll just skip to be safe and consistent.
                for _ in range(skip):
                    if not cap.grab():
                        current_pos = -1 # invalidate
                        break
                    current_pos += 1
            
            # If we somehow need to go back (shouldn't happen with sorted list)
            elif target_idx < current_pos:
                cap.set(cv2.CAP_PROP_POS_FRAMES, target_idx)
                current_pos = target_idx # Assume it works...
            
            if current_pos != target_idx:
                 success = False
                 break
                 
            ret, frame = cap.read()
            if not ret:
                print(f"Error: Could not read frame {target_idx}")
                success = False
                break
            
            process_frame(frame)
            current_pos += 1
            
    except Exception as e:
        print(f"Error during extraction: {e}")
        success = False
    finally:
        if out: out.release()
        if out_l: out_l.release()
        if out_r: out_r.release()
        cap.release()
    
    return success



def extract_session_data(json_path, output_dir, formats, num_jobs, start_acq=None, end_acq=None, latency_s=0.0, split_stereo=False):
    try:
        with open(json_path, 'r') as f: data = json.load(f)
    except Exception as e:
        print(f"Error reading {json_path}: {e}")
        return

    if data.get("type") != "dvrk_data:video_sidecar@1.0.0":
        print(f"CRITICAL: Unsupported sidecar type in {json_path}: {data.get('type')}")
        return

    video_filename = data.get("video_file")
    video_path = None
    if video_filename:
        video_path = os.path.join(os.path.dirname(json_path), video_filename)
    if not video_path or not os.path.exists(video_path):
        base = os.path.splitext(json_path)[0]
        if os.path.exists(base + ".mp4"):
            video_path = base + ".mp4"

    if not video_path: return

    frames = data.get("frames")
    if not isinstance(frames, list) or not frames:
        print(f"CRITICAL: Sidecar {json_path} is missing current frames[] data")
        return

    timestamps = [explicit_annotation_timestamp_ns(f) for f in frames]
    missing = [i for i, (f, ts) in enumerate(zip(frames, timestamps))
               if ts is None
               or not isinstance(f.get("gst"), dict)
               or f["gst"].get("pts_ns") is None]
    if missing:
        print(f"CRITICAL: Sidecar {json_path} has frames without explicit annotation time/GStreamer PTS")
        return

    is_ns = True
    is_ms = False

    indices = []
    for i, ts in enumerate(timestamps):
        if (start_acq is None or ts >= start_acq) and (end_acq is None or ts <= end_acq):
            indices.append(i)
    
    if not indices: return
    
    # Sidecar rows describe frames observed before encoding. Videorate may
    # drop/duplicate those frames, so frame_index is not an MP4 seek position.
    video_pts, fps = video_timeline(os.path.abspath(video_path))
    observation_pts = sidecar_video_pts(frames)
    if start_acq is None and end_acq is None:
        video_indices = list(range(len(video_pts)))
    else:
        selected_pts = [observation_pts[i] for i in indices]
        video_indices = list(video_frame_range(video_pts, min(selected_pts), max(selected_pts)))
    start_idx = video_indices[0]
    end_idx = video_indices[-1]

    # Preserve observed CPU timestamps for image filenames, but seek using
    # encoded PTS. Several observed frames may map to the same encoded frame.
    image_frames = {}
    if any(fmt != 'mp4' for fmt in formats):
        for i in indices:
            idx = bisect_left(video_pts, observation_pts[i])
            if idx < len(video_pts):
                image_frames.setdefault(idx, timestamps[i])
    video_basename = os.path.splitext(os.path.basename(video_path))[0]
    side_by_side = data.get("side_by_side", "undefined")
    if not split_stereo:
        side_by_side = "undefined"
    
    for fmt in formats:
        if fmt == 'mp4':
            out_all_name = f"{video_basename}.mp4"
            print(f"Extracting video segment: {out_all_name}")
            if not extract_video_range(video_path, os.path.join(output_dir, out_all_name), start_idx, end_idx, fps, video_indices, side_by_side):
                raise RuntimeError(f"Video extraction failed for {video_path}")
            write_extracted_video_sidecars(data, json_path, video_path, output_dir,
                                           video_basename, video_indices, video_pts,
                                           side_by_side)
        else:
            image_indices = sorted(image_frames)
            if not image_indices:
                raise ValueError(f"No sidecar timestamps overlap {video_path}")
            tasks = []
            chunk_size = math.ceil(len(image_indices) / num_jobs)
            for i in range(num_jobs):
                s = i * chunk_size
                e = min((i + 1) * chunk_size, len(image_indices))
                if s >= len(image_indices): break
                tasks.append((video_path, output_dir, [image_frames[idx] for idx in image_indices[s:e]], image_indices[s:e], fmt, video_basename, is_ns, is_ms, side_by_side))
            
            if num_jobs > 1:
                with multiprocessing.Pool(processes=num_jobs) as pool: pool.map(process_video_chunk, tasks)
            else:
                process_video_chunk(tasks[0])

def index_videos(index_data):
    if index_data.get("type") != "dvrk_data:index@1.0.0":
        print(f"CRITICAL: Unsupported index type: {index_data.get('type')}")
        return []

    streams = index_data.get("streams")
    if not isinstance(streams, list):
        print("CRITICAL: Current index format requires streams[]")
        return []

    videos = []
    for stream in streams:
        if not isinstance(stream, dict):
            continue
        stream_name = stream.get("name", "")
        for video in stream.get("videos", []) or []:
            if not isinstance(video, dict) or not video.get("file"):
                continue
            item = dict(video)
            if stream_name:
                item["gst_input"] = stream_name
            videos.append(item)

    return videos

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory", help="Session directory", required=True)
    parser.add_argument("-t", "--tags", help="Tags JSON file")
    parser.add_argument("-a", "--all", action="store_true", help="Extract all data")
    parser.add_argument("-f", "--format", choices=['jpg', 'png', 'mp4'], action='append', help="Output format(s).")
    parser.add_argument("-j", "--jobs", type=int)
    parser.add_argument("-S", "--split-stereo", action="store_true", help="Split stereo video streams (LR/RL) into two channels")
    args = parser.parse_args()

    if not args.tags and not args.all:
        print("Error: Either -t (tags) or -a (all) must be specified.")
        sys.exit(1)

    formats = args.format if args.format else ['mp4']
    num_jobs = args.jobs or max(1, (os.cpu_count() or 1) // 2)
    
    if not os.path.exists(args.directory):
        print(f"CRITICAL: Session directory does not exist: {args.directory}")
        sys.exit(1)

    index_path = os.path.join(args.directory, "index.json")
    if not os.path.exists(index_path):
        print(f"CRITICAL: index.json not found in {args.directory}")
        sys.exit(1)

    with open(index_path, 'r') as f: index_data = json.load(f)
    
    videos = index_videos(index_data)
    rosbag_name = index_data.get("rosbags")

    base_extracted_dir = os.path.join(args.directory, "extracted")
    os.makedirs(base_extracted_dir, exist_ok=True)

    extraction_targets = []
    if args.all:
        extraction_targets.append({"name": "full_session", "start": None, "end": None})
    if args.tags:
        tags_path = args.tags
        if not os.path.exists(tags_path):
            tags_path = os.path.join(args.directory, os.path.basename(args.tags))
            if not os.path.exists(tags_path):
                print(f"CRITICAL: Tag file not found: {args.tags} (also checked {tags_path})")
                sys.exit(1)
        with open(tags_path, 'r') as f: tags_data = json.load(f)
        
        tags_type = tags_data.get("type", "unknown")
        if tags_type not in ["dvrk_data:video_tags@1.0.0", "dvrk_data:session_tags@1.0.0"]:
             print(f"CRITICAL: Unsupported tags type: {tags_type}")
             sys.exit(1)
        
        if tags_type == "dvrk_data:session_tags@1.0.0":
             print("CRITICAL: Session tags are not yet supported for extraction.")
             sys.exit(1)

        source_v_name = tags_data.get("video_file")
        source_v_ts = []
        if source_v_name:
            for v in videos:
                if v["file"] == source_v_name:
                    v_base = os.path.splitext(v["file"])[0]
                    v_json = os.path.join(args.directory, f"{v_base}.json")
                    if os.path.exists(v_json):
                        with open(v_json, 'r') as vf:
                            v_data = json.load(vf)
                            frames = v_data.get("frames", [])
                            source_v_ts = [explicit_annotation_timestamp_ns(f)
                                           for f in frames]
                    break

        stage_counts = {}
        for stage in tags_data.get("stages", []):
            name = stage["name"]
            
            ts_s, f_s = parse_stage_timestamp(stage.get("start"))
            ts_e, f_e = parse_stage_timestamp(stage.get("end"))
            
            def resolve_acq(ts, f):
                # Use the same timestamp priority as video_tag when loading tags.
                if ts is not None:
                    return int(ts)
                # Handle types
                if f is not None:
                    try: f = int(f)
                    except: f = None
                
                if f is not None and source_v_ts:
                    if 0 <= f < len(source_v_ts):
                        final_val = source_v_ts[f]
                        if final_val is not None:
                            return int(final_val)
                
                return None

            start_val = resolve_acq(ts_s, f_s)
            end_val = resolve_acq(ts_e, f_e)
                
            stage_counts[name] = stage_counts.get(name, 0) + 1
            dir_name = f"{name}_{stage_counts[name]:03d}"
            extraction_targets.append({"name": dir_name, "start": start_val, "end": end_val})

    for target in extraction_targets:
        target_dir = os.path.join(base_extracted_dir, target["name"])
        os.makedirs(target_dir, exist_ok=True)
        print(f"Processing target: {target['name']} (start={target['start']}, end={target['end']})")
        
        for v in videos:
            v_base = os.path.splitext(v["file"])[0]
            v_json = os.path.join(args.directory, f"{v_base}.json")
            if os.path.exists(v_json):
                extract_session_data(v_json, target_dir, formats, num_jobs, target["start"], target["end"], split_stereo=args.split_stereo)
        
        if rosbag_name:
            bag_path = os.path.join(args.directory, rosbag_name)
            if os.path.exists(bag_path):
                rosbag_to_csv(bag_path, target_dir, target["start"], target["end"])

    print(f"Extraction complete. Files extracted to: {base_extracted_dir}")

if __name__ == "__main__":
    main()
