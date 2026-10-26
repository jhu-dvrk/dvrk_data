# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Johns Hopkins University, LCSR
"""Exercise the node through real sockets without camera hardware."""

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import cv2
import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst

EYE_WIDTH, HEIGHT = 320, 240
WIDTH = 2 * EYE_WIDTH


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


def write_calibration(path):
    camera = np.array([[200.0, 0.0, 160.0], [0.0, 200.0, 120.0], [0.0, 0.0, 1.0]])
    projection = np.column_stack((camera, np.zeros(3)))
    right_projection = projection.copy()
    right_projection[0, 3] = -1200.0
    fields = {
        "type": "dvrk_calibration:stereo@1.0.0",
        "distortion_model": "plumb_bob",
        "image_width": EYE_WIDTH, "image_height": HEIGHT,
        "K1": camera, "K2": camera,
        "D1": np.zeros((1, 5)), "D2": np.zeros((1, 5)),
        "R": np.eye(3), "T": np.array([[-6.0], [0.0], [0.0]]),
        "R1": np.eye(3), "R2": np.eye(3),
        "P1": projection, "P2": right_projection, "Q": np.eye(4),
    }
    output = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    for key, value in fields.items():
        output.write(key, value)
    output.release()


def source(socket, width=WIDTH, pattern="smpte"):
    pipeline = Gst.parse_launch(
        f"videotestsrc is-live=true pattern={pattern} ! "
        f"video/x-raw,format=RGBA,width={width},height={HEIGHT},framerate=30/1 ! "
        f"unixfdsink socket-path={socket} socket-type=abstract sync=false async=false")
    check(pipeline.set_state(Gst.State.PLAYING) != Gst.StateChangeReturn.FAILURE,
          "Cannot start synthetic producer")
    return pipeline


def receive(node, socket, log_path):
    consumer = None
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline:
            check(node.poll() is None, "Processor exited: " + log_path.read_text())
            if consumer is None:
                if "@" + socket not in Path("/proc/net/unix").read_text():
                    time.sleep(0.05)
                    continue
                consumer = Gst.parse_launch(
                    f"unixfdsrc socket-path={socket} socket-type=abstract ! "
                    "appsink name=out sync=false max-buffers=1 drop=true wait-on-eos=false")
                consumer.set_state(Gst.State.PLAYING)
            sink = consumer.get_by_name("out")
            sample = sink.emit("try-pull-sample", 100 * Gst.MSECOND)
            if sample:
                caps = sample.get_caps().get_structure(0)
                check(caps.get_value("width") == WIDTH and
                      caps.get_value("height") == HEIGHT and caps.get_value("format") == "I420",
                      "Incompatible output caps: " + caps.to_string())
                metadata = sample.get_buffer().get_custom_meta("DataCollectionFrameTimestamps")
                check(metadata, "No provenance metadata after unixfd transport")
                timing = metadata.get_structure()
                left, right = timing.get_value("left-source-ts"), timing.get_value("right-source-ts")
                if left and right:
                    check(timing.get_value("alignment-output-ts") > 0, "No processing-stage timestamp")
                    return left, right
            message = consumer.get_bus().pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message or sink.get_property("eos"):
                consumer.set_state(Gst.State.NULL)
                consumer = None
                time.sleep(0.05)
        raise RuntimeError("No complete stereo output: " + log_path.read_text())
    finally:
        if consumer:
            consumer.set_state(Gst.State.NULL)


def run(executable):
    Gst.init(None)
    Gst.meta_register_custom("DataCollectionFrameTimestamps", [])
    work = Path(tempfile.mkdtemp(prefix="dvrk-rectify-smoke-"))
    input_socket = f"dvrk:rectify_smoke:input_{os.getpid()}"
    output_socket = f"dvrk:rectify_smoke:output_{os.getpid()}"
    calibration = work / "calibration.yaml"
    write_calibration(calibration)
    config = work / "rectify.json"
    config.write_text(json.dumps({
        "type": "dvrk_data:stereo_rectify@1.0.0",
        "calibration_file": str(calibration),
        "gst_input": "@" + input_socket, "gst_output": "@" + output_socket,
    }))
    log_path = work / "node.log"
    environment = os.environ.copy()
    environment.update(ROS_LOG_DIR=str(work / "ros"), ROS_DOMAIN_ID="172",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    producer, node = None, None
    passed = False
    with log_path.open("w") as log:
        def start_node():
            return subprocess.Popen([executable, "-c", str(config)], stdout=log,
                                    stderr=subprocess.STDOUT, env=environment)
        try:
            # Start processing before its producer to exercise connection retry.
            node = start_node()
            time.sleep(0.2)
            producer = source(input_socket)
            first = receive(node, output_socket, log_path)
            print("Socket processing and provenance passed:", first, flush=True)
            producer.set_state(Gst.State.NULL)
            producer = None
            time.sleep(1.2)
            producer = source(input_socket, pattern="ball")
            second = receive(node, output_socket, log_path)
            check(second[0] > first[0] and second[1] > first[1],
                  "Stale frames survived reconnection")
            print("Producer restart/reconnection passed:", second, flush=True)
            producer.set_state(Gst.State.NULL)
            producer = None
            time.sleep(0.2)
            producer = source(input_socket, width=WIDTH - 20)
            check(node.wait(timeout=12) == 1, "Runtime dimension mismatch must terminate the node")
            check(f"configured for {WIDTH}x{HEIGHT}" in log_path.read_text(), log_path.read_text())
            # Caps may also fail before the GLib loop has started running.
            node = start_node()
            check(node.wait(timeout=8) == 1, "Initial dimension mismatch must terminate the node")
            print("Initial and runtime dimension mismatches rejected", flush=True)
            passed = True
        finally:
            if producer:
                producer.set_state(Gst.State.NULL)
            if node and node.poll() is None:
                node.send_signal(signal.SIGINT)
                try:
                    node.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    node.kill()
                    node.wait()
    if passed:
        shutil.rmtree(work)
    else:
        print("Socket test logs:", work, flush=True)


if __name__ == "__main__":
    run(sys.argv[1])
