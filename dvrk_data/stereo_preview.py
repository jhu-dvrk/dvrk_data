# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Johns Hopkins University, LCSR
"""Latest-frame readers and side-by-side preview widgets for calibration tools."""
import threading
import time
import cv2
import gi
gi.require_version("Gtk", "3.0")
gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf, GLib, Gtk

def as_preview_pipeline(source_snippet: str) -> str:
    snippet = source_snippet.strip()
    if "appsink" in snippet:
        return snippet
    return f"{snippet} ! videoconvert ! appsink drop=true sync=false"


def open_capture(source_label: str, source_snippet: str, timeout=10.0):
    pipeline = as_preview_pipeline(source_snippet)
    deadline = time.monotonic() + timeout
    while True:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            return cap
        cap.release()
        if time.monotonic() >= deadline:
            break
        time.sleep(0.25)
    msg = (f"Unable to open {source_label} source with GStreamer backend.\n"
           f"Pipeline: {pipeline}\n")
    if "unixfdsrc" in pipeline:
        import re
        match = re.search(r"socket-path=(\S+)", pipeline)
        socket_path = ("@" + match.group(1)) if match else "<socket>"
        msg += (
            f"\nThe abstract socket {socket_path} could not be opened.\n"
            "Start source and calibration together from the configuration directory:\n"
            "  ros2 launch dvrk_data stereo_alignment_calibration.launch.py\n"
            "\nTo list currently active @dvrk sockets:\n"
            "  ros2 run dvrk_data gscam_socket"
        )
    raise RuntimeError(msg)


class FrameReader:
    """Keep socket reads off the UI thread and retain only the newest frame."""

    def __init__(self, capture, first_frame):
        self.capture = capture
        self.frame = first_frame
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if ok and frame is not None:
                with self.lock:
                    self.frame = frame
            else:
                self.stop_event.wait(0.05)

    def latest(self):
        with self.lock:
            return self.frame

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=0.5)
        if not self.thread.is_alive():
            self.capture.release()


class VideoWindow(Gtk.Window):
    """A movable preview window that can be hidden and restored."""

    def __init__(self, title, width=720, height=550):
        super().__init__(title=title)
        self.set_default_size(width, height)
        self.image = Gtk.Image()
        self.add(self.image)
        self.connect("delete-event", self._hide_on_close)

    def _hide_on_close(self, _window, _event):
        self.hide()
        return True

    def set_frame(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        data = GLib.Bytes.new(rgb.tobytes())
        pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(
            data, GdkPixbuf.Colorspace.RGB, False, 8, width, height, width * 3)
        allocated = self.image.get_allocation()
        default_width, default_height = self.get_default_size()
        available_width = allocated.width if allocated.width > 1 else default_width - 16
        available_height = allocated.height if allocated.height > 1 else default_height - 70
        scale = min(available_width / width, available_height / height)
        pixbuf = pixbuf.scale_simple(max(1, round(width * scale)),
                                     max(1, round(height * scale)),
                                     GdkPixbuf.InterpType.BILINEAR)
        self.image.set_from_pixbuf(pixbuf)


