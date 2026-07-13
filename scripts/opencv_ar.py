#!/usr/bin/env python3

import sys
import os
import time
import math
import argparse

try:
    import cv2
    import numpy as np
except ImportError:
    print("Error: opencv-python and numpy are required. Install them with: pip install opencv-python numpy or sudo apt install python3-opencv python3-numpy")
    sys.exit(1)

import dvrk_gst_socket as dvrk_gst

def main():
    left_socket  = dvrk_gst.make(dvrk_gst.ROLE_STEREO_SOURCE, "left_ar")
    right_socket = dvrk_gst.make(dvrk_gst.ROLE_STEREO_SOURCE, "right_ar")

    parser = argparse.ArgumentParser(description="OpenCV-based 3D AR Overlay source for dVRK console")
    parser.add_argument("-w", "--width", type=int, default=640, help="Width of frame (default: 640)")
    parser.add_argument("-H", "--height", type=int, default=480, help="Height of frame (default: 480)")

    args, _ = parser.parse_known_args()
    width = args.width
    height = args.height
    fps = 30

    # Abstract sockets are cleaned up by the kernel — no filesystem removal needed.

    # GStreamer pipelines for cv2.VideoWriter (BGR 3-channels)
    # We specify format=BGR on appsrc, then convert to BGRx before unixfdsink.
    # This forces videoconvert to allocate FD-backed buffers from the unixfdsink pool.
    gst_pipeline_l = (
        f"appsrc format=time ! video/x-raw,format=BGR ! videoconvert ! video/x-raw,format=BGRx ! "
        f"queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
        f"{dvrk_gst.build_sink(left_socket)}"
    )
    gst_pipeline_r = (
        f"appsrc format=time ! video/x-raw,format=BGR ! videoconvert ! video/x-raw,format=BGRx ! "
        f"queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! "
        f"{dvrk_gst.build_sink(right_socket)}"
    )

    print("Initializing OpenCV GStreamer VideoWriters...")
    writer_l = cv2.VideoWriter(gst_pipeline_l, cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
    writer_r = cv2.VideoWriter(gst_pipeline_r, cv2.CAP_GSTREAMER, 0, fps, (width, height), True)

    if not writer_l.isOpened() or not writer_r.isOpened():
        print("Error: Failed to open one or both GStreamer VideoWriters.")
        print("Make sure OpenCV is compiled with GStreamer support.")
        sys.exit(1)

    print("Streaming simple OpenCV BGR AR feeds...")
    print(f"AR Left socket: {left_socket}")
    print(f"AR Right socket: {right_socket}")


    try:
        while True:
            t = time.time()
            
            # Center of the screen
            cx = width / 2.0
            cy = height / 2.0
            
            # Compute Lissajous curve positions
            x = cx + (width * 0.3) * math.sin(t * 1.5)
            y = cy + (height * 0.2) * math.sin(t * 3.0)
            
            # Stereo disparity (depth animation)
            disparity = 8.0 * math.sin(t * 0.5)
            
            # Create blue background frames (BGR 3-channels, Blue is (255, 0, 0))
            frame_l = np.full((height, width, 3), (255, 0, 0), dtype=np.uint8)
            frame_r = np.full((height, width, 3), (255, 0, 0), dtype=np.uint8)
            
            xl = int(x + disparity)
            xr = int(x - disparity)
            yi = int(y)
            
            size = 20
            color = (0, 255, 0) # Green (B, G, R)
            thickness = 2
            
            # Draw on Left frame
            cv2.line(frame_l, (xl - size, yi), (xl + size, yi), color, thickness)
            cv2.line(frame_l, (xl, yi - size), (xl, yi + size), color, thickness)
            
            # Draw on Right frame
            cv2.line(frame_r, (xr - size, yi), (xr + size, yi), color, thickness)
            cv2.line(frame_r, (xr, yi - size), (xr, yi + size), color, thickness)
            
            # Write frames to the GStreamer pipeline
            writer_l.write(frame_l)
            writer_r.write(frame_r)
            
            # Sleep to match frame rate
            time.sleep(1.0 / fps)
            

            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        writer_l.release()
        writer_r.release()
        # Abstract sockets are cleaned up by the kernel — no filesystem removal needed.

if __name__ == '__main__':
    main()
