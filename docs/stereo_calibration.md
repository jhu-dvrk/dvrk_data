# Stereo calibration tools

Stereo camera calibration tools designed specifically for small-baseline stereo endoscopes (such as da Vinci surgical endoscopes).

This package provides a modern, fast, and robust alternative to standard chessboard-based calibration packages:
- **ChArUco Boards**: Interpolates interior saddle points even when corners or edges are occluded or cut off by the endoscope FOV.
- **`dvrk_data` GStreamer Socket Ingestion**: Directly connects to `@dvrk:stereo_source:stereo` (combined side-by-side Unix abstract socket), eliminating DDS serialization latency and guaranteeing zero-jitter left/right synchronization.
- **Native GTKmm 3.0 GUI**: Real-time display, motion-stabilized auto-capture, spatial & angular coverage heatmaps, epipolar line validation, and Red-Cyan 3D anaglyph preview.
- **Export Standards**: Exports ROS 2 standard CameraInfo YAML files (`left.yaml`, `right.yaml`) and consolidated dVRK configuration files.

---

## Target Board Generation

Generate millimeter-accurate vector PDF, SVG, or high-DPI PNG target boards alongside a matching JSON configuration file:

```bash
# Generate 8x6 board with 5.0mm squares and 3.5mm markers
ros2 run dvrk_data generate_charuco_board.py \
  --squares-x 8 --squares-y 6 \
  --square-length 5.0 --marker-length 3.5 \
  --dict DICT_4X4_50 \
  --output endoscope_charuco_8x6.pdf
```

This generates:
1. `endoscope_charuco_8x6.pdf` — Printable vector target with a 1mm verification ruler and print notes.
2. `endoscope_charuco_8x6.json` — Target descriptor loaded directly by `charuco_stereo_calibrator`.

> [!IMPORTANT]
> Print the PDF at **100% scale** (disable "Fit to Printable Area"). Verify the printed dimensions using the included verification ruler and calipers before calibration. Mount the target onto a rigid, matte surface.

---

## Running the Calibrator

### 1. Launch with Board Config JSON
Point the calibrator to your board's JSON file:

```bash
ros2 run dvrk_data charuco_stereo_calibrator -c endoscope_charuco_8x6.json
```

**Options:**
- `-c, --config <file.json>`: Path to ChArUco board configuration JSON (required).
- `-i, --input <socket/pipe>`: Stereo side-by-side input (default: `@dvrk:stereo_source:stereo`).
- `-t, --testsrc`: Fallback to synthetic GStreamer test source (offline testing mode).
- `--list-sockets`: Pretty-print all active `@dvrk:*` Unix abstract sockets currently registered in the kernel.

### 2. Stream Auto-Connection & Reconnection
The calibrator monitors `/proc/net/unix` and GStreamer bus events continuously:
- **Order-Independent**: You can start `charuco_stereo_calibrator` before or after `stereo_source`. If started before, it waits with clear status guidance and connects immediately once the stream appears.
- **Auto-Reconnect**: If `stereo_source` restarts or temporarily drops, the calibrator cleanly recycles the pipeline and reconnects without having to restart the application.

### 3. Interactive Guidance & Top Dashboard
- **Prominent Top Banner**: Displays high-contrast status text (`STABLE — READY`, `HOLD STILL`, `✓ CAPTURED SAMPLE #N!`, `MOVE TARGET`), sample count (`Samples: N`), and tall coverage progress bars directly above the video preview for easy readability from across the room.
- **Auto-Capture**: Captures a stationary board view only when it differs from every retained sample. Corner IDs keep changes in partial-board detection from counting as motion, and estimated pose jitter does not trigger captures. Moving either the target or the endoscope can provide a new view; keeping both still should produce one sample. Manual captures use the same duplicate check.
- **Coverage Heatmap**: Visualizes spatial corner density across the image sensor to ensure adequate calibration at the optical peripheries.
- **Rectified Epipolar Lines**: Real-time rectification view with clickable horizontal alignment guidelines.
- **3D Anaglyph Mode**: Stereo disparity visualization with standard Red-Cyan glasses.
- **One-Click Outlier Filtering**: Automatically identifies frames whose reprojection error exceeds 1.5x the median error, purges them from the collection, rebuilds coverage, and recalibrates in one click.

### 4. Validate a Captured Pair

After calibration, click any row in the captured samples list. The view switches to
**Selected Sample: Grid Reprojection** and freezes that left/right pair. Click another
row to inspect another pair; choose a live view from **View** to resume streaming
and capture. In the grid view, use the mouse wheel to zoom around the pointer
(up to 32 times the fitted size), hold the left mouse button and drag to pan, and
double-click to fit the image again. Switching samples preserves zoom and pan;
leaving grid validation resets the view.

The green grid is projected onto the original images with lens distortion applied.
Yellow circles mark detected common ChArUco corners, and red segments show their
residuals. Left and right RMS residuals are displayed in pixels. The board pose is
estimated from the selected sample's left corners with the calibrated intrinsics;
the right projection uses that pose and the calibrated stereo rotation/translation.
This makes right-eye discrepancies useful for inspecting stereo geometry. These
samples were used to fit the calibration, so their residuals measure fit, rather
than independent accuracy on unseen images. Changing the frame collection requires
recalibration to update the calibration fit.

---

## Standalone Rectification Verification

Verify an existing calibration file anytime on live or recorded streams:

```bash
ros2 run dvrk_data verify_stereo_calibration -c /path/to/stereo_calibration.yaml
```

---

## Output Files

When you click **Save Calibration...**, the tool writes:
- `left.yaml` and `right.yaml`: Standard ROS 2 CameraInfo configuration (`camera_matrix`, `distortion_coefficients`, `rectification_matrix`, `projection_matrix`).
- `stereo_calibration.yaml`: Consolidated dVRK configuration with extrinsics ($R, T, E, F$), disparity-to-depth matrix ($Q$), baseline in millimeters, and RMS error metrics.

The save dialog includes **Also save raw calibration images in a dated directory**
(unchecked by default). Enabling it also creates
`stereo_calibration_YYYY-MM-DD_HH-MM-SS/` inside the selected output folder, using
local time. Repeated saves get a numeric suffix when needed. The directory contains:

- Copies of `left.yaml`, `right.yaml`, and `stereo_calibration.yaml`.
- `board.json` with the board settings.
- `left/sample_000001.png` and `right/sample_000001.png`, etc., with matching sample
  IDs from the list. PNG files preserve the original captured pixels without
  annotations or rectification.

The archive uses the exact sample set from the last successful calibration, even
if you subsequently add or delete samples. Recalibrate to archive a changed set.
