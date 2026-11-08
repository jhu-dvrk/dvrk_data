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
# Generate 12x12 board with 5.0mm squares and 3.5mm markers on Letter format
ros2 run dvrk_data generate_charuco_board.py
```

This generates:
1. `charuco_12x12_s5.0mm_letter.pdf` — Printable vector target with a 1mm verification ruler and print notes.
2. `charuco_12x12_s5.0mm_letter.json` — Target descriptor loaded directly by `charuco_stereo_calibrator`.

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

**Video Stream Source** and **ChArUco Board Target** start collapsed to leave more
room for capture and calibration controls. Click either heading to show its details.

- **Prominent Top Banner**: Displays high-contrast status text (`STABLE — READY`, `HOLD STILL`, `✓ CAPTURED SAMPLE #N!`, `MOVE TARGET`), sample count (`Samples: N`), and tall coverage progress bars directly above the video preview for easy readability from across the room.
- **Auto-Capture**: Captures a stationary board view only when it differs from every retained sample. Corner IDs keep changes in partial-board detection from counting as motion, and estimated pose jitter does not trigger captures. Moving either the target or the endoscope can provide a new view; keeping both still should produce one sample. Manual captures use the same duplicate check. Similar views can also be accepted when they make a real image-space change and fill density below the selected target; stationary views and corner noise remain rejected.
- **Live 5D Coverage Overlay**: The startup view draws nested depth bands with pan/tilt corner gradients directly over both live camera images. Coverage updates as samples are captured or removed; the rest of each image remains clear.
- **Rectified Epipolar Lines**: Real-time rectification view with fixed horizontal alignment guidelines. Clicking an image no longer adds a yellow line.
- **3D Anaglyph Mode**: Stereo disparity visualization with standard Red-Cyan glasses.
- **Lens Distortion**: After calibration, choose **View → Lens Distortion** for a
  static chart of both cameras. Gray lines show an ideal pixel grid; colored curves
  and arrows show where the lens maps it onto the sensor. Arrow lengths are actual
  pixel displacements. Both eyes share an automatic green/yellow/red scale from
  zero to the largest drawable displacement, shown in pixels. The view uses each
  camera's intrinsics and distortion coefficients, excluding stereo alignment and
  rectification cropping. It needs no live stream and supports wheel zoom, drag
  pan, and double-click to fit. Capture pauses while this view is selected.
  Recalibration refreshes the chart. Undefined or unrenderable model coordinates
  are skipped with a warning. This shows the fitted model, not its accuracy.
- **One-Click Outlier Filtering**: Uses independent fits, bidirectional cross projections and per-corner epipolar errors to trim isolated corner pairs or remove redundant bad samples while preserving coverage, then recalibrates. A report explains removals and protected data.

In **Stereo Calibration**, set **Rectification alpha** before clicking **Calibrate**
to control OpenCV's rectification crop. The default is `0.20`. `0.00` removes black
borders; increase alpha to retain more field of view. `1.00` retains
all source pixels and can introduce black borders. The chosen value affects the
rectified preview and the projection matrices/maps saved by that calibration.
Changing it after calibration requires clicking **Calibrate** again to apply it.
Calibration and recalibration preserve the selected view and refresh it with the
new result; they do not automatically switch to rectified stereo.

### 4. Validate a Captured Pair

After calibration, click any row in the captured samples list. The view switches to
**Selected Sample: Grid Reprojection** and freezes that left/right pair. Click another
row to inspect another pair; choose a live view from **View** to resume streaming
and capture. In the grid view, use the mouse wheel to zoom around the pointer
(up to 32 times the fitted size), hold the left mouse button and drag to pan, and
double-click to fit the image again. Switching samples preserves zoom and pan;
leaving grid validation resets the view.

Choose the board-pose source below **View**:

- **Pose from left -> right** fits the board in the left image and uses stereo
  rotation/translation to predict its appearance in the right image.
- **Independent left / right fits** estimates a separate pose in each image.
- **Pose from right -> left** fits the right image and predicts the left image.

The dim grid includes lens distortion. Detected corners and arrows from detected
points to predictions use a shared pixel-error scale: green at zero, yellow at
half scale, and red at or above **Red at / count above (px)** (default 3 px).
This control changes visualization and the reported counts; it does not reject
samples or corners. Select a **Corner #ID** to see its horizontal/vertical
residual and magnitude in each displayed eye; a white ring highlights it.

The panel always reports independent left/right reprojection RMS, both
cross-camera prediction RMS values, and rectified vertical epipolar RMS. A corner
has a residual magnitude; RMS summarizes residuals over the matching corners.
Unavailable metrics show a reason rather than zero. Diagnostics are recomputed
from the current selected sample and calibrated model. These measures describe
fit and stereo consistency; they cannot alone distinguish detection errors,
model errors, target deformation, and capture timing errors.

**Stereo optimization RMS** is the solver's overall reprojection RMS. The sample
list's **Epipolar RMS (px)** is the RMS vertical difference between rectified
matching corners. A low value does not establish correct depth or horizontal
disparity. Outlier filtering now combines per-corner epipolar errors with
independent and bidirectional reprojections and checks retained coverage.

The **ID**, **Corners**, and **Epipolar RMS (px)** headers toggle numeric sorting.
Entering validation automatically selects the first sample in the current sort
order if nothing is selected. Before calibration, the view shows raw images.
Calibration samples measure training fit; use separate images to assess unseen
views. Changing the collection requires recalibration to update the fitted model.

### Coverage views

**Live 5D Coverage Overlay** is the default view. It replaces the old spatial
coverage overlay and draws the tiling directly over both live camera images.
The stream keeps updating while samples are collected. The colored depth bands are translucent so the camera image remains visible. Detection marks
are drawn on top. Each X/Y image bucket contains four nested depth rectangles: the smallest is nearest, the largest is
farthest. Each depth band blends eight color anchors: the north, south, east
and west **edge midpoints show front-facing density** (both pan and tilt within
±15 degrees). The NW, NE, SW and SE **corners show tilted density** for left/up,
right/up, left/down and right/down pan/tilt combinations, respectively. Tilted
observations count at a corner when both angles exceed 15 degrees in those
directions. The same frontal count is repeated at all four edge midpoints;
it does not fill the tilted corners. Single-axis views still contribute to the
band's total X/Y/Z count in hover details.
Angles come from the board normal in that camera's coordinates, so in-plane roll
does not create a new pan/tilt bin.

Coverage uses a **red-to-green gradient at a constant 30% opacity**. Red means
no retained observations; green means the selected target density has been reached at that
orientation anchor. Intermediate counts interpolate between red and green, and
colors blend smoothly between frontal edge midpoints and tilted corners. Reaching the sample
target changes color rather than making the overlay disappear. There are no
outlines or separator lines between depth bands.
These counts are a collection heuristic, not a guarantee of calibration accuracy.

**Target density** in the capture controls ranges from 1 to 20 retained images
per frontal or tilted bin and defaults to 2. Changing it immediately updates the heatmap and
progress bars without adding/removing samples or
rerunning calibration. It is a collection/display setting, not a solver parameter.

The **X/Y density** bar is a quick spatial check: it reports the fraction of image
cells seen in at least the selected number of distinct retained images, separately
for left and right. It ignores depth and angles. The **5D density** bar replaces
the old angle-only bar. It reports capped observation credits toward the selected
target across all X/Y/depth cells and five orientation bins in both cameras:
one frontal bin (counted once) and four directional pan/tilt bins.
Oversampling one bin cannot compensate for an empty bin. Neither is an accuracy
score or a requirement to fill every possible bin.

A blue tint follows the current detected corner footprint and orientation in both
images. It occupies only the middle third of each depth band, leaving the density
colors visible along both edges as samples are added. It does not count as saved
coverage. Without a retained depth range it shows X/Y only; live depths outside
the range use the nearest depth band. Hover a band for its depth range, retained
image/corner counts, four tilted counts and the shared frontal count.

The live overlay supports wheel zoom (up to 32x), left-drag pan, and
double-click to fit, matching reprojection navigation. Hover coordinates follow
the current zoom and pan. Navigation persists while live images update at the
same resolution. Clicking does not open a separate coverage view. With no
stream, a waiting background keeps retained coverage available.

The **similar-to** rule now has two thresholds. A clearly different board view is
accepted as before. A smaller movement can be accepted while its X/Y/Z/pan/tilt
bins need more observations, but must exceed a tighter noise floor relative to
every retained sample. At 640-pixel image size this floor is at least about
1.3 pixels, or 1% of the board footprint when larger. Exact repeats, subpixel
jitter and already-filled similar views remain rejected; increasing density does
not cause a stationary board to be captured repeatedly.

The live renderer caches coverage gradients until samples change, reuses image
buffers, and coalesces pending frame notifications so slow GUI redraws do not
build a backlog of stale display requests. The camera/detector rate still limits
how often fresh board observations are available.

The depth range is automatic, with rounded bounds and a small margin around the
valid retained corner depths. Both cameras share that range and color scale.
Each corner uses its own camera Z coordinate, so one tilted board can contribute
to several depth buckets. Rebuilding after capture, deletion, outlier filtering,
or changes to the retained corner lists recalculates the range and every bin.
Excluded images and corners therefore leave no contribution; all remaining
samples are rebucketed when the range changes. The heatmap reflects the current
collection, while exported calibration archives still use the last calibrated
sample set.

Before calibration, pose/depth estimates use an approximate pinhole camera model;
the panel labels them approximate. After calibration they are recomputed using
the calibrated model. Missing poses contribute no
invented depth/orientation observations. A collection with no valid depth poses
shows unknown depth ranges. Coverage is collection guidance, not an accuracy
score or a request to fill every possible bin. The dashboard's spatial bars
report cells meeting the selected target density, without regard to depth/angles.

### 5. Reload Images from a Past Calibration

Click **Load Calibration Images...** and select a saved
`stereo_calibration_<date/time>` archive containing `board.json`, `left/`, and
`right/`. The tool loads matching `sample_<ID>.png` pairs in the background,
restores their IDs and board settings, and re-detects ChArUco corners. A successful
load replaces the current collection and resets its calibration; a failed load
leaves the collection intact. Missing pairs, inconsistent sizes, unreadable
images, or insufficient common corners are reported as errors.

The first imported sample is displayed automatically. Auto-capture is turned off
to keep the imported collection stable. Set the desired alpha and click **Run
Stereo Calibration** to compute new results; the old archive is not modified.

---

## Standalone Rectification Verification

Verify an existing calibration file anytime on live or recorded streams:

```bash
ros2 run dvrk_data verify_stereo_calibration -c /path/to/stereo_calibration.yaml
```

---

## Output Files

After a successful calibration, click **Save Calibration & Images...** in
**View Mode & Export** and choose an output folder. The tool writes:

- `left.yaml` and `right.yaml`: Standard ROS 2 CameraInfo configuration (`camera_matrix`, `distortion_coefficients`, `rectification_matrix`, `projection_matrix`).
- `stereo_calibration.yaml`: Consolidated dVRK configuration with extrinsics ($R, T, E, F$), disparity-to-depth matrix ($Q$), baseline in millimeters, and RMS error metrics.

The sidebar's **Include raw calibration images** checkbox is enabled by default.
Leave it checked to also create
`stereo_calibration_YYYY-MM-DD_HH-MM-SS/` inside the selected output folder, using
local time. Repeated saves get a numeric suffix when needed. The directory contains:

- Copies of `left.yaml`, `right.yaml`, and `stereo_calibration.yaml`.
- `board.json` with the board settings.
- `left/sample_000001.png` and `right/sample_000001.png`, etc., with matching sample
  IDs from the list. PNG files preserve the original captured pixels without
  annotations or rectification.

The archive uses the exact sample set from the last successful calibration, even
if you subsequently add or delete samples. Recalibrate to archive a changed set.
Uncheck **Include raw calibration images** to export only the YAML files.

All three YAML files include the same `calibration_date` timestamp in UTC ISO 8601
format (for example, `2026-10-26T18:30:00Z`). This records when calibration
succeeded and remains unchanged when saving again or loading and re-exporting.
`calibration_date` is required when loading or exporting calibration YAML.

### Coverage-preserving outlier filtering

**Filter Outliers & Recalibrate** performs one conservative pass using the current
calibration. It computes independent left/right fits with robust pose estimation,
both cross-camera projections, and rectified vertical error for each corner.
Robust pose fitting prevents a few bad detections from pulling the entire board
fit away from its good corners. All detected corners are still evaluated.

Cutoffs use median + 4.5 × scaled median absolute deviation, with minimum cutoffs
of 1.5 px for independent/epipolar errors and 2.5 px for cross projections.
An isolated corner must exceed both its collection and within-sample independent
fit cutoffs, with supporting cross-camera or epipolar disagreement. It is removed
as a **paired correspondence** from both eyes; the good eye is not labeled bad,
but the stereo solver requires matching arrays. Raw image pixels stay intact.
No sample loses more than a quarter of its corners, at least six remain, and
the retained object points must span the board plane.

Before any removal, the median sample errors must be within 1.5 px for each
independent fit and epipolar error, and 2.5 px for each transfer direction. If
this consistency gate fails, the entire collection is retained for model review.

A whole sample is considered only when its median errors in **both** transfer
directions and epipolar error are anomalous relative to other samples. Independent
RMS alone never removes a sample. Broad disagreement across the collection is
reported for review as possible model/timing/detection trouble instead of triggering
mass deletion. At least five fully diagnosed samples remain. Missing diagnostics are retained
for review; these checks cannot prove whether the model or detections caused an error.

Every proposed removal checks both cameras' X/Y coverage and all nine recorded
pan/tilt combinations in the X/Y/Z density map. Depth range and independently fitted
poses are frozen for the pass so removing an extreme sample cannot hide coverage
loss by rescaling the buckets. Counts represent distinct images per bin, not raw
corners. A removal must retain the selected density target in every affected bin;
where coverage is below target, it must preserve every existing observation.
Checks accumulate across removals. Thus a clearly bad but unique view may remain
protected: collect a replacement at that pose before filtering again. Protection
is a coverage warning, not an endorsement of that observation's quality.

The report below the button lists removed/protected samples and paired corners,
and samples needing review. Hover it for sample/corner IDs and cutoffs. Successful
filtering refreshes the live map and corner counts and recalibrates in the current
view. If recalibration fails, the pre-filter collection is restored. The coverage
map refits its depth range and poses after calibration, so its displayed buckets
can still move even though the frozen-bin checks passed. This first strategy uses
the fitted model and is heuristic, not independent validation or leave-one-out testing.

New raw-image archives include `retained_corners.json` containing retained corner
IDs per sample. Reload redetects the original images and applies those masks;
missing/invalid retained IDs fail the load without replacing the collection.
Archives without the manifest retain the original redetection behavior.
