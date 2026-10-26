# Stereo video workflow

`stereo_alignment` provides manual crop and horizontal/vertical shifts.
`stereo_rectify` uses a saved stereo calibration to correct distortion and
rectify epipolar geometry. Both accept one **side-by-side left/right stream**
and publish side-by-side I420 video using the existing abstract sockets.
Physical left/right camera inputs remain the responsibility of `stereo_source`.

```text
stereo_source -> stereo_alignment or stereo_rectify -> stereo_display / recording
                              ^
                 stereo_color_adjustment.json
```

## Separate geometry and color

Keep these files in the system's configuration directory:

| File | Purpose |
| --- | --- |
| `stereo_source.json` | Physical camera capture and source stereo output |
| `stereo_alignment.json` | Manual crop and image shifts |
| `stereo_rectify.json` | Rectification input/output and calibration-file selection |
| `stereo_calibration.yaml` | Calibrated camera matrices and extrinsics |
| `stereo_color_adjustment.json` | Shared per-eye brightness, contrast, saturation and hue |
| `stereo_display.json` | Display and HUD settings |

Determine geometry and color independently:

```bash
ros2 run dvrk_data stereo_alignment_calibration -c stereo_alignment.json
ros2 run dvrk_data stereo_find_color_adjustment -c stereo_color_adjustment.json
# Both calibration programs accept a named source override:
ros2 run dvrk_data stereo_find_color_adjustment -i @dvrk:stereo_source:stereo
```

Alignment calibration saves only geometry and has no color controls. The color
program loads/saves its own JSON file, provides original/corrected stereo
preview, and supports automatic matching or manual adjustments for each eye.
Neither program changes the other configuration file.

Both runtime processors discover `stereo_color_adjustment.json` next to their
processing config. An absent discovered file means neutral correction. Override
it with `--color-adjustment /path/to/file.json`; an explicitly requested missing
or invalid file is an error. Old inline `camera.left.color`/`camera.right.color`
settings are ignored with a warning. Move those values to the shared file:

```json
{
  "type": "dvrk_data:stereo_color_adjustment@1.0.0",
  "left": {"brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0},
  "right": {"brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0}
}
```

## Calibrated rectification

Generate and collect calibration using the imported applications:

```bash
ros2 run dvrk_data generate_charuco_board.py --output board.pdf
ros2 run dvrk_data charuco_stereo_calibrator -c board.json
ros2 run dvrk_data verify_stereo_calibration
ros2 run dvrk_data verify_stereo_calibration -c stereo_calibration.yaml -i @dvrk:stereo_source:stereo
```

Verification defaults to `stereo_calibration.yaml` in the current directory and
the source stereo socket. If no calibration is available it clearly labels the
preview as raw stereo. Input files and sockets always use named CLI options;
no application accepts an implicit positional input filename.

Copy `share/config/stereo_rectify.json` into the configuration directory next
to the saved calibration. The calibration path is resolved relative to that
JSON. The output eye size is exactly the calibrated eye size. Dimension
mismatches are errors; manual crop, alignment shifts and preserve-size settings
are not rectification settings. Calibrate the raw source at the production
resolution. Saved projection matrices determine zoom and valid borders.

```bash
ros2 run dvrk_data stereo_rectify -c stereo_rectify.json
ros2 run dvrk_data stereo_alignment -c stereo_alignment.json --color-adjustment stereo_color_adjustment.json
```

`-i` / `--input` overrides the single side-by-side input on either processor.
The rectifier defaults to output `@dvrk:stereo_rectify:stereo`; the alignment
node defaults to `@dvrk:stereo_alignment:stereo`. Output sockets are configurable.

## Launch selection

The full launcher in `dvrk_console` discovers the processing configuration:

```bash
ros2 launch dvrk_console stereo_video_pipeline.launch.py config_dir:=/path/to/configs
# If both geometry configs exist, an explicit choice is required:
ros2 launch dvrk_console stereo_video_pipeline.launch.py processor:=rectify config_dir:=/path/to/configs
ros2 launch dvrk_console stereo_video_pipeline.launch.py processor:=alignment config_dir:=/path/to/configs
```

With exactly one geometry config, `processor:=auto` selects it. With both, the
launcher exits before starting any nodes and explains the two choices. With
neither, it explains which configuration is needed. There is no silently
preferred processor. The launcher wires source to processor and processor to
display using their configured stereo output sockets, overriding input with
named `-i` arguments. Existing system/config-parent selection remains supported.
`color_config:=/path/to/file.json` explicitly chooses a shared color file.

For source and processing without display, use the equivalent
`ros2 launch dvrk_data stereo_processing.launch.py` arguments.

## Timing, acceleration, and validation

The processing stage preserves each eye's source and hardware-reference times.
`alignment_output_ts` and its legacy `stereo_output_ts` alias mark processing
completion for existing downstream consumers. Frames are not gated on matching
PTS; rectification does not synchronize independent camera captures. Bounded
queues retain the newest available images and source restarts reconnect.

This version uses OpenCV CPU remapping with precomputed maps. `backend` is
`cpu`; unsupported names fail explicitly. No NVIDIA SDK is required. GPU
backends remain future work and must match the reference geometry, including
rational distortion, while preserving frame provenance.

Tests cover calibration math/IO, static-target capture rejection, both eyes'
pixel parity with OpenCV, padded rows, rational distortion, caps changes,
metadata, actual socket reconnection, shared color discovery/validation, named
CLI inputs, and unambiguous launcher selection.

```bash
colcon build --packages-select dvrk_data dvrk_console
colcon test --packages-select dvrk_data dvrk_console
colcon test-result --verbose
```
