# dvrk_data

dVRK data acquisition, video transport, processing, and collection utilities.

See https://dvrk.readthedocs.io for documentation.

Stereo calibration and rectification tools are included in this package. See
[calibration tools](docs/stereo_calibration.md) and
[choosing a stereo processor](docs/stereo_rectify.md).

Simulator Unix-FD producers declare their layout using the `dvrk-video-layout`
caps field: `mono`, `stereo-LR` (left eye followed by right), or `stereo-RL`.
The per-frame timestamp metadata's `channel` field carries the same declaration.
`dvrk_data/video_layout.hpp` provides `layout_from_sample` and bounded concurrent
`probe_socket_layouts` for discovering these streams. Readers also support
standard GStreamer multiview caps and older channel-only metadata. The control
panel uses this information to select and crop the requested eye without relying
on a simulator-specific socket name.
