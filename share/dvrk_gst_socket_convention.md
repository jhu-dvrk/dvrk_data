# dvrk_gst Socket Naming Convention

This document describes the abstract Unix socket naming convention used for
inter-process video streaming in the dVRK software stack.

## Overview

Video frames are exchanged between dVRK nodes through GStreamer
[`unixfdsink`](https://gstreamer.freedesktop.org/documentation/unixfd/unixfdsink.html) /
[`unixfdsrc`](https://gstreamer.freedesktop.org/documentation/unixfd/unixfdsrc.html)
elements.  Every socket lives in the **Linux abstract namespace** — sockets have
no entry in `/tmp` and are reclaimed automatically by the kernel when all
file descriptors referencing them are closed.

## Naming pattern

```
@dvrk_gst:<role>:<name>
```

| Field  | Description                                                |
|--------|------------------------------------------------------------|
| `@`    | Signals an abstract-namespace socket (our convention).     |
| `role` | Fixed per application type (see table below).              |
| `name` | Short identifier unique within a role (e.g. `left`, `stereo`). |

### Fixed roles

| Role                | Produced by                        | Typical names          |
|---------------------|------------------------------------|------------------------|
| `stereo_source`     | `stereo_source`, AR scripts        | `left`, `right`, `left_ar`, `right_ar` |
| `stereo_alignment`  | `stereo_alignment`                 | `stereo`               |
| `stereo_display`    | `stereo_display` (dvrk_console)    | `stereo`, `overlay`    |

### Multi-instance naming

When two instances of the same role run simultaneously (e.g. two camera inputs),
embed a discriminator in the short name:

```
@dvrk_gst:stereo_source:hdmi_left
@dvrk_gst:stereo_source:sdi_left
@dvrk_gst:stereo_alignment:hdmi_stereo
```

The corresponding JSON `unixfdsources` entries must reference the discriminated
names:

```json
{"socket": "stereo_source:hdmi_left"}
```

## GStreamer pipeline fragments

The `@` is stripped when passing the name to GStreamer's `socket-path=` property.
Both elements require `socket-type=abstract`:

```
unixfdsink socket-path=dvrk_gst:stereo_source:left  socket-type=abstract sync=false async=false
unixfdsrc  socket-path=dvrk_gst:stereo_alignment:stereo  socket-type=abstract do-timestamp=true
```

The C++ utility (`dvrk_gst::build_sink()`, `dvrk_gst::build_src()`) and the
Python utility (`dvrk_gst.build_sink()`, `dvrk_gst.build_src()`) generate these
strings from a fully-qualified name.

## JSON configuration

All JSON configuration files use a single `"socket"` field.  Three forms are
accepted by the resolution functions:

| Form                               | Expands to                                    |
|------------------------------------|-----------------------------------------------|
| `"left"`                           | `@dvrk_gst:<default_role>:left`               |
| `"stereo_alignment:stereo"`        | `@dvrk_gst:stereo_alignment:stereo`           |
| `"@dvrk_gst:stereo_alignment:stereo"` | used as-is                               |

### stereo_source

```json
{
  "type": "dvrk_data:stereo_source@1.0.0",
  "unixfdsinks": [
    {"socket": "left"},
    {"socket": "right"}
  ]
}
```
Default role for `unixfdsinks`: `stereo_source`.

### stereo_alignment

```json
{
  "type": "dvrk_data:stereo_alignment@1.0.0",
  "unixfdsources": [
    {"socket": "left"},
    {"socket": "right"}
  ],
  "unixfdsinks": [
    {"socket": "stereo"}
  ]
}
```

Default role for `unixfdsources`: `stereo_source` (alignment reads from the
source app).  Default role for `unixfdsinks`: `stereo_alignment`.

### stereo_display

```json
{
  "type": "dvrk_console:stereo_display@1.0.0",
  "unixfdsinks": [
    {"socket": "stereo"},
    {"socket": "overlay"}
  ],
  "stereo": {
    "socket": "stereo_alignment:stereo"
  }
}
```

The `stereo.socket` field accepts any of the three forms above.  When omitted,
fall back to `stereo.stream` (raw GStreamer pipeline string).

Default role for `unixfdsinks`: `stereo_display`.

### record

```json
{
  "type": "dvrk_data:record@1.0.0",
  "videos": [
    {
      "name": "stereo_recording",
      "socket": "stereo_alignment:stereo",
      "side_by_side": "LR"
    }
  ]
}
```

When `"socket"` is present, `"stream"` is ignored.  Use the full
`"role:name"` form to unambiguously identify the producer.

## Discovery

### Command-line (ss)

```bash
ss -x | grep dvrk_gst
```

Example output:
```
u_str LISTEN 0 128 @dvrk_gst:stereo_source:left  0
u_str LISTEN 0 128 @dvrk_gst:stereo_source:right 0
u_str LISTEN 0 128 @dvrk_gst:stereo_alignment:stereo 0
```

### C++ API

```cpp
#include <dvrk_data/dvrk_gst_socket.hpp>

// List all active @dvrk_gst sockets:
dvrk_gst::print_sockets();

// Programmatic access:
for (const auto &name : dvrk_gst::list_sockets()) {
    std::cout << name << "\n";
}

// Check one socket, print alternatives on failure:
if (!dvrk_gst::check_socket("@dvrk_gst:stereo_alignment:stereo")) {
    // error already printed to stderr
}
```

### Python API

```python
import dvrk_data.dvrk_gst_socket as dvrk_gst

# List and print:
dvrk_gst.print_sockets()

# Programmatic access:
for name in dvrk_gst.list_sockets():
    print(name)

# Check one socket:
if not dvrk_gst.check_socket("@dvrk_gst:stereo_alignment:stereo"):
    pass  # error already printed to stderr
```

## Shared utility locations

| Language | File                                                          |
|----------|---------------------------------------------------------------|
| C++      | `dvrk_data/include/dvrk_data/dvrk_gst_socket.hpp`            |
| Python   | `dvrk_data/dvrk_data/dvrk_gst_socket.py`                     |
