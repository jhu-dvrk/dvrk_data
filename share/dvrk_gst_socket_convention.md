# dVRK GStreamer endpoint convention

All dVRK-managed GStreamer Unix sockets use the canonical abstract name:

```text
@dvrk:<role>:<name>
```

The `@dvrk:` prefix is mandatory. It identifies a socket carrying dVRK frame
metadata and using the dVRK reconnection/data path. Bare `@` socket names are
not dVRK endpoint references.

The fixed roles are:

| Role | Inputs | Outputs |
| --- | --- | --- |
| `stereo_source` | camera pipelines | `left`, `right` |
| `stereo_alignment` | source `left`, `right` | `stereo` |
| `stereo_display` | aligned `stereo` | optional `stereo`, `overlay` |

## JSON fields

Use `gst_input` for an input. It may contain either ordinary GStreamer
pipeline text or a canonical dVRK socket reference. Use `gst_output` for an
output socket; it must always be a canonical `@dvrk:` reference.

For example, a source configuration can be written as:

```json
{
  "camera": {
    "left": {
      "gst_input": "v4l2src device=/dev/video0",
      "gst_output": "@dvrk:stereo_source:left"
    },
    "right": {
      "gst_input": "v4l2src device=/dev/video2",
      "gst_output": "@dvrk:stereo_source:right"
    }
  }
}
```

Alignment consumes those endpoints and publishes one stereo endpoint:

```json
{
  "camera": {
    "left": {"gst_input": "@dvrk:stereo_source:left"},
    "right": {"gst_input": "@dvrk:stereo_source:right"}
  },
  "gst_output": "@dvrk:stereo_alignment:stereo"
}
```

Display accepts either a pipeline or socket input:

```json
{
  "gst_input": "@dvrk:stereo_alignment:stereo",
  "eye_size": {"width": 1920, "height": 1080},
  "gst_output": "@dvrk:stereo_display:stereo"
}
```

The source and alignment executables calculate their standard output names
when `gst_output` is omitted. Display output is optional. `pip_gst_inputs`
replaces the old extra-stream setting and uses nested `gst_input` objects for
monoscopic and stereo picture-in-picture streams.

## GStreamer and discovery

The `@` is removed when constructing GStreamer properties:

```text
unixfdsink socket-path=dvrk:stereo_source:left socket-type=abstract
unixfdsrc  socket-path=dvrk:stereo_source:left socket-type=abstract
```

The shared C++ and Python helpers provide `make`, `resolve`, `build_input`,
`build_sink`, `build_src`, and socket discovery. `resolve` accepts only the
canonical `@dvrk:role:name` form; plain input pipelines are passed through
unchanged.
