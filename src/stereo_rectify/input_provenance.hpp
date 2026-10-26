// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#pragma once

struct EyeInputState {
  bool left;
  gint64 last_source_ts = 0;
};

GstPadProbeReturn input_timestamp_probe_cb(GstPad *, GstPadProbeInfo *info,
                                          gpointer user_data) {
  auto &state = *static_cast<EyeInputState *>(user_data);
  if (info->type & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM) {
    if (GST_EVENT_TYPE(GST_PAD_PROBE_INFO_EVENT(info)) == GST_EVENT_STREAM_START)
      state.last_source_ts = 0;
    return GST_PAD_PROBE_OK;
  }
  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) return GST_PAD_PROBE_OK;
  GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
  auto timestamps = dc_buffer_get_frame_timestamps(buffer);
  auto &source_ts = state.left ? timestamps.left_source_ts : timestamps.right_source_ts;
  // The source emits when either eye changes. Avoid reprocessing this eye when
  // only its peer changed; this is deduplication, never timestamp matching.
  if (source_ts > 0 && source_ts == state.last_source_ts) return GST_PAD_PROBE_DROP;
  if (source_ts == 0) source_ts = dc_clock_realtime_ns();
  state.last_source_ts = source_ts;
  auto &references = state.left ? timestamps.left_reference_timestamps : timestamps.right_reference_timestamps;
  if (references.empty()) references = dc_buffer_get_reference_timestamps(buffer);
  buffer = gst_buffer_make_writable(buffer);
  GST_PAD_PROBE_INFO_DATA(info) = buffer;
  dc_buffer_set_frame_timestamps(buffer, timestamps);
  return GST_PAD_PROBE_OK;
}

void add_probe(GstElement *pipeline, const std::string &element_name,
               GstPadProbeCallback callback, gpointer user_data) {
  GstElement *element =
      gst_bin_get_by_name(GST_BIN(pipeline), element_name.c_str());
  if (element == nullptr) {
    return;
  }

  GstPad *pad = gst_element_get_static_pad(element, "src");
  if (pad != nullptr) {
    gst_pad_add_probe(pad, static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM),
                      callback, user_data, nullptr);
    gst_object_unref(pad);
  }
  gst_object_unref(element);
}

