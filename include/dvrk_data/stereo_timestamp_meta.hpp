#ifndef DVRK_DATA_STEREO_TIMESTAMP_META_HPP
#define DVRK_DATA_STEREO_TIMESTAMP_META_HPP

#include <gst/base/gstaggregator.h>
#include <dvrk_data/cpu_timestamp_meta.hpp>

// The samples-selected callback and source-pad probe run synchronously in the
// aggregator's output thread. Attach before PLAYING, directly to the compositor
// (not after a queue), and keep this object alive until the pipeline is NULL.
enum class DcStereoTimestampStage { Source, Alignment, Preserve };

struct DcStereoTimestampState {
    DcStereoTimestampStage stage = DcStereoTimestampStage::Alignment;
    bool camera_only = false; // AR mixers: sink_0 is the camera, sink_1 is artwork.
    GstClockTime pts = GST_CLOCK_TIME_NONE;
    DcFrameTimestamps selected;
};

inline void dc_stereo_samples_selected(GstAggregator *aggregator, GstSegment *,
                                      guint64 pts, guint64, guint64,
                                      GstStructure *, gpointer user_data) {
    auto &state = *static_cast<DcStereoTimestampState *>(user_data);
    state.pts = pts;
    state.selected = {};
    int selected_count = 0;
    for (int side = 0; side < (state.camera_only ? 1 : 2); ++side) {
        GstPad *pad = gst_element_get_static_pad(GST_ELEMENT(aggregator), side == 0 ? "sink_0" : "sink_1");
        if (!pad) continue;
        GstSample *sample = gst_aggregator_peek_next_sample(aggregator, GST_AGGREGATOR_PAD(pad));
        gst_object_unref(pad);
        if (!sample) continue;
        ++selected_count;
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        const auto timestamps = dc_buffer_get_frame_timestamps(buffer);
        if (state.camera_only) {
            state.selected = timestamps;
            gst_sample_unref(sample);
            continue;
        }
        // A stage timestamp describes a whole stereo pair. If a later mixer
        // selects eyes from different pairs, do not invent a shared stage time.
        if (side == 0) {
            state.selected.source_stereo_output_ts = timestamps.source_stereo_output_ts;
            state.selected.alignment_output_ts = timestamps.alignment_output_ts;
            state.selected.stereo_output_ts = timestamps.stereo_output_ts;
            state.selected.overlay_output_ts = timestamps.overlay_output_ts;
        } else {
            if (state.selected.source_stereo_output_ts != timestamps.source_stereo_output_ts)
                state.selected.source_stereo_output_ts = 0;
            if (state.selected.alignment_output_ts != timestamps.alignment_output_ts)
                state.selected.alignment_output_ts = 0;
            if (state.selected.stereo_output_ts != timestamps.stereo_output_ts)
                state.selected.stereo_output_ts = 0;
            if (state.selected.overlay_output_ts != timestamps.overlay_output_ts)
                state.selected.overlay_output_ts = 0;
        }
        auto references = side == 0 ? timestamps.left_reference_timestamps : timestamps.right_reference_timestamps;
        if (references.empty()) references = dc_buffer_get_reference_timestamps(buffer);
        if (side == 0) {
            state.selected.left_source_ts = timestamps.left_source_ts;
            state.selected.left_reference_timestamps = std::move(references);
        } else {
            state.selected.right_source_ts = timestamps.right_source_ts;
            state.selected.right_reference_timestamps = std::move(references);
        }
        gst_sample_unref(sample);
    }
    if (!state.camera_only && selected_count != 2) {
        state.selected.source_stereo_output_ts = 0;
        state.selected.alignment_output_ts = 0;
        state.selected.stereo_output_ts = 0;
        state.selected.overlay_output_ts = 0;
    }
}

inline GstPadProbeReturn dc_stereo_output_timestamps(GstPad *, GstPadProbeInfo *info, gpointer user_data) {
    if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) return GST_PAD_PROBE_OK;
    auto &state = *static_cast<DcStereoTimestampState *>(user_data);
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    buffer = gst_buffer_make_writable(buffer);
    GST_PAD_PROBE_INFO_DATA(info) = buffer;
    // Never substitute timestamps from an earlier or merely queued frame.
    DcFrameTimestamps timestamps;
    if (GST_CLOCK_TIME_IS_VALID(state.pts) && GST_BUFFER_PTS(buffer) == state.pts)
        timestamps = state.selected;
    if (state.stage == DcStereoTimestampStage::Source) {
        timestamps.channel = "stereo-LR";
        timestamps.source_stereo_output_ts = dc_clock_realtime_ns();
    } else if (state.stage == DcStereoTimestampStage::Alignment) {
        timestamps.channel = "stereo-LR";
        timestamps.alignment_output_ts = dc_clock_realtime_ns();
        timestamps.stereo_output_ts = timestamps.alignment_output_ts;
    }
    if (state.stage == DcStereoTimestampStage::Preserve && !state.camera_only)
        timestamps.channel = "stereo-LR";
    dc_buffer_set_frame_timestamps(buffer, timestamps);
    return GST_PAD_PROBE_OK;
}

inline bool dc_attach_stereo_timestamps(GstElement *compositor, DcStereoTimestampState *state) {
    if (!compositor || !GST_IS_AGGREGATOR(compositor) || !state) return false;
    GstPad *output = gst_element_get_static_pad(compositor, "src");
    if (!output) return false;
    g_object_set(compositor, "emit-signals", TRUE, nullptr);
    g_signal_connect(compositor, "samples-selected", G_CALLBACK(dc_stereo_samples_selected), state);
    gst_pad_add_probe(output, GST_PAD_PROBE_TYPE_BUFFER, dc_stereo_output_timestamps, state, nullptr);
    gst_object_unref(output);
    return true;
}
// Attach to a compositor or the aggregator inside a GL mixer bin. The
// aggregator owns its callback state, including across display rebuilds.
inline bool dc_attach_mixer_timestamps(GstElement *element, bool camera_only) {
  if (!element) return false;
  GstElement *aggregator = nullptr;
  if (GST_IS_AGGREGATOR(element)) aggregator = GST_ELEMENT(gst_object_ref(element));
  else if (GST_IS_BIN(element)) {
    GstIterator *iterator = gst_bin_iterate_recurse(GST_BIN(element));
    GValue value = G_VALUE_INIT;
    while (gst_iterator_next(iterator, &value) == GST_ITERATOR_OK) {
      auto *child = GST_ELEMENT(g_value_get_object(&value));
      if (GST_IS_AGGREGATOR(child)) {
        aggregator = GST_ELEMENT(gst_object_ref(child));
        g_value_reset(&value);
        break;
      }
      g_value_reset(&value);
    }
    g_value_unset(&value);
    gst_iterator_free(iterator);
  }
  if (aggregator) {
    auto *state = new DcStereoTimestampState;
    state->stage = DcStereoTimestampStage::Preserve;
    state->camera_only = camera_only;
    if (!dc_attach_stereo_timestamps(aggregator, state)) {
      delete state;
      gst_object_unref(aggregator);
      return false;
    }
    g_object_set_data_full(G_OBJECT(aggregator), "dvrk-selected-timestamps", state,
        [](gpointer data) { delete static_cast<DcStereoTimestampState *>(data); });
    gst_object_unref(aggregator);
    return true;
  }
  return false;
}

#endif
