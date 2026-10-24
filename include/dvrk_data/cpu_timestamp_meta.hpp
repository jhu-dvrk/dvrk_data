#ifndef CPU_TIMESTAMP_META_HPP
#define CPU_TIMESTAMP_META_HPP

#include <gst/gst.h>
#include <ctime>
#include <string>
#include <vector>
#include <utility>

#define DC_FRAME_TIMESTAMPS_META_NAME "DataCollectionFrameTimestamps"

// Preserve reference timestamps verbatim: they are not CPU wall time or PTS.
// The caps string identifies the reference domain (e.g. DeckLink hardware or
// stream time); different devices may still have independent clock epochs.
struct DcReferenceTimestamp {
    std::string reference;
    GstClockTime timestamp = GST_CLOCK_TIME_NONE;
    GstClockTime duration = GST_CLOCK_TIME_NONE;
};

struct DcFrameTimestamps {
    std::string channel; // left, right, stereo-LR, or stereo-RL
    gint64 mono_source_ts = 0;
    gint64 left_source_ts = 0;
    gint64 right_source_ts = 0;
    // CPU CLOCK_REALTIME nanoseconds, zero when the stage is unavailable.
    gint64 source_stereo_output_ts = 0;
    gint64 alignment_output_ts = 0;
    gint64 stereo_output_ts = 0; // legacy alignment-output alias
    gint64 overlay_output_ts = 0;
    std::vector<DcReferenceTimestamp> mono_reference_timestamps;
    std::vector<DcReferenceTimestamp> left_reference_timestamps;
    std::vector<DcReferenceTimestamp> right_reference_timestamps;
};

inline const GstMetaInfo *dc_frame_timestamps_meta_register() {
    static const GstMetaInfo *info = nullptr;
    if (g_once_init_enter(&info)) {
        const GstMetaInfo *new_info =
            gst_meta_register_custom_simple(DC_FRAME_TIMESTAMPS_META_NAME);
        g_once_init_leave(&info, new_info);
    }
    return info;
}

inline GstStructure *dc_buffer_get_frame_timestamps_structure(GstBuffer *buffer) {
    if (!buffer) return nullptr;
    dc_frame_timestamps_meta_register();
    GstCustomMeta *meta =
        gst_buffer_get_custom_meta(buffer, DC_FRAME_TIMESTAMPS_META_NAME);
    if (!meta) {
        meta = gst_buffer_add_custom_meta(buffer, DC_FRAME_TIMESTAMPS_META_NAME);
    }
    return meta ? gst_custom_meta_get_structure(meta) : nullptr;
}

inline std::vector<DcReferenceTimestamp> dc_buffer_get_reference_timestamps(GstBuffer *buffer) {
    std::vector<DcReferenceTimestamp> result;
    if (!buffer) return result;
    gpointer state = nullptr;
    while (GstMeta *meta = gst_buffer_iterate_meta_filtered(
               buffer, &state, GST_REFERENCE_TIMESTAMP_META_API_TYPE)) {
        auto *reference = reinterpret_cast<GstReferenceTimestampMeta *>(meta);
        gchar *caps = gst_caps_to_string(reference->reference);
        result.push_back({caps ? caps : "", reference->timestamp, reference->duration});
        g_free(caps);
    }
    return result;
}

inline void dc_structure_set_reference_timestamps(
    GstStructure *structure, const char *field,
    const std::vector<DcReferenceTimestamp> &timestamps) {
    if (timestamps.empty()) {
        gst_structure_remove_field(structure, field);
        return;
    }
    GValue list = G_VALUE_INIT;
    g_value_init(&list, GST_TYPE_LIST);
    for (const auto &timestamp : timestamps) {
        GstStructure *entry = gst_structure_new("reference-timestamp",
            "reference", G_TYPE_STRING, timestamp.reference.c_str(),
            "timestamp", G_TYPE_UINT64, timestamp.timestamp,
            "duration", G_TYPE_UINT64, timestamp.duration, nullptr);
        GValue value = G_VALUE_INIT;
        g_value_init(&value, GST_TYPE_STRUCTURE);
        gst_value_set_structure(&value, entry);
        gst_value_list_append_value(&list, &value);
        g_value_unset(&value);
        gst_structure_free(entry);
    }
    gst_structure_set_value(structure, field, &list);
    g_value_unset(&list);
}

inline std::vector<DcReferenceTimestamp> dc_structure_get_reference_timestamps(
    const GstStructure *structure, const char *field) {
    std::vector<DcReferenceTimestamp> result;
    const GValue *list = gst_structure_get_value(structure, field);
    if (!list || !GST_VALUE_HOLDS_LIST(list)) return result;
    for (guint i = 0; i < gst_value_list_get_size(list); ++i) {
        const GValue *value = gst_value_list_get_value(list, i);
        if (!GST_VALUE_HOLDS_STRUCTURE(value)) continue;
        const GstStructure *entry = gst_value_get_structure(value);
        const char *reference = gst_structure_get_string(entry, "reference");
        DcReferenceTimestamp timestamp;
        if (!reference || !gst_structure_get_uint64(entry, "timestamp", &timestamp.timestamp)) continue;
        timestamp.reference = reference;
        gst_structure_get_uint64(entry, "duration", &timestamp.duration);
        result.push_back(std::move(timestamp));
    }
    return result;
}

inline bool dc_buffer_set_frame_timestamps(GstBuffer *buffer,
                                           const DcFrameTimestamps &timestamps) {
    GstStructure *s = dc_buffer_get_frame_timestamps_structure(buffer);
    if (!s) return false;
    gst_structure_set(s,
                      "channel", G_TYPE_STRING, timestamps.channel.c_str(),
                      "mono-source-ts", G_TYPE_INT64, timestamps.mono_source_ts,
                      "left-source-ts", G_TYPE_INT64, timestamps.left_source_ts,
                      "right-source-ts", G_TYPE_INT64, timestamps.right_source_ts,
                      "source-stereo-output-ts", G_TYPE_INT64, timestamps.source_stereo_output_ts,
                      "alignment-output-ts", G_TYPE_INT64, timestamps.alignment_output_ts,
                      "stereo-output-ts", G_TYPE_INT64, timestamps.stereo_output_ts,
                      "overlay-output-ts", G_TYPE_INT64, timestamps.overlay_output_ts,
                      NULL);
    dc_structure_set_reference_timestamps(s, "mono-reference-timestamps", timestamps.mono_reference_timestamps);
    dc_structure_set_reference_timestamps(s, "left-reference-timestamps", timestamps.left_reference_timestamps);
    dc_structure_set_reference_timestamps(s, "right-reference-timestamps", timestamps.right_reference_timestamps);
    return true;
}

inline DcFrameTimestamps dc_buffer_get_frame_timestamps(GstBuffer *buffer) {
    DcFrameTimestamps timestamps;
    if (!buffer) return timestamps;
    GstCustomMeta *meta =
        gst_buffer_get_custom_meta(buffer, DC_FRAME_TIMESTAMPS_META_NAME);
    if (!meta) return timestamps;

    GstStructure *s = gst_custom_meta_get_structure(meta);
    if (const char *channel = gst_structure_get_string(s, "channel"))
        timestamps.channel = channel;
    gst_structure_get_int64(s, "mono-source-ts", &timestamps.mono_source_ts);
    gst_structure_get_int64(s, "left-source-ts", &timestamps.left_source_ts);
    gst_structure_get_int64(s, "right-source-ts", &timestamps.right_source_ts);
    gst_structure_get_int64(s, "source-stereo-output-ts", &timestamps.source_stereo_output_ts);
    gst_structure_get_int64(s, "alignment-output-ts", &timestamps.alignment_output_ts);
    gst_structure_get_int64(s, "stereo-output-ts", &timestamps.stereo_output_ts);
    gst_structure_get_int64(s, "overlay-output-ts", &timestamps.overlay_output_ts);
    timestamps.mono_reference_timestamps = dc_structure_get_reference_timestamps(s, "mono-reference-timestamps");
    timestamps.left_reference_timestamps = dc_structure_get_reference_timestamps(s, "left-reference-timestamps");
    timestamps.right_reference_timestamps = dc_structure_get_reference_timestamps(s, "right-reference-timestamps");
    return timestamps;
}

inline bool dc_buffer_has_frame_timestamps(GstBuffer *buffer) {
    return buffer &&
           gst_buffer_get_custom_meta(buffer, DC_FRAME_TIMESTAMPS_META_NAME) != nullptr;
}

inline gint64 dc_clock_realtime_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<gint64>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

#endif // CPU_TIMESTAMP_META_HPP
