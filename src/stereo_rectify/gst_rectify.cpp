// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#include "gst_rectify.hpp"
#include <dvrk_data/calibration/rectification.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <gst/video/gstvideofilter.h>
#include <string>

namespace {
struct RectifyState {
    std::string path;
    bool right = false;
    dvrk_data::calibration::Rectification rectification;
};
}

typedef struct _DcRectify {
    GstVideoFilter parent;
    RectifyState *state;
} DcRectify;
typedef struct _DcRectifyClass { GstVideoFilterClass parent_class; } DcRectifyClass;
G_DEFINE_TYPE(DcRectify, dc_rectify, GST_TYPE_VIDEO_FILTER)

namespace {
enum { PROP_NONE, PROP_FILE, PROP_RIGHT };

void set_property(GObject *object, guint property, const GValue *value, GParamSpec *spec) {
    auto &state = *reinterpret_cast<DcRectify *>(object)->state;
    switch (property) {
    case PROP_FILE: {
        const char *path = g_value_get_string(value);
        state.path = path ? path : "";
        break;
    }
    case PROP_RIGHT: state.right = g_value_get_boolean(value); break;
    default: G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property, spec);
    }
}
void get_property(GObject *object, guint property, GValue *value, GParamSpec *spec) {
    const auto &state = *reinterpret_cast<DcRectify *>(object)->state;
    switch (property) {
    case PROP_FILE: g_value_set_string(value, state.path.c_str()); break;
    case PROP_RIGHT: g_value_set_boolean(value, state.right); break;
    default: G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property, spec);
    }
}
gboolean start(GstBaseTransform *transform) {
    auto &state = *reinterpret_cast<DcRectify *>(transform)->state;
    std::string error;
    if (!state.rectification.load(state.path, error)) {
        GST_ELEMENT_ERROR(transform, RESOURCE, SETTINGS, ("%s", error.c_str()), (nullptr));
        return FALSE;
    }
    return TRUE;
}
gboolean set_info(GstVideoFilter *filter, GstCaps *, GstVideoInfo *in,
                  GstCaps *, GstVideoInfo *out) {
    const auto size = reinterpret_cast<DcRectify *>(filter)->state->rectification.image_size();
    if (in->width != size.width || in->height != size.height ||
        out->width != size.width || out->height != size.height ||
        GST_VIDEO_INFO_FORMAT(in) != GST_VIDEO_FORMAT_BGR ||
        GST_VIDEO_INFO_FORMAT(out) != GST_VIDEO_FORMAT_BGR) {
        GST_ELEMENT_ERROR(filter, STREAM, FORMAT,
            ("Rectification requires calibrated %dx%d BGR frames; got %dx%d",
             size.width, size.height, in->width, in->height), (nullptr));
        return FALSE;
    }
    return TRUE;
}
GstFlowReturn transform_frame(GstVideoFilter *filter, GstVideoFrame *in, GstVideoFrame *out) {
    try {
        const auto &state = *reinterpret_cast<DcRectify *>(filter)->state;
        cv::Mat source(in->info.height, in->info.width, CV_8UC3,
                       GST_VIDEO_FRAME_PLANE_DATA(in, 0), GST_VIDEO_FRAME_PLANE_STRIDE(in, 0));
        cv::Mat destination(out->info.height, out->info.width, CV_8UC3,
                            GST_VIDEO_FRAME_PLANE_DATA(out, 0), GST_VIDEO_FRAME_PLANE_STRIDE(out, 0));
        state.rectification.remap(source, destination, state.right);
        // Preserve per-eye source and hardware-reference provenance explicitly.
        // GstBaseTransform also carries PTS/duration/flags to the output buffer.
        dc_buffer_set_frame_timestamps(out->buffer, dc_buffer_get_frame_timestamps(in->buffer));
        return GST_FLOW_OK;
    } catch (const std::exception &error) {
        GST_ELEMENT_ERROR(filter, STREAM, FAILED, ("%s", error.what()), (nullptr));
        return GST_FLOW_ERROR;
    }
}
void finalize(GObject *object) {
    delete reinterpret_cast<DcRectify *>(object)->state;
    G_OBJECT_CLASS(dc_rectify_parent_class)->finalize(object);
}
}

static void dc_rectify_class_init(DcRectifyClass *klass) {
    auto *object = G_OBJECT_CLASS(klass);
    object->set_property = set_property;
    object->get_property = get_property;
    object->finalize = finalize;
    const auto flags = static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | GST_PARAM_MUTABLE_READY);
    g_object_class_install_property(object, PROP_FILE,
        g_param_spec_string("calibration-file", "Calibration file", "Stereo calibration YAML or JSON", "", flags));
    g_object_class_install_property(object, PROP_RIGHT,
        g_param_spec_boolean("right-eye", "Right eye", "Use the right camera rectification maps", FALSE, flags));
    auto *element = GST_ELEMENT_CLASS(klass);
    gst_element_class_set_static_metadata(element, "dVRK stereo rectification", "Filter/Effect/Video",
        "Apply calibrated distortion and epipolar rectification maps", "Johns Hopkins University, LCSR");
    GstCaps *caps = gst_caps_from_string("video/x-raw,format=BGR");
    gst_element_class_add_pad_template(element, gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
    gst_element_class_add_pad_template(element, gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
    gst_caps_unref(caps);
    GST_BASE_TRANSFORM_CLASS(klass)->start = start;
    GST_VIDEO_FILTER_CLASS(klass)->set_info = set_info;
    GST_VIDEO_FILTER_CLASS(klass)->transform_frame = transform_frame;
}
static void dc_rectify_init(DcRectify *self) {
    self->state = new RectifyState;
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), FALSE);
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), FALSE);
}
bool register_rectify_element() {
    static const bool registered = gst_element_register(nullptr, "dvrkrectify", GST_RANK_NONE, dc_rectify_get_type());
    return registered;
}
