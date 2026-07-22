#include <dvrk_data/gst_utils.hpp>
namespace dc {

namespace {

struct PendingDotDump {
    GstElement *pipeline;
    std::string file_name;
};

gboolean dump_dot_timeout(gpointer user_data) {
    auto *pending = static_cast<PendingDotDump *>(user_data);
    GstState state = GST_STATE_NULL;
    gst_element_get_state(pending->pipeline, &state, nullptr, 0);
    if (state == GST_STATE_PLAYING) {
        dump_dot_after_negotiation(pending->pipeline, pending->file_name);
    }
    gst_object_unref(pending->pipeline);
    delete pending;
    return G_SOURCE_REMOVE;
}

} // namespace

void dump_dot_after_negotiation(GstElement *pipeline,
                                const std::string &file_name) {
    if (pipeline == nullptr || !GST_IS_BIN(pipeline)) {
        return;
    }
    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
        GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, file_name.c_str());
}

void schedule_dot_dump_after_negotiation(GstElement *pipeline,
                                         const std::string &file_name,
                                         guint delay_ms) {
    if (pipeline == nullptr || !GST_IS_BIN(pipeline)) {
        return;
    }
    auto *pending = new PendingDotDump{
        GST_ELEMENT(gst_object_ref(pipeline)), file_name};
    g_timeout_add(delay_ms, dump_dot_timeout, pending);
}

} // namespace dc
