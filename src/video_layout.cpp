#include <dvrk_data/video_layout.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <memory>
#include <thread>

namespace dvrk_video {

Layout layout_from_sample(GstSample *sample) {
    if (!sample) return Layout::Unknown;
    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample))) return Layout::Unknown;
    const bool valid_stereo = info.width > 0 && info.width % 2 == 0;
    const char *declared = gst_structure_get_string(
        gst_caps_get_structure(gst_sample_get_caps(sample), 0), "dvrk-video-layout");
    if (declared) {
        const std::string layout(declared);
        if (layout == "mono") return Layout::Mono;
        if (valid_stereo && layout == "stereo-LR") return Layout::SideBySideLR;
        if (valid_stereo && layout == "stereo-RL") return Layout::SideBySideRL;
        return Layout::Unknown;
    }
    const char *mode = gst_structure_get_string(
        gst_caps_get_structure(gst_sample_get_caps(sample), 0), "multiview-mode");
    if (mode && std::string(mode) == "mono") return Layout::Mono;
    if (mode && std::string(mode) == "side-by-side") {
        if (!valid_stereo) return Layout::Unknown;
        return (GST_VIDEO_INFO_MULTIVIEW_FLAGS(&info) & GST_VIDEO_MULTIVIEW_FLAGS_RIGHT_VIEW_FIRST)
            ? Layout::SideBySideRL : Layout::SideBySideLR;
    }
    const auto channel = dc_buffer_get_frame_timestamps(gst_sample_get_buffer(sample)).channel;
    if (channel == "mono" || channel == "left" || channel == "right") return Layout::Mono;
    if (valid_stereo && channel == "stereo-LR") return Layout::SideBySideLR;
    if (valid_stereo && channel == "stereo-RL") return Layout::SideBySideRL;
    return Layout::Unknown;
}

std::map<std::string, Layout> probe_socket_layouts(
    const std::vector<std::string> &sockets, std::chrono::milliseconds timeout) {
    struct Probe {
        GstElement *pipeline = nullptr;
        GstElement *sink = nullptr;
        ~Probe() {
            if (pipeline) gst_element_set_state(pipeline, GST_STATE_NULL);
            if (sink) gst_object_unref(sink);
            if (pipeline) gst_object_unref(pipeline);
        }
    };
    dc_frame_timestamps_meta_register();
    std::map<std::string, Layout> result;
    std::map<std::string, std::unique_ptr<Probe>> pending;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (const auto &socket : sockets) {
        result[socket] = Layout::Unknown;
        auto probe = std::make_unique<Probe>();
        const auto description = dvrk_gst::build_src(socket) +
            " ! appsink name=layout_probe sync=false max-buffers=1 drop=true";
        GError *error = nullptr;
        probe->pipeline = gst_parse_launch(description.c_str(), &error);
        if (error) { g_error_free(error); continue; }
        if (!probe->pipeline) continue;
        probe->sink = gst_bin_get_by_name(GST_BIN(probe->pipeline), "layout_probe");
        if (!probe->sink || gst_element_set_state(probe->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
            continue;
        pending.emplace(socket, std::move(probe));
    }
    while (!pending.empty() && std::chrono::steady_clock::now() < deadline) {
        for (auto it = pending.begin(); it != pending.end();) {
            auto *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(it->second->sink), 0);
            if (sample) {
                result[it->first] = layout_from_sample(sample);
                gst_sample_unref(sample);
                it = pending.erase(it);
            } else {
                ++it;
            }
        }
        if (!pending.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return result;
}

}  // namespace dvrk_video
