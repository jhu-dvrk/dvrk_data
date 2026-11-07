#include <dvrk_data/video_layout.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <stdexcept>
#include <iostream>
#include <unistd.h>

using dvrk_video::Layout;
void check(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}

Layout sample_layout(const char *extra, const char *channel = "", int width = 32) {
    const auto caps_text = "video/x-raw,format=RGBA,width=" + std::to_string(width) +
        ",height=8,framerate=30/1" + extra;
    auto *caps = gst_caps_from_string(caps_text.c_str());
    auto *buffer = gst_buffer_new();
    if (*channel) {
        DcFrameTimestamps timestamps;
        timestamps.channel = channel;
        dc_buffer_set_frame_timestamps(buffer, timestamps);
    }
    auto *sample = gst_sample_new(buffer, caps, nullptr, nullptr);
    const auto result = dvrk_video::layout_from_sample(sample);
    gst_sample_unref(sample);
    gst_buffer_unref(buffer);
    gst_caps_unref(caps);
    return result;
}

int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    dc_frame_timestamps_meta_register();
    check(sample_layout("") == Layout::Unknown, "Untagged wide video must not imply stereo");
    check(sample_layout(",dvrk-video-layout=mono") == Layout::Mono, "Mono caps were ignored");
    check(sample_layout(",dvrk-video-layout=stereo-LR") == Layout::SideBySideLR, "LR caps were ignored");
    check(sample_layout(",dvrk-video-layout=stereo-RL") == Layout::SideBySideRL, "RL caps were ignored");
    check(sample_layout(",dvrk-video-layout=stereo-LR", "", 15) == Layout::Unknown, "Odd-width stereo accepted");
    check(sample_layout(",multiview-mode=side-by-side,views=2") == Layout::SideBySideLR, "Standard stereo caps ignored");
    check(sample_layout("", "stereo-LR") == Layout::SideBySideLR, "Legacy frame metadata ignored");
    check(sample_layout("", "stereo-RL") == Layout::SideBySideRL, "Legacy RL metadata ignored");
    check(sample_layout("", "mono") == Layout::Mono, "Legacy mono metadata ignored");
    if (argc > 1 && std::string(argv[1]) == "--unit") return 0;

    std::vector<GstElement *> producers;
    std::vector<std::string> sockets;
    for (const char *layout : {"mono", "stereo-LR", "stereo-RL"}) {
        const auto socket = "@dvrk:arbitrary:layout_" + std::string(layout) + "_" + std::to_string(getpid());
        sockets.push_back(socket);
        const auto pipeline = "videotestsrc is-live=true ! video/x-raw,format=RGBA,width=32,height=8,"
            "framerate=30/1,dvrk-video-layout=" + std::string(layout) + " ! " + dvrk_gst::build_sink(socket);
        GError *error = nullptr;
        auto *producer = gst_parse_launch(pipeline.c_str(), &error);
        check(producer && !error, "Cannot construct layout producer");
        check(gst_element_set_state(producer, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE,
              "Cannot start layout producer");
        gst_element_get_state(producer, nullptr, nullptr, GST_SECOND);
        producers.push_back(producer);
    }
    const auto detected = dvrk_video::probe_socket_layouts(sockets, std::chrono::milliseconds(1000));
    check(detected.at(sockets[0]) == Layout::Mono, "Socket mono layout was lost");
    check(detected.at(sockets[1]) == Layout::SideBySideLR, "Socket LR layout was lost");
    check(detected.at(sockets[2]) == Layout::SideBySideRL, "Socket RL layout was lost");
    for (auto *producer : producers) {
        gst_element_set_state(producer, GST_STATE_NULL);
        gst_object_unref(producer);
    }
    const auto start = std::chrono::steady_clock::now();
    const auto missing = dvrk_video::probe_socket_layouts(
        {"@dvrk:arbitrary:missing_layout_" + std::to_string(getpid())}, std::chrono::milliseconds(50));
    check(missing.begin()->second == Layout::Unknown, "Unavailable socket classified as stereo");
    check(std::chrono::steady_clock::now() - start < std::chrono::seconds(1), "Discovery exceeded its deadline");
    std::cout << "Layout declarations, legacy metadata and bounded socket discovery passed\n";
}
