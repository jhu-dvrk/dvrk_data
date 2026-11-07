#ifndef DVRK_DATA_VIDEO_LAYOUT_HPP
#define DVRK_DATA_VIDEO_LAYOUT_HPP

#include <gst/gst.h>
#include <chrono>
#include <map>
#include <string>
#include <vector>

namespace dvrk_video {

enum class Layout { Unknown, Mono, SideBySideLR, SideBySideRL };

// Read dvrk-video-layout caps (mono/stereo-LR/stereo-RL), standard multiview
// caps, or the channel in DataCollectionFrameTimestamps for older producers.
Layout layout_from_sample(GstSample *sample);

// Probe sockets concurrently within one shared deadline. Call off the UI thread;
// quiet, disconnected, and undeclared streams return Unknown.
std::map<std::string, Layout> probe_socket_layouts(
    const std::vector<std::string> &sockets,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(250));

}  // namespace dvrk_video
#endif
