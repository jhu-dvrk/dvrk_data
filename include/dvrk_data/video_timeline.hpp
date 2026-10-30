#ifndef DVRK_DATA_VIDEO_TIMELINE_HPP
#define DVRK_DATA_VIDEO_TIMELINE_HPP

#include <vector>

namespace dc {
// Sidecars observe the live recorder clock; MP4 media time starts at zero.
inline void normalize_video_pts(std::vector<long long>& timestamps) {
    if (timestamps.empty()) return;
    const long long origin = timestamps.front();
    for (auto& timestamp : timestamps) timestamp -= origin;
}
} // namespace dc

#endif
