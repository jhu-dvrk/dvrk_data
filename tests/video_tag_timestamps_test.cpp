#include <dvrk_data/video_timeline.hpp>
#include <vector>

int main() {
    std::vector<long long> empty;
    dc::normalize_video_pts(empty);
    if (!empty.empty()) return 1;
    for (long long origin : {0LL, 20311938412LL, 307217843676LL}) {
        std::vector<long long> pts{origin, origin + 16666667, origin + 307677127213LL};
        dc::normalize_video_pts(pts);
        if (pts != std::vector<long long>{0, 16666667, 307677127213LL}) return 1;
        dc::normalize_video_pts(pts);
        if (pts.front() != 0 || pts.back() != 307677127213LL) return 1;
    }
    return 0;
}
