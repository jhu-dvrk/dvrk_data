#include <dvrk_data/tags.hpp>
#include <ctime>
#include <cstdio>
#include <cstring>

namespace dc {

std::string get_current_timestamp_iso8601() {
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    char buf[100];
    struct tm* tm_info = localtime(&now.tv_sec);
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", tm_info);
    std::sprintf(buf + std::strlen(buf), ".%03ld", now.tv_nsec / 1000000);
    return std::string(buf);
}

} // namespace dc
