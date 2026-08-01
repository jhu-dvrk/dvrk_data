#ifndef TAGS_HPP
#define TAGS_HPP

#include <string>

namespace dc {
    /**
     * @brief Returns current date/time in "YYYY-MM-DDTHH:MM:SS.sss" format (local time)
     */
    std::string get_current_timestamp_iso8601();

}

#endif // TAGS_HPP
