#ifndef DATA_COLLECTION_COMMON_GST_UTILS_HPP
#define DATA_COLLECTION_COMMON_GST_UTILS_HPP

#include <gst/gst.h>
#include <string>

namespace dc {

/**
 * Write a full-detail Graphviz snapshot after caps negotiation.
 *
 * GStreamer writes the file only when GST_DEBUG_DUMP_DOT_DIR is set.  The
 * standard WITH_TS helper adds a timestamp and the .dot extension, avoiding
 * collisions when a pipeline is rebuilt or reconnected.
 */
void dump_dot_after_negotiation(GstElement *pipeline,
                                const std::string &file_name);

/**
 * Schedule the same snapshot after a live pipeline has begun playing.
 *
 * Live pipelines do not necessarily emit GST_MESSAGE_ASYNC_DONE.  The delay
 * gives their first buffers time to negotiate caps before the snapshot.
 */
void schedule_dot_dump_after_negotiation(GstElement *pipeline,
                                         const std::string &file_name,
                                         guint delay_ms = 500);

} // namespace dc

#endif // DATA_COLLECTION_COMMON_GST_UTILS_HPP
