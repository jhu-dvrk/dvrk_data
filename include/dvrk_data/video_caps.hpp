#ifndef DVRK_DATA_VIDEO_CAPS_HPP
#define DVRK_DATA_VIDEO_CAPS_HPP

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <chrono>
#include <functional>
#include <string>
#include <thread>

namespace dvrk_video {

struct Size {
  int width = 0;
  int height = 0;

  bool valid() const { return width > 0 && height > 0; }
  bool operator==(const Size &other) const {
    return width == other.width && height == other.height;
  }
  bool operator!=(const Size &other) const { return !(*this == other); }
};

inline std::string describe(Size size) {
  return std::to_string(size.width) + "x" + std::to_string(size.height);
}

inline Size size_from_caps(const GstCaps *caps) {
  GstVideoInfo info;
  if (caps == nullptr || !gst_video_info_from_caps(&info, caps)) return {};
  return {static_cast<int>(GST_VIDEO_INFO_WIDTH(&info)),
          static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info))};
}

struct ProbeResult {
  Size size;
  std::string error;
};

// Learn an input's negotiated size before constructing a size-dependent pipeline.
// A short-lived consumer is used so the eventual pipeline can use the exact caps
// announced by unixfdsrc, rather than a second copy of the producer's settings.
inline ProbeResult probe_input_size(
    const std::string &input, const std::function<bool()> &keep_waiting,
    const std::function<void()> &on_waiting) {
  const std::string description = input +
      " ! queue max-size-buffers=1 leaky=downstream"
      " ! appsink name=__size_probe__ sync=false max-buffers=1 drop=true";
  auto last_notice = std::chrono::steady_clock::time_point{};
  while (keep_waiting()) {
    GError *parse_error = nullptr;
    GstElement *pipeline = gst_parse_launch(description.c_str(), &parse_error);
    if (parse_error != nullptr || pipeline == nullptr) {
      const std::string error = parse_error && parse_error->message
                                    ? parse_error->message : "cannot create input pipeline";
      if (parse_error) g_error_free(parse_error);
      if (pipeline) gst_object_unref(pipeline);
      return {{}, error};
    }

    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "__size_probe__");
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    bool reconnect = false;
    ProbeResult result;
    while (keep_waiting()) {
      GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink),
                                                       250 * GST_MSECOND);
      if (sample != nullptr) {
        result.size = size_from_caps(gst_sample_get_caps(sample));
        gst_sample_unref(sample);
        if (!result.size.valid()) {
          result.error = "input did not negotiate positive raw video dimensions";
        }
        break;
      }
      GstMessage *message = gst_bus_pop_filtered(
          bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
      if (message != nullptr) {
        reconnect = true;
        gst_message_unref(message);
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - last_notice >= std::chrono::seconds(5)) {
        on_waiting();
        last_notice = now;
      }
    }
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(sink);
    gst_object_unref(pipeline);
    if (!reconnect) return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  return {{}, "stopped while waiting for video dimensions"};
}

struct CapsMonitor {
  std::string label;
  Size expected;
  Size first;
  std::function<void(const std::string &)> on_error;
  std::function<void(Size)> on_first;
  bool failed = false;
};

inline GstPadProbeReturn caps_monitor_callback(GstPad *, GstPadProbeInfo *info,
                                               gpointer user_data) {
  if (!(info->type & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM)) return GST_PAD_PROBE_OK;
  GstEvent *event = GST_PAD_PROBE_INFO_EVENT(info);
  if (GST_EVENT_TYPE(event) != GST_EVENT_CAPS) return GST_PAD_PROBE_OK;
  auto &monitor = *static_cast<CapsMonitor *>(user_data);
  if (monitor.failed) return GST_PAD_PROBE_DROP;
  GstCaps *caps = nullptr;
  gst_event_parse_caps(event, &caps);
  const Size size = size_from_caps(caps);
  std::string error;
  if (!size.valid()) {
    error = monitor.label + " has no valid raw video dimensions";
  } else if (monitor.expected.valid() && size != monitor.expected) {
    error = monitor.label + " is " + describe(size) + ", configured for " +
            describe(monitor.expected) + "; update the configuration and restart";
  } else if (monitor.first.valid() && size != monitor.first) {
    error = monitor.label + " changed from " + describe(monitor.first) +
            " to " + describe(size) + "; restart the node";
  }
  if (!error.empty()) {
    monitor.failed = true;
    monitor.on_error(error);
    return GST_PAD_PROBE_DROP;
  }
  if (!monitor.first.valid()) {
    monitor.first = size;
    if (monitor.on_first) monitor.on_first(size);
  }
  return GST_PAD_PROBE_OK;
}

inline bool add_caps_monitor(GstElement *pipeline, const char *element_name,
                             CapsMonitor *monitor) {
  GstElement *element = gst_bin_get_by_name(GST_BIN(pipeline), element_name);
  if (!element) return false;
  GstPad *pad = gst_element_get_static_pad(element, "sink");
  if (!pad) {
    gst_object_unref(element);
    return false;
  }
  gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM,
                    caps_monitor_callback, monitor, nullptr);
  gst_object_unref(pad);
  gst_object_unref(element);
  return true;
}

}  // namespace dvrk_video

#endif
