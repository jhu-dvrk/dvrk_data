#ifndef DVRK_DATA_STEREO_COMMON_HPP
#define DVRK_DATA_STEREO_COMMON_HPP

#include <glib-unix.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <dvrk_data/config.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>

namespace dc_stereo {

inline GMainLoop *g_main_loop = nullptr;

struct PipelineReconnector {
  GstElement *pipeline = nullptr;
  rclcpp::Node *node = nullptr;
  guint timer_id = 0;
  bool is_active = false;
  std::string name;

  void start(GstElement *pipe, rclcpp::Node *n, const std::string &pipeline_name) {
    pipeline = pipe;
    node = n;
    name = pipeline_name;
    is_active = true;
  }

  void stop() {
    if (timer_id != 0) {
      g_source_remove(timer_id);
      timer_id = 0;
    }
    is_active = false;
    pipeline = nullptr;
  }

  void handle_error_or_eos() {
    if (!is_active || !pipeline) return;

    gst_element_set_state(pipeline, GST_STATE_NULL);

    if (timer_id == 0) {
      RCLCPP_WARN(node->get_logger(), "[%s] Connection lost. Retrying to reconnect...", name.c_str());
      timer_id = g_timeout_add(1000, [](gpointer data) -> gboolean {
        auto *self = static_cast<PipelineReconnector *>(data);
        if (!self->is_active || !self->pipeline) {
          self->timer_id = 0;
          return FALSE;
        }

        GstStateChangeReturn ret = gst_element_set_state(self->pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
          return TRUE;
        }

        RCLCPP_INFO(self->node->get_logger(), "[%s] Successfully reconnected!", self->name.c_str());
        self->timer_id = 0;
        return FALSE;
      }, this);
    }
  }
};

struct PipelineUserData {
  rclcpp::Node *node = nullptr;
  PipelineReconnector reconnector;
};

inline void warn_if_interlaced_stream(const std::string &stream,
                                      const rclcpp::Logger &logger,
                                      const std::string &name) {
  if (stream.empty()) {
    return;
  }

  const std::string probe_pipeline =
      stream + " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
               "leaky=downstream"
               " ! appsink name=__caps_probe_sink__ sync=false async=false "
               "emit-signals=false drop=true max-buffers=1";

  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(probe_pipeline.c_str(), &error);
  if (error != nullptr || pipeline == nullptr) {
    if (error != nullptr) {
      RCLCPP_WARN(logger, "Unable to probe caps for '%s' stream: %s",
                  name.c_str(),
                  error->message != nullptr ? error->message : "unknown error");
      g_error_free(error);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
    return;
  }

  GstElement *probe_sink =
      gst_bin_get_by_name(GST_BIN(pipeline), "__caps_probe_sink__");
  if (probe_sink == nullptr) {
    gst_object_unref(pipeline);
    RCLCPP_WARN(logger,
                "Unable to probe caps for '%s' stream: missing probe sink",
                name.c_str());
    return;
  }

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  GstSample *sample =
      gst_app_sink_try_pull_sample(GST_APP_SINK(probe_sink), 2 * GST_SECOND);

  if (sample != nullptr) {
    GstCaps *caps = gst_sample_get_caps(sample);
    if (caps != nullptr && gst_caps_get_size(caps) > 0) {
      const GstStructure *structure = gst_caps_get_structure(caps, 0);
      const gchar *interlace_mode =
          gst_structure_get_string(structure, "interlace-mode");
      if (interlace_mode != nullptr &&
          std::strcmp(interlace_mode, "progressive") != 0) {
        RCLCPP_WARN(logger,
                    "%s stream caps report interlace-mode='%s'. Consider "
                    "adding deinterlace to this stream in the config.",
                    name.c_str(), interlace_mode);
      }
    }
    gst_sample_unref(sample);
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(probe_sink);
  gst_object_unref(pipeline);
}

/// Resolve a sink's "socket" field to a full abstract name using the given role.
inline std::string resolve_unixfd_sink(
    const sv::UnixfdSinkConfig &sink, const std::string &default_role) {
  return dvrk_gst::resolve(sink.socket, default_role);
}

/// Collect all unixfd sinks whose short name (last ':' component) matches.
inline std::vector<sv::UnixfdSinkConfig>
collect_unixfd_sinks(const sv::AppConfig &cfg, const std::string &role,
                     const std::string &short_name) {
  std::vector<sv::UnixfdSinkConfig> sinks;
  const std::string full = dvrk_gst::make(role, short_name);
  for (const auto &sink : cfg.unixfd_sinks) {
    const std::string resolved = dvrk_gst::resolve(sink.socket, role);
    if (resolved == full) {
      sinks.push_back(sink);
    }
  }
  return sinks;
}

/// Resolve a source's "socket" field to a full abstract name using the given role.
inline std::string resolve_unixfd_source(
    const sv::UnixfdSourceConfig &source, const std::string &default_role) {
  return dvrk_gst::resolve(source.socket, default_role);
}

/// Collect all unixfd sources whose short name (last ':' component) matches.
inline std::vector<sv::UnixfdSourceConfig>
collect_unixfd_sources(const sv::AppConfig &cfg, const std::string &source_role,
                       const std::string &short_name) {
  std::vector<sv::UnixfdSourceConfig> sources;
  const std::string full = dvrk_gst::make(source_role, short_name);
  for (const auto &src : cfg.unixfd_sources) {
    const std::string resolved = dvrk_gst::resolve(src.socket, source_role);
    if (resolved == full) {
      sources.push_back(src);
    }
  }
  return sources;
}

/// Build a GStreamer unixfdsrc fragment from a fully-qualified abstract name.
/// Convenience wrapper around dvrk_gst::build_src().
inline std::string build_unixfdsrc_string(const std::string &abstract_name,
                                          int width, int height) {
  return dvrk_gst::build_src(abstract_name, width, height);
}

/// Ensure at least one sink with the given short name exists; add a default if empty.
inline void ensure_sink(std::vector<sv::UnixfdSinkConfig> &sinks,
                        const std::string &short_name) {
  if (!sinks.empty()) {
    return;
  }
  sv::UnixfdSinkConfig sink;
  sink.socket = short_name;
  sinks.push_back(std::move(sink));
}

/// For abstract sockets the kernel reclaims them automatically when the process
/// exits, so there is nothing to remove.  This function is kept for API
/// compatibility but is a no-op; it logs the sockets that will be (re)opened.
inline void remove_stale_sockets(const std::string & /*app_name*/,
                                 const std::vector<sv::UnixfdSinkConfig> &sinks,
                                 const std::string &role,
                                 const rclcpp::Logger &logger) {
  for (const auto &sink : sinks) {
    RCLCPP_DEBUG(logger, "unixfd abstract sink: %s",
                 dvrk_gst::resolve(sink.socket, role).c_str());
  }
}

inline bool validate_pipeline(const std::string &pipeline_string,
                              const rclcpp::Logger &logger,
                              const std::string &name) {
  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
  if (error == nullptr && pipeline != nullptr) {
    gst_object_unref(pipeline);
    return true;
  }

  RCLCPP_ERROR(logger, "Unable to parse GStreamer pipeline '%s': %s",
               name.c_str(),
               error && error->message ? error->message : "unknown error");
  if (error != nullptr) {
    g_error_free(error);
  }
  if (pipeline != nullptr) {
    gst_object_unref(pipeline);
  }
  return false;
}

inline gboolean on_sigint(gpointer) {
  if (g_main_loop != nullptr) {
    g_main_loop_quit(g_main_loop);
  }
  return G_SOURCE_REMOVE;
}

inline gboolean on_ros_spin(gpointer user_data) {
  if (user_data == nullptr || !rclcpp::ok()) {
    return G_SOURCE_CONTINUE;
  }

  auto *node = static_cast<rclcpp::Node *>(user_data);
  rclcpp::spin_some(node->get_node_base_interface());
  return G_SOURCE_CONTINUE;
}

inline gboolean on_bus_message(GstBus *, GstMessage *msg, gpointer user_data) {
  if (msg == nullptr || user_data == nullptr) {
    return G_SOURCE_CONTINUE;
  }

  auto *data = static_cast<PipelineUserData *>(user_data);
  auto *node = data->node;

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
    RCLCPP_INFO(node->get_logger(), "GStreamer bus: received EOS");
    data->reconnector.handle_error_or_eos();
  } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    GError *err = nullptr;
    gchar *dbg = nullptr;
    gst_message_parse_error(msg, &err, &dbg);
    RCLCPP_ERROR(node->get_logger(), "GStreamer error: %s",
                 err && err->message ? err->message : "unknown");
    if (dbg != nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Debug details: %s", dbg);
      g_free(dbg);
    }
    if (err != nullptr) {
      g_error_free(err);
    }
    data->reconnector.handle_error_or_eos();
  } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_WARNING) {
    GError *err = nullptr;
    gchar *dbg = nullptr;
    gst_message_parse_warning(msg, &err, &dbg);
    RCLCPP_WARN(node->get_logger(), "GStreamer warning: %s",
                 err && err->message ? err->message : "unknown");
    if (dbg != nullptr) {
      RCLCPP_WARN(node->get_logger(), "Debug details: %s", dbg);
      g_free(dbg);
    }
    if (err != nullptr) {
      g_error_free(err);
    }
  }

  return G_SOURCE_CONTINUE;
}

inline int run_pipeline(GstElement *pipeline,
                        const std::shared_ptr<rclcpp::Node> &node,
                        const std::string &started_message) {
  PipelineUserData user_data;
  user_data.node = node.get();
  user_data.reconnector.start(pipeline, node.get(), "stereo_pipeline");

  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, on_bus_message, &user_data);
  gst_object_unref(bus);

  g_unix_signal_add(SIGINT, on_sigint, nullptr);
  g_unix_signal_add(SIGTERM, on_sigint, nullptr);
  g_timeout_add(20, on_ros_spin, node.get());

  g_main_loop = g_main_loop_new(nullptr, FALSE);
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  RCLCPP_INFO(node->get_logger(), "%s", started_message.c_str());
  g_main_loop_run(g_main_loop);

  user_data.reconnector.stop();

  g_main_loop_unref(g_main_loop);
  g_main_loop = nullptr;

  gst_element_send_event(pipeline, gst_event_new_eos());
  gst_element_set_state(pipeline, GST_STATE_NULL);
  return 0;
}

}  // namespace dc_stereo

#endif  // DVRK_DATA_STEREO_COMMON_HPP
