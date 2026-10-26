#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <iostream>
#include <string>

#include <dvrk_data/config.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <dvrk_data/stereo_common.hpp>
#include <dvrk_data/latest_video.hpp>
#include <dvrk_data/video_caps.hpp>

#include <atomic>

namespace {

struct CommandLineOptions {
  std::string config_file;
};

void print_usage(const char *executable) {
  std::cerr << "Usage: " << executable << " -c <config.json>" << std::endl;
}

bool parse_arguments(int argc, char *argv[], CommandLineOptions &options) {
  bool seen_config = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ros-args") {
      break;
    }

    if (arg == "-c" && i + 1 < argc) {
      if (seen_config) {
        std::cerr << "Error: multiple -c arguments are not supported."
                  << std::endl;
        return false;
      }
      options.config_file = argv[++i];
      seen_config = true;
      continue;
    }

    std::cerr << "Error: unknown argument '" << arg << "'." << std::endl;
    return false;
  }

  if (!seen_config) {
    std::cerr << "Error: exactly one config file is required." << std::endl;
    return false;
  }

  return true;
}

GstPadProbeReturn source_timestamp_probe_cb(GstPad *pad,
                                            GstPadProbeInfo *info,
                                            gpointer user_data) {
  (void)pad;
  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) {
    return GST_PAD_PROBE_OK;
  }

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  if (!gst_buffer_is_writable(buf)) {
    buf = gst_buffer_make_writable(buf);
    GST_PAD_PROBE_INFO_DATA(info) = buf;
  }

  DcFrameTimestamps timestamps = dc_buffer_get_frame_timestamps(buf);
  const gint64 now = dc_clock_realtime_ns();
  const int side = GPOINTER_TO_INT(user_data);
  if (side == 0) {
    timestamps.channel = "left";
    timestamps.left_source_ts = now;
    const auto references = dc_buffer_get_reference_timestamps(buf);
    if (!references.empty()) timestamps.left_reference_timestamps = references;
  } else {
    timestamps.channel = "right";
    timestamps.right_source_ts = now;
    const auto references = dc_buffer_get_reference_timestamps(buf);
    if (!references.empty()) timestamps.right_reference_timestamps = references;
  }
  dc_buffer_set_frame_timestamps(buf, timestamps);
  return GST_PAD_PROBE_OK;
}

void add_timestamp_probe(GstElement *pipeline, const std::string &element_name,
                         int side) {
  GstElement *element =
      gst_bin_get_by_name(GST_BIN(pipeline), element_name.c_str());
  if (element == nullptr) {
    return;
  }

  GstPad *pad = gst_element_get_static_pad(element, "src");
  if (pad != nullptr) {
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                      source_timestamp_probe_cb, GINT_TO_POINTER(side),
                      nullptr);
    gst_object_unref(pad);
  }
  gst_object_unref(element);
}

std::string build_pipeline_string(const dvrk_data::AppConfig &cfg) {
  const auto branch = [](const std::string &input, const std::string &side,
                         const std::string &output, int sink) {
    std::string result = input + " ! queue name=__" + side +
        "_src_q__ max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream"
        " ! tee name=__" + side + "_tee__ ";
    if (!output.empty()) {
      result += " __" + side + "_tee__. ! queue max-size-buffers=1 leaky=downstream"
          " ! videoconvert ! video/x-raw,format=I420 ! " +
          dvrk_gst::build_sink(dvrk_gst::resolve(output)) + " ";
    }
    result += " __" + side + "_tee__. ! queue max-size-buffers=1 leaky=downstream"
        " ! videoconvert ! video/x-raw,format=I420 ! " +
        dvrk_video::LatestVideoMixers::sink(sink == 0 ? "__latest_left__" : "__latest_right__") + " ";
    return result;
  };
  return branch(cfg.left.gst_input, "left", cfg.left.gst_output, 0) +
         branch(cfg.right.gst_input, "right", cfg.right.gst_output, 1) +
         " " + dvrk_video::LatestVideoMixers::source("stereo_mix") +
         " ! videoconvert ! video/x-raw,format=I420,width=" +
         std::to_string(2 * cfg.original_width) + ",height=" +
         std::to_string(cfg.original_height) +
         " ! queue max-size-buffers=1 leaky=downstream ! " +
         dvrk_gst::build_sink(dvrk_gst::resolve(cfg.stereo.gst_output));
}

}  // namespace

int main(int argc, char *argv[]) {
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  dc_frame_timestamps_meta_register();

  CommandLineOptions options;
  if (!parse_arguments(argc, argv, options)) {
    print_usage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("stereo_source");
  const std::string &path = options.config_file;
  if (!std::filesystem::exists(path)) {
    RCLCPP_ERROR(node->get_logger(), "Config file does not exist: %s",
                 path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  Json::Value root;
  if (!dvrk_data::Config::load_from_file(path, root)) {
    rclcpp::shutdown();
    return 1;
  }

  if (!dvrk_data::Config::check_type(root, "dvrk_data:stereo_source@1.0.0", path)) {
    rclcpp::shutdown();
    return 1;
  }

  dvrk_data::AppConfig cfg;
  try {
    cfg = dvrk_data::Config::parse_app_config(root);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  if (cfg.left.gst_input.empty() || cfg.right.gst_input.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must define camera.left.gst_input and "
                 "camera.right.gst_input",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (!cfg.stereo.gst_output_specified)
    cfg.stereo.gst_output = dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "stereo");
  if (cfg.stereo.gst_output.empty()) {
    RCLCPP_ERROR(node->get_logger(), "stereo_source requires a stereo gst_output");
    rclcpp::shutdown(); return 1;
  }

  cfg.left.gst_input = dvrk_gst::build_input(
      cfg.left.gst_input, dvrk_gst::ROLE_STEREO_SOURCE,
      cfg.auto_camera_size ? 0 : cfg.original_width,
      cfg.auto_camera_size ? 0 : cfg.original_height);
  cfg.right.gst_input = dvrk_gst::build_input(
      cfg.right.gst_input, dvrk_gst::ROLE_STEREO_SOURCE,
      cfg.auto_camera_size ? 0 : cfg.original_width,
      cfg.auto_camera_size ? 0 : cfg.original_height);

  if (cfg.auto_camera_size) {
    dvrk_video::Size detected;
    for (const auto &input : {cfg.left.gst_input, cfg.right.gst_input}) {
      const auto result = dvrk_video::probe_input_size(
          input, [] { return rclcpp::ok(); },
          [&] { RCLCPP_INFO(node->get_logger(), "Waiting for camera video caps"); });
      if (!result.error.empty() || !result.size.valid() ||
          (detected.valid() && detected != result.size)) {
        RCLCPP_ERROR(node->get_logger(), "Camera size detection failed: %s",
                     result.error.empty() ? "left/right sizes differ" : result.error.c_str());
        rclcpp::shutdown(); return 1;
      }
      detected = result.size;
    }
    cfg.original_width = detected.width;
    cfg.original_height = detected.height;
    RCLCPP_INFO(node->get_logger(), "Camera size detected as %s",
                dvrk_video::describe(detected).c_str());
  }

  for (const auto &output : {cfg.left.gst_output, cfg.right.gst_output, cfg.stereo.gst_output}) {
    if (output.empty()) {
      continue;
    }
    if (dvrk_gst::resolve(output).empty()) {
      RCLCPP_ERROR(node->get_logger(), "Invalid gst_output socket reference: %s",
                   output.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "stereo gst output: %s", cfg.stereo.gst_output.c_str());
  if (!cfg.left.gst_output.empty())
    RCLCPP_INFO(node->get_logger(), "left gst output: %s", cfg.left.gst_output.c_str());
  if (!cfg.right.gst_output.empty())
    RCLCPP_INFO(node->get_logger(), "right gst output: %s", cfg.right.gst_output.c_str());

  dc_stereo::warn_if_interlaced_stream(cfg.left.gst_input, node->get_logger(),
                                       "camera.left");
  dc_stereo::warn_if_interlaced_stream(cfg.right.gst_input, node->get_logger(),
                                       "camera.right");

  const std::string pipeline_string = build_pipeline_string(cfg);
  if (!dc_stereo::validate_pipeline(pipeline_string, node->get_logger(),
                                    "stereo_source")) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "GStreamer pipeline string:\n%s",
              pipeline_string.c_str());

  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
  if (error != nullptr || pipeline == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create pipeline: %s",
                 error && error->message ? error->message : "unknown");
    if (error != nullptr) {
      g_error_free(error);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
    rclcpp::shutdown();
    return 1;
  }

  add_timestamp_probe(pipeline, "__left_src_q__", 0);
  add_timestamp_probe(pipeline, "__right_src_q__", 1);
  dvrk_video::LatestVideoSpec spec;
  spec.name = "stereo_mix";
  spec.width = 2 * cfg.original_width;
  spec.height = cfg.original_height;
  spec.stage = dvrk_video::OutputStage::Source;
  spec.inputs = {
      {"__latest_left__", 0, 0, cfg.original_width, cfg.original_height, dvrk_video::CameraContent::Left},
      {"__latest_right__", cfg.original_width, 0, cfg.original_width, cfg.original_height, dvrk_video::CameraContent::Right}};
  dvrk_video::LatestVideoMixers latest;
  std::string latest_error;
  if (!latest.start(pipeline, {spec}, latest_error)) {
    RCLCPP_ERROR(node->get_logger(), "%s", latest_error.c_str());
    gst_object_unref(pipeline); rclcpp::shutdown(); return 1;
  }

  std::atomic_bool video_caps_failed{false};
  auto make_monitor = [&](const char *side) {
    dvrk_video::CapsMonitor monitor;
    monitor.label = std::string("stereo_source ") + side + " input";
    if (!cfg.auto_camera_size) {
      monitor.expected = {cfg.original_width, cfg.original_height};
    }
    monitor.on_first = [&, side](dvrk_video::Size size) {
      RCLCPP_INFO(node->get_logger(), "%s input negotiated %s", side,
                  dvrk_video::describe(size).c_str());
    };
    monitor.on_error = [&](const std::string &message) {
      video_caps_failed = true;
      RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
      g_main_context_invoke(nullptr, [](gpointer) -> gboolean {
        if (dc_stereo::g_main_loop) g_main_loop_quit(dc_stereo::g_main_loop);
        return G_SOURCE_REMOVE;
      }, nullptr);
    };
    return monitor;
  };
  auto left_caps = make_monitor("left");
  auto right_caps = make_monitor("right");
  if (!dvrk_video::add_caps_monitor(pipeline, "__left_src_q__", &left_caps) ||
      !dvrk_video::add_caps_monitor(pipeline, "__right_src_q__", &right_caps)) {
    RCLCPP_ERROR(node->get_logger(), "Cannot monitor stereo source input caps");
    gst_object_unref(pipeline);
    rclcpp::shutdown();
    return 1;
  }

  const int status = dc_stereo::run_pipeline(
      pipeline, node, "stereo_source",
      "Stereo source background pipeline started");
  latest.stop();
  gst_object_unref(pipeline);
  rclcpp::shutdown();
  return video_caps_failed ? 1 : status;
}
