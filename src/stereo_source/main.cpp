#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <iostream>
#include <string>

#include <dvrk_data/config.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <dvrk_data/stereo_common.hpp>

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
    timestamps.left_source_ts = now;
  } else {
    timestamps.right_source_ts = now;
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

std::string build_branch(const std::string &source,
                         const std::string &stream_name,
                         const std::string &queue_name,
                         const std::string &output) {
  std::string branch =
      source + " ! queue name=" + queue_name +
      " max-size-buffers=3 max-size-time=0 max-size-bytes=0 leaky=downstream";

  if (output.empty()) {
    branch += " ! fakesink sync=false";
  } else {
    const std::string abstract_name = dvrk_gst::resolve(output);
    branch += " ! videoconvert ! video/x-raw,format=I420"
              " ! queue name=__" +
              stream_name + "_output_q__ max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
              "leaky=downstream ! " + dvrk_gst::build_sink(abstract_name);
  }
  return branch;
}

std::string build_pipeline_string(const dvrk_data::AppConfig &cfg) {
  return build_branch(cfg.left.gst_input, "left", "__left_src_q__",
                      cfg.left.gst_output) +
         " " +
         build_branch(cfg.right.gst_input, "right", "__right_src_q__",
                      cfg.right.gst_output);
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

  if (!cfg.left.gst_output_specified)
    cfg.left.gst_output = dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "left");
  if (!cfg.right.gst_output_specified)
    cfg.right.gst_output = dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "right");

  cfg.left.gst_input = dvrk_gst::build_input(
      cfg.left.gst_input, dvrk_gst::ROLE_STEREO_SOURCE,
      cfg.original_width, cfg.original_height);
  cfg.right.gst_input = dvrk_gst::build_input(
      cfg.right.gst_input, dvrk_gst::ROLE_STEREO_SOURCE,
      cfg.original_width, cfg.original_height);

  for (const auto &output : {cfg.left.gst_output, cfg.right.gst_output}) {
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

  RCLCPP_INFO(node->get_logger(), "left gst output: %s", cfg.left.gst_output.c_str());
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

  const int status = dc_stereo::run_pipeline(
      pipeline, node, "stereo_source",
      "Stereo source background pipeline started");
  gst_object_unref(pipeline);
  rclcpp::shutdown();
  return status;
}
