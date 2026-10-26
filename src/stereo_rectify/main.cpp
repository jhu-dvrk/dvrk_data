// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#include "gst_rectify.hpp"
#include "pipeline.hpp"
#include <dvrk_data/color_adjustment.hpp>
#include <dvrk_data/calibration/rectification.hpp>
#include <dvrk_data/stereo_common.hpp>
#include <dvrk_data/video_caps.hpp>
#include <dvrk_data/latest_video.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <atomic>
#include <iostream>
#include "input_provenance.hpp"

int main(int argc, char **argv) {
    std::string path, input, color_file;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--ros-args") break;
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: stereo_rectify -c <config.json> [-i|--input <stereo>] [--color-adjustment <file.json>] [--ros-args ...]\n";
            return 0;
        }
        if ((arg == "-c" || arg == "--config") && i + 1 < argc && path.empty()) path = argv[++i];
        else if ((arg == "-i" || arg == "--input") && i + 1 < argc && input.empty()) input = argv[++i];
        else if (arg == "--color-adjustment" && i + 1 < argc && color_file.empty()) color_file = argv[++i];
        else { std::cerr << "Unexpected argument: " << arg << '\n'; return 1; }
    }
    if (path.empty()) { std::cerr << "Usage: stereo_rectify -c <config.json>\n"; return 1; }
    gst_init(nullptr, nullptr);
    RectifyConfig config;
    cv::Size size;
    try {
        config = load_rectify_config(path);
        if (!input.empty()) config.streams.stereo.gst_input = input;
        color_file = dvrk_data::load_stereo_color_adjustment(config.streams, path, color_file);
        dvrk_data::calibration::Rectification validation;
        std::string error;
        if (!validation.load(config.calibration_file, error)) throw std::runtime_error(error);
        size = validation.image_size();
        const auto &cfg = config.streams;
        if ((cfg.original_width || cfg.original_height) &&
            (cfg.original_width != size.width || cfg.original_height != size.height))
            throw std::runtime_error("camera.size does not match the calibration dimensions");
    } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("stereo_rectify");
    if (!register_rectify_element()) { rclcpp::shutdown(); return 1; }
    const auto description = build_rectify_pipeline(config, size);
    RCLCPP_INFO(node->get_logger(), "CPU rectification %dx%d per eye using %s", size.width, size.height, config.calibration_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Color adjustment: %s", color_file.empty() ? "neutral (no shared file)" : color_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Output: %s", config.streams.stereo.gst_output.empty() ? "disabled" : config.streams.stereo.gst_output.c_str());
    GError *error = nullptr;
    auto *pipeline = gst_parse_launch(description.c_str(), &error);
    if (!pipeline || error) {
        RCLCPP_ERROR(node->get_logger(), "Cannot construct pipeline: %s", error ? error->message : "unknown error");
        if (error) g_error_free(error);
        if (pipeline) gst_object_unref(pipeline);
        rclcpp::shutdown(); return 1;
    }
    for (const auto *eye : {"left_rectify", "right_rectify"}) {
        auto *element = gst_bin_get_by_name(GST_BIN(pipeline), eye);
        g_object_set(element, "calibration-file", config.calibration_file.c_str(), nullptr);
        gst_object_unref(element);
    }
    EyeInputState left_input{true}, right_input{false};
    add_probe(pipeline, "left_input", input_timestamp_probe_cb, &left_input);
    add_probe(pipeline, "right_input", input_timestamp_probe_cb, &right_input);
    dvrk_video::LatestVideoSpec spec;
    spec.name = "rectified"; spec.width = 2 * size.width; spec.height = size.height;
    // Preserve the processing-stage timestamp contract used by existing consumers.
    spec.stage = dvrk_video::OutputStage::Alignment;
    spec.inputs = {{"left_cache", 0, 0, size.width, size.height, dvrk_video::CameraContent::Left},
                   {"right_cache", size.width, 0, size.width, size.height, dvrk_video::CameraContent::Right}};
    dvrk_video::LatestVideoMixers mixer;
    std::string mixer_error;
    if (!mixer.start(pipeline, {spec}, mixer_error)) {
        RCLCPP_ERROR(node->get_logger(), "%s", mixer_error.c_str());
        gst_object_unref(pipeline); rclcpp::shutdown(); return 1;
    }
    std::atomic_bool invalid_caps{false};
    auto monitor = [&](const char *eye) {
        dvrk_video::CapsMonitor caps;
        caps.label = std::string("stereo_rectify ") + eye;
        caps.expected = {size.width, size.height};
        caps.on_error = [&](const std::string &message) {
            invalid_caps = true;
            RCLCPP_ERROR(node->get_logger(), "%s; use calibration for the incoming dimensions", message.c_str());
            // Always queue this callback: caps can fail during set_state(),
            // before run_pipeline has entered its main loop.
            g_idle_add([](gpointer) -> gboolean {
                if (dc_stereo::g_main_loop) g_main_loop_quit(dc_stereo::g_main_loop);
                return G_SOURCE_REMOVE;
            }, nullptr);
        };
        return caps;
    };
    auto left_caps = monitor("left"), right_caps = monitor("right");
    auto stereo_caps = monitor("stereo");
    stereo_caps.expected.width *= 2;
    if (!config.streams.stereo.gst_input.empty() &&
        !dvrk_video::add_caps_monitor(pipeline, "stereo_input", &stereo_caps)) {
        mixer.stop(); gst_object_unref(pipeline); rclcpp::shutdown(); return 1;
    }
    if (!dvrk_video::add_caps_monitor(pipeline, "left_input", &left_caps) ||
        !dvrk_video::add_caps_monitor(pipeline, "right_input", &right_caps)) {
        mixer.stop(); gst_object_unref(pipeline); rclcpp::shutdown(); return 1;
    }
    const int status = dc_stereo::run_pipeline(pipeline, node, "stereo_rectify", "Stereo rectification pipeline started");
    mixer.stop();
    gst_object_unref(pipeline);
    rclcpp::shutdown();
    return invalid_caps ? 1 : status;
}
