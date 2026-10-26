// SPDX-License-Identifier: BSD-2-Clause
// Copyright (c) 2026 Johns Hopkins University, LCSR
#include "pipeline.hpp"
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <dvrk_data/latest_video.hpp>
#include <filesystem>
#include <cmath>
#include <stdexcept>
#include <regex>

RectifyConfig load_rectify_config(const std::string &path) {
    Json::Value root;
    if (!dvrk_data::Config::load_from_file(path, root) ||
        !dvrk_data::Config::check_type(root, "dvrk_data:stereo_rectify@1.0.0", path))
        throw std::runtime_error("Cannot load stereo_rectify configuration");
    if (!root["calibration_file"].isString() || root["calibration_file"].asString().empty())
        throw std::runtime_error("calibration_file must name a stereo calibration YAML/JSON file");
    if (root.get("backend", "cpu").asString() != "cpu")
        throw std::runtime_error("Only the cpu rectification backend is available in this build");
    const Json::Value &settings = root;
    const auto &camera = settings["camera"];
    if (root.isMember("camera") && !camera.isObject())
        throw std::runtime_error("camera must be an object");
    if (camera["size"].isObject()) {
        const auto &size = camera["size"];
        if (!size["width"].isInt() || !size["height"].isInt() ||
            size["width"].asInt() < 2 || size["height"].asInt() < 2)
            throw std::runtime_error("camera.size must contain positive per-eye width and height");
    }
    if (root.isMember("gst_input") && (!root["gst_input"].isString() || root["gst_input"].asString().empty()))
        throw std::runtime_error("gst_input must be a nonempty string");
    if (root.isMember("gst_output") && !root["gst_output"].isString())
        throw std::runtime_error("gst_output must be a string");
    for (const auto *eye : {"left", "right"}) {
        if (camera.isMember(eye) && !camera[eye].isObject())
            throw std::runtime_error("camera eye endpoints must be objects");
        if (camera[eye].isMember("gst_input"))
            throw std::runtime_error("Per-eye inputs are unsupported; use the combined side-by-side gst_input");
        const auto &input = camera[eye]["gst_input"];
        if (camera[eye].isMember("gst_input") && (!input.isString() || input.asString().empty()))
            throw std::runtime_error("camera eye gst_input must be a nonempty string");
    }
    if (camera.isMember("crop") || camera.isMember("alignment") || root.isMember("preserve_size"))
        throw std::runtime_error("Rectification uses calibrated geometry; crop, alignment shifts and preserve_size belong to stereo_alignment");
    RectifyConfig config;
    config.streams = dvrk_data::Config::parse_app_config(root);
    auto calibration_path = std::filesystem::path(root["calibration_file"].asString());
    if (calibration_path.is_relative()) calibration_path = std::filesystem::absolute(path).parent_path() / calibration_path;
    config.calibration_file = calibration_path.lexically_normal().string();
    auto &cfg = config.streams;
    if (!cfg.left.gst_input.empty() || !cfg.right.gst_input.empty())
        throw std::runtime_error("Only combined side-by-side stereo input is supported; use gst_input or -i/--input");
    if (cfg.stereo.gst_input.empty())
        cfg.stereo.gst_input = dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "stereo");
    if (!cfg.stereo.gst_output_specified)
        cfg.stereo.gst_output = dvrk_gst::make(dvrk_gst::ROLE_STEREO_RECTIFY, "stereo");
    const std::regex socket_pattern("^@dvrk:[A-Za-z0-9_.-]+:[A-Za-z0-9_.-]+$");
    for (const auto &input : {cfg.stereo.gst_input, cfg.left.gst_input, cfg.right.gst_input}) {
        if (dvrk_gst::is_socket_reference(input) && !std::regex_match(input, socket_pattern))
            throw std::runtime_error("Invalid dVRK input socket: " + input);
    }
    if (!cfg.stereo.gst_output.empty() && !std::regex_match(cfg.stereo.gst_output, socket_pattern))
        throw std::runtime_error("gst_output must be a canonical @dvrk:role:name socket or empty");
    return config;
}

std::string build_rectify_pipeline(const RectifyConfig &config, cv::Size size) {
    const auto &cfg = config.streams;
    const std::string queue = " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream";
    std::string pipeline = dvrk_gst::build_input(cfg.stereo.gst_input, {}) +
        " ! queue name=stereo_input max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! tee name=raw ";
    const std::string left = "raw. " + queue + " ! videocrop right=" + std::to_string(size.width);
    const std::string right = "raw. " + queue + " ! videocrop left=" + std::to_string(size.width);
    auto branch = [&](const std::string &input, const std::string &eye,
                      const dvrk_data::ColorAdjustment &color) {
        return input + " ! queue name=" + eye + "_input max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream"
            " ! videoconvert ! videobalance brightness=" + std::to_string(color.brightness) +
            " contrast=" + std::to_string(color.contrast) + " saturation=" + std::to_string(color.saturation) +
            " hue=" + std::to_string(color.hue) +
            " ! videoconvert ! video/x-raw,format=BGR ! dvrkrectify name=" + eye + "_rectify right-eye=" +
            (eye == "right" ? "true" : "false") + " ! videoconvert ! video/x-raw,format=I420 ! " +
            dvrk_video::LatestVideoMixers::sink(eye + "_cache") + " ";
    };
    pipeline += branch(left, "left", cfg.left_color) + branch(right, "right", cfg.right_color);
    pipeline += dvrk_video::LatestVideoMixers::source("rectified") +
        " ! video/x-raw,format=I420,width=" + std::to_string(2 * size.width) + ",height=" + std::to_string(size.height);
    pipeline += queue + " ! " + (cfg.stereo.gst_output.empty() ? "fakesink sync=false" : dvrk_gst::build_sink(cfg.stereo.gst_output));
    return pipeline;
}
